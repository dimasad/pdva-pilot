
/** @file
 * Implementation of the MAVLink communication infrastructure.
 */

/* *** Includes *** */

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "comm.h"
#include "mavlink_bridge.h"
#include "param.h"


/* *** Macros *** */

#ifndef FAKE_SPI
//#define FAKE_SPI
///< For tests which do not use SPI interface.
#endif // not FAKE_SPI

#ifndef RADIO_STREAM_PATH
//#define RADIO_STREAM_PATH "/dev/ttyS2"
#define RADIO_STREAM_PATH "/home/rafael/my_empty_file" //////////////////////
///< File path of the RADIO_COMM_CHANNEL.
#endif // not RADIO_STREAM_PATH

#ifndef SENSOR_HEAD_STREAM_PATH
#define SENSOR_HEAD_STREAM_PATH "/dev/spidev1.1"
///< File path of the SENSOR_HEAD_COMM_CHANNEL
#endif // not SENSOR_HEAD_STREAM_PATH

#ifndef SPI_MAX_SPEED_HZ
#define SPI_MAX_SPEED_HZ 500000
///< Maximum SPI transfer speed, in Hertz.
#endif // not SPI_MAX_SPEED_HZ


/* *** Internal variables *** */

static int radio; ///< File descriptor of the RADIO_COMM_CHANNEL.
static int sensor_head; ///< File descriptor of the SENSOR_HEAD_COMM_CHANNEL.
static size_t sensor_head_send_count = 0; ///< Amount of data in buffer.

/// Buffer for the outgoing data on the SENSOR_HEAD_COMM_CHANNEL
static uint8_t sensor_head_send_buffer[MAVLINK_MAX_PACKET_LEN];

static mavlink_message_handler_t msg_handlers[256];
///< Array with all registered message handlers, indexed by msgid.
uint32_t max_speed_hz;

/* *** Prototypes *** */

/// Send data over a MAVLink channel.
static inline void 
mavlink_send_uart_bytes(mavlink_channel_t, const uint8_t*, size_t);

/// Start a message send over a MAVLink channel.
static inline void
mavlink_start_uart_send(mavlink_channel_t chan, size_t len);

/// End a message send over a MAVLink channel.
static inline void
mavlink_end_uart_send(mavlink_channel_t chan, size_t len);

/// True if there is data available for reading in the RADIO_COMM_CHANNEL.
static inline bool
radio_data_available();

/// Read from the RADIO_COMM_CHANNEL.
static inline ret_status_t
radio_read(mavlink_message_t *msg, mavlink_status_t *status);

/// Send data over the RADIO_COMM_CHANNEL.
static inline void 
radio_send_bytes(const uint8_t*, size_t);

/// Send data over the SENSOR_HEAD_COMM_CHANNEL.
static inline void 
sensor_head_send_bytes(const uint8_t*, size_t);


/* *** Include MAVLink helper functions *** */

#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes
#include "mavlink/v1.0/mavlink_helpers.h"


/* *** Public functions *** */

void 
radio_handle_all() {
  mavlink_message_t msg;
  mavlink_status_t status;
  mavlink_message_handler_t handler;
  
  while (radio_data_available())
    if (radio_read(&msg, &status) == STATUS_SUCCESS)
      if (handler = msg_handlers[msg.msgid])
        handler(&msg);
}

void
radio_poll() {
  struct pollfd fds = {.fd = radio, .events = POLLIN};
  int polled = poll(&fds, 1, -1);
}


mavlink_message_handler_t
radio_register_handler(uint8_t msgid, mavlink_message_handler_t handler) {
  mavlink_message_handler_t old_handler = msg_handlers[msgid];
  msg_handlers[msgid] = handler;
  return old_handler;
}

ret_status_t
sensor_head_read(mavlink_sensor_head_data_t *payload) {
  //Read data from SENSOR_HEAD_COMM_CHANNEL
  uint8_t buf[MAVLINK_MSG_ID_SENSOR_HEAD_DATA_LEN 
	      + MAVLINK_NUM_NON_PAYLOAD_BYTES];
  mavlink_reset_channel_status(SENSOR_HEAD_COMM_CHANNEL);

  ssize_t len = read(sensor_head, &buf, sizeof buf);

  //Check if read successful
  if (len < 0) {
    syslog(LOG_DEBUG, "Error reading from sensor head: %m (%s)%d",
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }

  if (len < sizeof buf) {
    syslog(LOG_DEBUG, "Packet from sensor head too small (%s)%d",
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }
  
  //Decode message
  mavlink_message_t msg;
  mavlink_status_t status;
  printf("Message received at read() len=%d\n",len);
  //Parse the data
  for (int i = 0; i < sizeof buf; i++){if(i%10==0)printf("\n");printf("%d,",buf[i]);
    if(mavlink_parse_char(SENSOR_HEAD_COMM_CHANNEL, buf[i], &msg, &status)){
        //Retrieve the message payload and return
        mavlink_msg_sensor_head_data_decode(&msg, payload);
        return STATUS_SUCCESS;
    }
  }
  printf("\nEnd of message\n");
  syslog(LOG_DEBUG, "Could not decode message from sensor head (%s)%d",
	 __FILE__, __LINE__);
  return STATUS_SUCCESS;//return STATUS_FAILURE; ///////////*************
  
}

ret_status_t setup_comm() {
  //Open radio stream
  radio = open(RADIO_STREAM_PATH, O_RDWR);
  if (radio < 0) {
    syslog(LOG_ERR, "Error opening RADIO_COMM_CHANNEL stream at `%s': %m "
	   "(%s)%d", RADIO_STREAM_PATH, __FILE__, __LINE__);    
    return STATUS_FAILURE;
  }
  
  //Set radio stream for nonblocking operation
  fcntl(radio, F_SETFL, O_NONBLOCK);

  //Open sensor head stream
  sensor_head = open(SENSOR_HEAD_STREAM_PATH, O_RDWR | O_NONBLOCK); ///////////////////********
  if (sensor_head < 0) {
    syslog(LOG_ERR, "Error opening SENSOR_HEAD_COMM_CHANNEL stream at `%s': %m "
	   "(%s)%d", SENSOR_HEAD_STREAM_PATH, __FILE__, __LINE__);    
    return STATUS_FAILURE;
  }

  
#ifndef FAKE_SPI

  //Set SPI port parameters
  uint8_t mode = SPI_MODE_0;
  if (ioctl(sensor_head, SPI_IOC_WR_MODE, &mode))
    syslog(LOG_ERR, "Error setting SPI port mode: %m (%s)%d",
	   __FILE__, __LINE__);

  uint8_t lsb_first = false;
  if (ioctl(sensor_head, SPI_IOC_WR_LSB_FIRST, &lsb_first))
    syslog(LOG_ERR, "Error setting SPI port bit endianness: %m (%s)%d",
	   __FILE__, __LINE__);

  uint8_t bits_per_word = 8;
  if (ioctl(sensor_head, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word))
    syslog(LOG_ERR, "Error setting SPI port bits per word: %m (%s)%d",
	   __FILE__, __LINE__);
  
//  uint32_t max_speed_hz = SPI_MAX_SPEED_HZ;
printf("\nEnter SPI frequency (Hz): ");
scanf("%u", &max_speed_hz);
printf("Frequency %u Hz.\n", max_speed_hz);
  if (ioctl(sensor_head, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed_hz))
    syslog(LOG_ERR, "Error setting SPI port maximum speed: %m (%s)%d",
	   __FILE__, __LINE__);

#endif // not FAKE_SPI
  
  return STATUS_SUCCESS;
}

void teardown_comm() {
  //Close the RADIO_COMM_CHANNEL
  if (close(radio))
    syslog(LOG_ERR, "Error closing radio channel %m (%s)%d",
           __FILE__, __LINE__);

  //Close the SENSOR_HEAD_COMM_CHANNEL
  if (close(sensor_head))
    syslog(LOG_ERR, "Error closing sensor head channel %m (%s)%d", 
           __FILE__, __LINE__);
}


/* *** Internal functions *** */

static inline void
mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t* buff, 
			size_t len) {
  switch (chan) {
  case RADIO_COMM_CHANNEL:
    radio_send_bytes(buff, len);
    break;
  case SENSOR_HEAD_COMM_CHANNEL:
    sensor_head_send_bytes(buff, len);
    break;
  }
}

static inline void
radio_send_bytes(const uint8_t* buff, size_t len) {
  //Write data
  size_t written = write(radio, buff, len);
  
  //Treat error
  if (written != len) {
    switch (errno) {
    case EAGAIN:
      syslog(LOG_WARNING, "Message sending failed: write would block.");
      break;
    case EINTR:
      syslog(LOG_WARNING, "Message sending failed: system call interrupted.");
      break;
    default:
      syslog(LOG_ERR, "Message sending failed: %m (%s)%d.", __FILE__, __LINE__);
      break;      
    }
  }
}

static inline void
sensor_head_send_bytes(const uint8_t* buff, size_t len) {
  //See if there is space availabe in the buffer
  if (sensor_head_send_count + len > sizeof(sensor_head_send_buffer))
    return;

  //Copy data to buffer
  memcpy(sensor_head_send_buffer + sensor_head_send_count, buff, len);
  sensor_head_send_count += len;
}

static inline void
mavlink_start_uart_send(mavlink_channel_t chan, size_t len) {
  if (chan == SENSOR_HEAD_COMM_CHANNEL) {
    sensor_head_send_count = 0;
  }
}

static inline void
mavlink_end_uart_send(mavlink_channel_t chan, size_t len) {
  if (chan == SENSOR_HEAD_COMM_CHANNEL && len == sensor_head_send_count) {
    ssize_t written = write(sensor_head, sensor_head_send_buffer, len);
    printf("\nWrote %d bytes to sensor-head!!!\n", (int)written);
  }
}

static inline bool
radio_data_available() {
  struct pollfd fds = {.fd = radio, .events = POLLIN};
  int polled = poll(&fds, 1, 0);
  if (polled < 0) {
    syslog(LOG_ERR, "Error during poll: %m (%s)%d", __FILE__, __LINE__);
    return false;
  }

  return polled;
}

static inline ret_status_t
radio_read(mavlink_message_t *msg, mavlink_status_t *status) {
  uint8_t data;
  ssize_t num_read = read(radio, &data, 1);
  
  //Read from radio stream until it would block or an error occurs
  while (num_read == 1) {
    //Parse the character
    if(mavlink_parse_char(RADIO_COMM_CHANNEL, data, msg, status))
      return STATUS_SUCCESS;
    
    //Read next byte
    num_read = read(radio, &data, 1);
  }
  
  //Treat error condition
  if (num_read < 0) {
    switch (errno) {
    case EAGAIN: 
    case EINTR:
      break;
      
    default:
      syslog(LOG_ERR, "Error during RADIO_COMM_CHANNEL read: %m (%s)%d", 
	     __FILE__, __LINE__);
    }
  }
  
  return STATUS_FAILURE;
}
