
/** @file
 * Implementation of the feedback control.
 */

/* *** Includes *** */

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "mavlink_bridge.h"
#include "param.h"


/* *** Macros *** */

#ifndef SPI_STREAM_PATH
#define SPI_STREAM_PATH "/dev/spidev1.1"
///< File path of the SENSOR_HEAD_COMM_CHANNEL
#endif // not SPI_STREAM_PATH

#ifndef SPI_MAX_SPEED_HZ
#define SPI_MAX_SPEED_HZ 500000
///< Maximum SPI transfer speed, in Hertz.
#endif // not SPI_MAX_SPEED_HZ

#ifndef CONTROL_TIMER_PERIOD_NS
#define CONTROL_TIMER_PERIOD_NS 500000000L
///< Period of the control loop in nanosecods.
#endif // not CONTROL_TIMER_PERIOD_NS

/* *** Prototypes *** */

static void alarm_handler(int signum, siginfo_t *info, void *context);
static inline void calculate_control();
static inline ret_status_t read_sensor_head();
static inline ret_status_t write_control();


/* *** Internal variables *** */

/// Latest sensor data.
static mavlink_sensor_head_data_t sensor_data; 

/// The current control action.
static mavlink_servo_output_raw_t output; 

/// File descriptor of spi port stream.
static int spi;

/// Control loop timer.
static timer_t timer;


/* *** Public functions *** */

ret_status_t 
setup_control() {
  //Open SPI port
  spi = open(SPI_STREAM_PATH, O_RDWR);
  if (spi < 0) {
    syslog(LOG_CRIT, "Error opening SPI stream at `%s': %m (%s)%d",
	   SPI_STREAM_PATH, __FILE__, __LINE__);
  }
  
#ifndef FAKE_SPI

  //Set SPI port parameters
  uint8_t mode = SPI_MODE_0;
  if (ioctl(spi, SPI_IOC_WR_MODE, &mode))
    syslog(LOG_ERR, "Error setting SPI port mode: %m (%s)%d",
	   __FILE__, __LINE__);

  uint8_t lsb_first = false;
  if (ioctl(spi, SPI_IOC_WR_LSB_FIRST, &lsb_first))
    syslog(LOG_ERR, "Error setting SPI port bit endianness: %m (%s)%d",
	   __FILE__, __LINE__);

  uint8_t bits_per_word = 8;
  if (ioctl(spi, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word))
    syslog(LOG_ERR, "Error setting SPI port bits per word: %m (%s)%d",
	   __FILE__, __LINE__);
  
  uint32_t max_speed_hz = SPI_MAX_SPEED_HZ;
  if (ioctl(spi, SPI_IOC_WR_MODE, &max_speed_hz))
    syslog(LOG_ERR, "Error setting SPI port maximum speed: %m (%s)%d",
	   __FILE__, __LINE__);

#endif // not FAKE_SPI
  
  //Setup the interrupt handler
  struct sigaction alarm_action;
  alarm_action.sa_sigaction = &alarm_handler;
  alarm_action.sa_flags = SA_RESTART;
  sigfillset(&alarm_action.sa_mask);
  sigaction(SIGALRM, &alarm_action, NULL);
    

  //Create timer
  if (timer_create(CLOCK_REALTIME, NULL, &timer)) {
    syslog(LOG_ERR, "Error creating control loop timer: %m (%s)%d",
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }

  return STATUS_SUCCESS;
}

ret_status_t
start_control() {
  struct itimerspec timer_spec;
  timer_spec.it_interval.tv_sec = 0;
  timer_spec.it_value.tv_sec = 0;
  timer_spec.it_interval.tv_nsec = CONTROL_TIMER_PERIOD_NS;
  timer_spec.it_value.tv_nsec = CONTROL_TIMER_PERIOD_NS;
  
  if (timer_settime(timer, 0, &timer_spec, NULL)) {
    syslog(LOG_ERR, "Error setting control loop timer: %m (%s)%d.",
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }
  
  return STATUS_SUCCESS;
}

void 
teardown_control() {
  close(spi);
}


/* *** Internal functions *** */

static void 
alarm_handler(int signum, siginfo_t *info, void *context) {
  if (signum != SIGALRM)
    return;

  //Get readings from sensor head
  if (read_sensor_head()) {
    //How to proceed when failed to obtain sensor head measurements?
  }
  
  //Calculate the control action
  calculate_control();

  //Write control action to the SPI port
  if (write_control()) {
    //How to proceed when failed send control action?
  }  
}

static inline void 
calculate_control() {
}

static inline ret_status_t
read_sensor_head() {
  //Read data from spi
  uint8_t buf[MAVLINK_MSG_ID_SENSOR_HEAD_DATA_LEN];
  ssize_t len = read(spi, &buf, sizeof buf);

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
  
  //Read from stream until it would block or an error occurs
  for (int i = 0; i < sizeof buf; i++)
    mavlink_parse_char(SENSOR_HEAD_COMM_CHANNEL, buf[i], &msg, &status);
  
  if (!status.msg_received) {
    syslog(LOG_DEBUG, "Could not decode message from sensor head (%s)%d",
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }
  
  //Retrieve the message payload
  mavlink_msg_sensor_head_data_decode(&msg, &sensor_data);
  
  return STATUS_SUCCESS;
}

static inline ret_status_t write_control() {
  //Encode the MAVLink message
  mavlink_message_t msg;    
  mavlink_msg_servo_output_raw_encode(pdva_config.sysid,
				      mavlink_system.compid, &msg,
				      &output);
  
  //Write to buffer
  uint8_t buf[MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN];
  size_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  //Write buffer to SPI port
  ssize_t written = write(spi, buf, len);

  //Check for error during send
  if (written < 0) {
    syslog(LOG_ERR, "Error writing on SPI port: %m (%s)%d", __FILE__, __LINE__);
    return STATUS_FAILURE;
  }
  
  //Check if the message was completely written
  if (written < len) {
    syslog(LOG_ERR, "Could not write full message to SPI port (%s)%d", 
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }

  return STATUS_SUCCESS;
}
