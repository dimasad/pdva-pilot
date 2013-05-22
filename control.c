
/** @file
 * Implementation of the feedback control.
 */

/* *** Includes *** */

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>
#include <sys/ioctl.h>
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


/* *** Prototypes *** */

static inline ret_status_t read_sensor_head();


/* *** Internal variables *** */

static mavlink_sensor_head_data_t sensor_data;
static int spi;


/* *** Public functions *** */

ret_status_t 
setup_control() {
  //Open SPI port
  spi = open(SPI_STREAM_PATH, O_RDWR);
  if (spi < 0) {
    syslog(LOG_CRIT, "Error opening SPI stream at `%s': %m (%s)%d",
	   SPI_STREAM_PATH, __FILE__, __LINE__);
  }
  
#ifdef FAKE_SPI

  //Set stream for nonblocking operation
  fcntl(spi, F_SETFL, O_NONBLOCK);

#else // not FAKE_SPI

  //Set SPI port parameters
  uint8_t mode = SPI_MODE_0;
  if (ioctl(spi, SPI_IOC_WR_MODE, &mode))
    syslog(LOG_ERR, "Error setting SPI port parameter: %m (%s)%d",
	   __FILE__, __LINE__);

  uint8_t lsb_first = false;
  if (ioctl(spi, SPI_IOC_WR_LSB_FIRST, &lsb_first))
    syslog(LOG_ERR, "Error setting SPI port parameter: %m (%s)%d",
	   __FILE__, __LINE__);

  uint8_t bits_per_word = 8;
  if (ioctl(spi, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word))
    syslog(LOG_ERR, "Error setting SPI port parameter: %m (%s)%d",
	   __FILE__, __LINE__);
  
  uint32_t max_speed_hz = SPI_MAX_SPEED_HZ;
  if (ioctl(spi, SPI_IOC_WR_MODE, &max_speed_hz))
    syslog(LOG_ERR, "Error setting SPI port parameter: %m (%s)%d",
	   __FILE__, __LINE__);

#endif // not FAKE_SPI
  
  return STATUS_SUCCESS;
}

void 
teardown_control() {
}


/* *** Internal functions *** */

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
