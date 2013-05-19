
/** @file
 * Implementation of the MAVLink communication infrastructure.
 */

/* *** Includes *** */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

#include <libconfig.h>

#include "comm.h"
#include "mavlink_bridge.h"
#include "param.h"


/* *** Macros *** */

#ifndef COMM0_PATH
#define COMM0_PATH "/dev/ttyS2"
///< File path of the COMM0 mavlink channel.
#endif // not COMM0_PATH


/* *** Internal variables *** */

FILE *comm0; ///< Stream of the MAVLINK_COMM_0 channel.

/* *** Prototypes *** */

void handle_message(mavlink_channel_t chan, mavlink_message_t *msg);


/* *** Public functions *** */

ret_status_t
mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t* buff, 
			size_t len) {
  
  return STATUS_SUCCESS;
}

void recv_comm() {
  mavlink_message_t msg;
  mavlink_status_t status;

  int data;
  for (int data = fgetc(comm0); data != EOF; data = fgetc(comm0)) {
    if(mavlink_parse_char(MAVLINK_COMM_0, data, &msg, &status))
      handle_message(MAVLINK_COMM_0, &msg);
  }

  switch (errno){
  case EAGAIN:
  case EINTR:
    break;
    
  default:
    syslog(LOG_ERR, "Error during MAVLINK_COMM_0 read: %m (%s)%d", 
	   __FILE__, __LINE__);
  }
  
}

ret_status_t setup_comm() {
  //Open stream
  comm0 = fopen(COMM0_PATH, "r+");
  if (!comm0) {
    syslog(LOG_ERR, "Error opening COMM_0 mavlink channel at `%s': %m (%s)%d",
	   COMM0_PATH, __FILE__, __LINE__);
    
    return STATUS_FAILURE;
  }

  //Set stream for nonblocking operation
  fcntl(fileno(comm0), F_SETFL, O_NONBLOCK);
  
  return STATUS_SUCCESS;
}

void teardown_comm() {
  fclose(comm0);
}



void update_parameters() {
}


/* *** Internal functions *** */

void handle_message(mavlink_channel_t chan, mavlink_message_t *msg) {
}
