
/** @file
 * Implementation of the MAVLink communication infrastructure.
 */

/* *** Includes *** */

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
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

static FILE *comm0; ///< Stream of the MAVLINK_COMM_0 channel.
static mavlink_message_handler_t msg_handlers[255];


/* *** Prototypes *** */

/// MAVLink message handler for PARAM_SET message.
static void param_set_handler(mavlink_message_t *msg);

/// End a block of code in which a parameter will be updated.
static inline void update_parameter_end();

/// Start a block of code in which a parameter will be updated.
static inline void update_parameter_start();


/* *** Public functions *** */

ret_status_t
mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t* buff, 
			size_t len) {
  syslog(LOG_ERR, "Function mavlink_send_uart_bytes not yet implemented.",
	 __FILE__, __LINE__);
  return STATUS_FAILURE;
}

void
message_send_end() {
  sigset_t set;
  sigemptyset(&set);
  if (sigprocmask(SIG_UNBLOCK, &set, NULL))
    syslog(LOG_ERR, "Error unblocking signals: %m (%s)%d", __FILE__, __LINE__);
}

void
message_send_start() {
  sigset_t set;
  sigfillset(&set);
  if (sigprocmask(SIG_BLOCK, &set, NULL))
    syslog(LOG_ERR, "Error blocking signals: %m (%s)%d", __FILE__, __LINE__);  
}

mavlink_message_handler_t
register_message_handler(uint8_t msgid, mavlink_message_handler_t handler) {
  mavlink_message_handler_t old_handler = msg_handlers[msgid];
  msg_handlers[msgid] = handler;
  return old_handler;
}

void 
recv_comm() {
  mavlink_message_t msg;
  mavlink_status_t status;

  int data;
  for (int data = fgetc(comm0); data != EOF; data = fgetc(comm0)) {
    if(mavlink_parse_char(MAVLINK_COMM_0, data, &msg, &status)) {
      mavlink_message_handler_t handler = msg_handlers[msg.msgid];
      if (handler)
	handler(&msg);
    }
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
  
  //Set handler for PARAM_SET message.
  register_message_handler(MAVLINK_MSG_ID_PARAM_SET, &param_set_handler);

  return STATUS_SUCCESS;
}

void teardown_comm() {
  fclose(comm0);
}


/* *** Internal functions *** */

static void param_set_handler(mavlink_message_t *msg) {
  mavlink_param_set_t payload;
  mavlink_msg_param_set_decode(msg, &payload);

  if (payload.target_system != mavlink_system.sysid)
    return;
  
  update_parameter_start();
  update_parameter_end();

  syslog(LOG_ERR, "Function param_set_handler not yet implemented.",
	 __FILE__, __LINE__);
}

static inline void update_parameter_end() {
  sigset_t set;
  sigemptyset(&set);
  if (sigprocmask(SIG_UNBLOCK, &set, NULL))
    syslog(LOG_ERR, "Error unblocking signals: %m (%s)%d", __FILE__, __LINE__);
}

static inline void update_parameter_start() {
  sigset_t set;
  sigfillset(&set);
  if (sigprocmask(SIG_BLOCK, &set, NULL))
    syslog(LOG_ERR, "Error blocking signals: %m (%s)%d", __FILE__, __LINE__);
}
