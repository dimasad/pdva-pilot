
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

#ifndef MAX_MAV_COMPONENTS
#define MAX_MAV_COMPONENTS 2
///< Maximum allowed number of MAVLink components.
#endif // not MAX_MAV_COMPONENTS


/* *** Types *** */

typedef struct mav_component {
  int compid;
  size_t param_count;
  param_def_t *param_defs;
} mav_component_t;


/* *** Internal variables *** */

static FILE *comm0; ///< Stream of the MAVLINK_COMM_0 channel.
static mavlink_message_handler_t msg_handlers[255];
mav_component_t mav_components[MAX_MAV_COMPONENTS];


/* *** Prototypes *** */

/// Find given parameter definition.
static param_def_t *
find_param_def(uint8_t compid, const char *param_id, int *param_index, 
	       int *param_count);

/// MAVLink message handler for PARAM_SET message.
static void param_set_handler(mavlink_message_t *msg);

/* *** Public functions *** */

void
mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t* buff, 
			size_t len) {
  if (chan != MAVLINK_COMM_0)
    syslog(LOG_ERR, "Error unblocking signals: %m (%s)%d", __FILE__, __LINE__);
  
  //Block all signals
  sigset_t set, old_set;
  sigfillset(&set);
  sigprocmask(SIG_BLOCK, &set, &old_set);

  //Write data
  size_t written = fwrite(buff, len, 1, comm0);

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

  //Restore blocked signals
  sigprocmask(SIG_SETMASK, &old_set, NULL);
}

void
message_send_end() {
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
register_mav_component(uint8_t compid, param_def_t *param_defs) {
  for (int i = 0; i < MAX_MAV_COMPONENTS; i++) {
    if (mav_components[i].compid == 0) {
      mav_components[i].compid = compid;
      mav_components[i].param_defs = param_defs;
      
      int j;
      for (j = 0; !IS_PARAM_DEF_LIST_END(param_defs[j]); j++);
      mav_components[i].param_count = j;
      
      return;
    }
  }
  
  syslog(LOG_ERR, "Maximum number of MAVLink components exceeded, could not "
	 "register component #%d (%s)%d", compid, __FILE__, __LINE__);
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
  //Close the COMM0 channel
  fclose(comm0);
}


/* *** Internal functions *** */

static void param_set_handler(mavlink_message_t *msg) {
  mavlink_param_set_t payload;
  mavlink_msg_param_set_decode(msg, &payload);

  if (payload.target_system != mavlink_system.sysid)
    return;

  int param_index, param_count;
  param_def_t *param_def = find_param_def(payload.target_component, 
					  payload.param_id, &param_index,
					  &param_count);

  if (param_def == NULL) {
    syslog(LOG_DEBUG, "Attempt to set unexistent parameter %.16s in "
	   "component #%d.", payload.param_id, payload.target_component);
    return;
  }

  if (param_def->type != payload.param_type) {
    syslog(LOG_DEBUG, "Attempt to set parameter %.16s to a value with the "
	   "wrong type.", payload.param_id);
    return;
  }
  
  param_value_union_t new_value = {.param_float = payload.param_value};

  //Block all signals
  sigset_t set, old_set;
  sigfillset(&set);
  sigprocmask(SIG_BLOCK, &set, &old_set);
  
  //Update the parameter
  if (update_param(param_def, new_value) == STATUS_SUCCESS)
    mavlink_msg_param_value_send(MAVLINK_COMM_0, payload.param_id, 
				 new_value.param_float, payload.param_type, 
				 param_count, param_index);
  
  //Restore blocked signals
  sigprocmask(SIG_SETMASK, &old_set, NULL);
}

static param_def_t *
find_param_def(uint8_t compid, const char *param_id, int *param_index, 
	       int *param_count) {
  //Transverse all components
  for (int i = 0; i < MAX_MAV_COMPONENTS; i++) {
    mav_component_t *comp = mav_components + i;
    
    //Check if the component id matches
    if (comp->compid == compid) {
      param_def_t *param_defs = comp->param_defs;

      //Transverse all parameters
      for (int j = 0; j < comp->param_count; j++) {
	if (strncmp(param_defs[j].id, param_id, sizeof param_defs->id) == 0) {
	  *param_index = j;
	  *param_count = comp->param_count;
	  return param_defs + j;
	}
      }
    }
    
    //Check end of registered parameters
    if (comp->compid == 0)
      return NULL;
  }
}
