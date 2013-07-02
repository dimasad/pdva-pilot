/* *** Includes *** */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

#include <libconfig.h>

#include "comm.h"
#include "control.h"
#include "mavlink_bridge.h"
#include "param.h"
#include "pdva-pilot.h"



/* *** Macros *** */

#ifndef PDVA_CONFIG_DIR
#define PDVA_CONFIG_DIR "/etc/pdva"
///< Path of the pdva-pilot configuration directory.
#endif // not PDVA_CONFIG_DIR

#ifndef LOG_PERROR
#define LOG_PERROR 0
///< Linux extension of syslog options that copies messages to stderr.
#endif// not LOG_PERROR


/* *** Variables *** */

mavlink_system_t mavlink_system;
pdva_pilot_config_t pdva_config;


/* *** Mavlink parameters *** */

float dummy1 = 0.1; ///< A dummy parameter for testing.
uint8_t dummy2 = 4; ///< Another dummy parameter.

param_handler_t param_handler; ///< The runtime parameter handler.


/* *** Prototypes *** */

ret_status_t setup();
void teardown();
void param_set_msg_handler(mavlink_message_t *msg);
void param_request_read_msg_handler(mavlink_message_t *msg);

/* *** Internal functions *** */

ret_status_t
setup() {
  //Configure logging
  openlog("pdva-pilot", LOG_CONS | LOG_PERROR | LOG_PID, LOG_USER);
  
  //Load pdva-pilot configuration
  pdva_config_init(&pdva_config);
  if (pdva_config_load(&pdva_config, PDVA_CONFIG_DIR "/pdva-pilot.cfg"))
    syslog(LOG_WARNING, "Could not load pdva-pilot configuration, "
	   "using default values.");
  
  //Set the mavlink_system structure
  mavlink_system.sysid = pdva_config.sysid;
  mavlink_system.compid = MAV_COMP_ID_SYSTEM_CONTROL;
  mavlink_system.type = MAV_TYPE_FIXED_WING;
  mavlink_system.state = MAV_STATE_BOOT;
  mavlink_system.mode = MAV_MODE_PREFLIGHT;
  mavlink_system.nav_mode = 0;
  
  //Setup parameter handler and register parameters
  param_handler_init(&param_handler, 2);
  param_register(&param_handler, MAV_PARAM_TYPE_REAL32, 
                 "dummy1", &dummy1, NULL, NULL);
  param_register(&param_handler, MAV_PARAM_TYPE_UINT8, 
                 "dummy2", &dummy2, NULL, NULL);
  
  //Load parameters from file
  if (param_load(&param_handler, PDVA_CONFIG_DIR "/mav_params.cfg"))
    syslog(LOG_WARNING, "Could not load parameters, using default values.");
  
  //Setup the communication module
  if (setup_comm()) {
    syslog(LOG_WARNING, "Could not setup the communication module.");
    return STATUS_FAILURE;
  }

  //Register radio message handlers
  radio_register_handler(MAVLINK_MSG_ID_PARAM_REQUEST_READ, 
                         &param_request_read_msg_handler);
  radio_register_handler(MAVLINK_MSG_ID_PARAM_SET, &param_set_msg_handler);

  //Setup the control module
  if (setup_control()) {
    syslog(LOG_ERR, "Could not setup the control module.");
    return STATUS_FAILURE;
  }
  
  return STATUS_SUCCESS;
}

void 
teardown() {
  teardown_control();
  teardown_comm();
  pdva_config_destroy(&pdva_config);
  closelog();
}

void param_request_read_msg_handler(mavlink_message_t *msg) {
  //Retrieve the message payload
  mavlink_param_request_read_t payload;    
  mavlink_msg_param_request_read_decode(msg, &payload);

  //See if we are the recipient of the message
  if (payload.target_system != mavlink_system.sysid &&
      payload.target_component != mavlink_system.compid)
    return;

  //Copy the id to an array which can hold the terminating nul byte
  char id[MAX_LENGTH_PARAM_ID + 1] = {0};
  strncpy(id, payload.param_id, MAX_LENGTH_PARAM_ID);
  
  //Lookup the parameter
  param_t *param = param_lookup(&param_handler, id, &payload.param_index);
  if (param == NULL)
    return;
  
  //Get and send the parameter value
  param_value_union_t val = param_get(param);
  mavlink_msg_param_value_send(RADIO_COMM_CHANNEL, id, val.param_float,
                               param->type, param_count(&param_handler),
                               payload.param_index);
}

void param_set_msg_handler(mavlink_message_t *msg) {
  //Retrieve the message payload
  mavlink_param_set_t payload;    
  mavlink_msg_param_set_decode(msg, &payload);

  //Check if we are the recipient of the message
  if (payload.target_system != mavlink_system.sysid &&
      payload.target_component != mavlink_system.compid)
    return;
  
  //Lookup the parameter
  int16_t index = -1;
  char id[MAX_LENGTH_PARAM_ID + 1] = {0};
  strncpy(id, payload.param_id, MAX_LENGTH_PARAM_ID);
  param_t *param = param_lookup(&param_handler, id, &index);
  if (param == NULL)
    return;

  //Block all signals
  sigset_t oldmask, newmask;
  sigfillset(&newmask);
  sigprocmask(SIG_SETMASK, &newmask, &oldmask);
  
  //Set the parameter
  param_value_union_t val = {.param_float = payload.param_value};
  ret_status_t set_status = param_set(param, val);
  
  //Restore the signal mask
  sigprocmask(SIG_SETMASK, &oldmask, NULL);

  //Broadcast the parameter
  if (set_status == STATUS_SUCCESS) {
    val = param_get(param);
    mavlink_msg_param_value_send(RADIO_COMM_CHANNEL, id, val.param_float,
                                 param->type, param_count(&param_handler),
                                 index);
  }
}

int
main(int argc, char* argv[]) {
  if (setup()) {
    syslog(LOG_CRIT, "Failure in pdva-pilot setup, aborting.");
    return EXIT_FAILURE;
  }
  
  start_control();
  while (true) {
    radio_poll();
    radio_handle_all();
  }

  teardown();
  return EXIT_SUCCESS;
}
