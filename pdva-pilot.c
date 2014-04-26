/* *** Includes *** */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/time.h>
#include <pthread.h>

#include <libconfig.h>

#include "comm.h"
#include "control.h"
#include "datalog.h"
#include "mavlink_bridge.h"
#include "param.h"
#include "pdva-pilot.h"



/* *** Macros *** */

#ifndef PARAM_LIST_SEND_INTERVAL_S
#define PARAM_LIST_SEND_INTERVAL_S 0.25
///< Interval in seconds for sending parameters of a PARAM_REQUEST_LIST.
#endif // not PARAM_LIST_SEND_INTERVAL_S

#ifndef PDVA_CONFIG_DIR
#define PDVA_CONFIG_DIR "/etc/pdva"
///< Path of the pdva-pilot configuration directory.
#endif // not PDVA_CONFIG_DIR

#ifndef LOG_PERROR
#define LOG_PERROR 0
///< Linux extension of syslog options that copies messages to stderr.
#endif// not LOG_PERROR


/* *** Variables *** */

pthread_t datalog_thread;

mavlink_system_t mavlink_system;
pdva_pilot_config_t pdva_config;

struct {
  struct timeval last_sent;
  int next_index;
} param_list_queue = {.last_sent = {0,0}, .next_index = -1};

struct {
  uint16_t imu_raw;
  uint16_t gps_raw;
  uint16_t pressure_raw;
  uint16_t attitude;
} telemetry_downsample = {0,0,0,0};


/* *** Mavlink parameters *** */

float dummy1 = 0.1; ///< A dummy parameter for testing.
uint8_t dummy2 = 4; ///< Another dummy parameter.

param_handler_t param_handler; ///< The runtime parameter handler.


/* *** Prototypes *** */

ret_status_t setup();
void teardown();

/// Send parameters queued by PARAM_REQUEST_LIST MAVLink message.
void param_list_send_queued();

/// Handler for PARAM_REQUEST_LIST MAVLink message.
void param_request_list_msg_handler(mavlink_message_t *msg);

/// Handler for PARAM_REQUEST_READ MAVLink message.
void param_request_read_msg_handler(mavlink_message_t *msg);

/// Handler for PARAM_SET MAVLink message.
void param_set_msg_handler(mavlink_message_t *msg);

/// Handler for REQUEST_DATA_STREAM MAVLink message.
void request_data_stream_msg_handler(mavlink_message_t *msg);

/// Send all telemetry messages currently on.
void telemetry_send_all();

/// Creates a new thread with the specified priority.
int create_thread_with_priority(pthread_t * thread,
       void *(*start_routine) (void *), int priority);

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
  radio_register_handler(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, 
                         &param_request_list_msg_handler);
  radio_register_handler(MAVLINK_MSG_ID_PARAM_REQUEST_READ, 
                         &param_request_read_msg_handler);
  radio_register_handler(MAVLINK_MSG_ID_PARAM_SET, 
                         &param_set_msg_handler);
  radio_register_handler(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, 
                         &request_data_stream_msg_handler);

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

void param_list_send_queued() {
  if (param_list_queue.next_index < 0)
    return;
  
  //Get current time
  struct timeval now;
  gettimeofday(&now, NULL);
  
  //Calculate the elapsed time since last parameter was sent
  double dt_secs = now.tv_sec - param_list_queue.last_sent.tv_sec;
  dt_secs += 1e-6*(now.tv_usec - param_list_queue.last_sent.tv_usec);
  
  if (dt_secs >= PARAM_LIST_SEND_INTERVAL_S) {
    //Lookup the parameter
    int16_t index = param_list_queue.next_index++;
    param_t *param = param_lookup(&param_handler, "", &index);
    if (param == NULL) {
      param_list_queue.next_index = -1;
      return;
    }
  
    //Get and send the parameter value
    param_value_union_t val = param_get(param);
    char id[MAX_LENGTH_PARAM_ID] = {0};
    strncpy(id, param->id, sizeof id);
    mavlink_msg_param_value_send(RADIO_COMM_CHANNEL, id, val.param_float,
                                 param->type, param_count(&param_handler),
                                 index);
    
    //Save the time the parameter was sent
    memcpy(&param_list_queue.last_sent, &now, sizeof(struct timeval));
  }
}

void param_request_list_msg_handler(mavlink_message_t *msg) {
  //Retrieve the message payload
  mavlink_param_request_list_t payload;    
  mavlink_msg_param_request_list_decode(msg, &payload);

  //See if we are the recipient of the message
  if (payload.target_system != mavlink_system.sysid &&
      payload.target_component != mavlink_system.compid)
    return;

  //Set the next_index to the start of the list
  param_list_queue.next_index = 0;
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

void request_data_stream_msg_handler(mavlink_message_t *msg) {
  //Retrieve the message payload
  mavlink_request_data_stream_t payload;
  mavlink_msg_request_data_stream_decode(msg, &payload);
  
  //Check if we are the recipient of the message
  if (payload.target_system != mavlink_system.sysid &&
      payload.target_component != mavlink_system.compid)
    return;
  
  uint16_t downrate = (payload.start_stop == 1) * payload.req_stream_id;
  
  switch (payload.req_stream_id) {
  case MAV_DATA_STREAM_RAW_SENSORS:
    telemetry_downsample.imu_raw = downrate;
    telemetry_downsample.gps_raw = downrate;
    telemetry_downsample.pressure_raw = downrate;
    
  case MAV_DATA_STREAM_ALL:
    telemetry_downsample.imu_raw = downrate;
    telemetry_downsample.gps_raw = downrate;
    telemetry_downsample.pressure_raw = downrate;
    telemetry_downsample.attitude = downrate;
  }
}

void
telemetry_send_all() {
  //Get the sensor data and tick count
  mavlink_sensor_head_data_t data;
  mavlink_sensor_head_command_t control_data;
  get_sensor_and_control_data(&data, &control_data);
  unsigned ticks = control_loop_ticks();
  
  if (telemetry_downsample.imu_raw && 
      ticks % telemetry_downsample.imu_raw == 0) {
    mavlink_msg_raw_imu_send(RADIO_COMM_CHANNEL, data.time_gps_ms*1000,
                             data.acc[0], data.acc[1], data.acc[2],
                             data.gyro[0], data.gyro[1], data.gyro[2],
                             data.mag[0], data.mag[1], data.mag[2]);
  }
  
  if (telemetry_downsample.gps_raw && 
      ticks % telemetry_downsample.gps_raw == 0) {
    mavlink_msg_gps_raw_int_send(RADIO_COMM_CHANNEL, data.time_gps_ms*1000,
                                 3, data.lat_gps, data.lon_gps, data.alt_gps,
                                 UINT16_MAX, UINT16_MAX, data.speed_gps,
                                 data.hdg_gps * 180 / M_PI / 10, 255);
  }

  if (telemetry_downsample.pressure_raw && 
      ticks % telemetry_downsample.pressure_raw == 0) {
    mavlink_msg_raw_pressure_send(RADIO_COMM_CHANNEL, data.time_gps_ms,
                                  data.stat_press, data.dyn_press, 0, 0);
  }

  if (telemetry_downsample.attitude && 
      ticks % telemetry_downsample.attitude == 0) {
    mavlink_msg_attitude_send(RADIO_COMM_CHANNEL, data.time_gps_ms,
                              data.att_est[0], data.att_est[1], data.att_est[2],
                              0, 0, 0);
  }
}


int
create_thread_with_priority(pthread_t * thread,
       void *(*start_routine) (void *), int priority) {

  pthread_attr_t attr;

  if(pthread_attr_init(&attr)){
    syslog(LOG_ERR, "Could not initialize thread attributes.");
    return STATUS_FAILURE;
  }
  if(pthread_attr_setschedpolicy(&attr, SCHED_OTHER)){/////////////////////SCHED_RR
    syslog(LOG_ERR, "Could not set thread scheduling policy.");
    return STATUS_FAILURE;
  }
  struct sched_param prio = {.sched_priority = priority};
  if(pthread_attr_setschedparam(&attr, &prio)){
    syslog(LOG_ERR, "Could not set thread priority.");
    return STATUS_FAILURE;
  }
  if(pthread_create(thread, &attr, start_routine, NULL)){
    syslog(LOG_ERR, "Could not create datalog thread.");
    return STATUS_FAILURE;
  }
  if(pthread_attr_destroy(&attr)){
    syslog(LOG_ERR, "Could not destroy datalog thread attributes.");
    return STATUS_FAILURE;
  }
}

int
main(int argc, char* argv[]) {
  if (setup()) {
    syslog(LOG_CRIT, "Failure in pdva-pilot setup, aborting.");
    return STATUS_FAILURE;
  }

  pthread_t main_thread = pthread_self();
  struct sched_param prio = {.sched_priority = 0};////////////////////3
  if(pthread_setschedparam(main_thread, SCHED_OTHER, &prio)){///////////////////////SCHED_RR
    syslog(LOG_ERR, "Could not set main thread priority.");
    return STATUS_FAILURE;
  }

  start_control();

  if(create_thread_with_priority(&datalog_thread, &datalogging, 0)){////////////1
    syslog(LOG_ERR, "Error creating datalog thread.");
    return STATUS_FAILURE;
  }

  while (true) {
    radio_poll();
    radio_handle_all();
    param_list_send_queued();
    telemetry_send_all();
  }

  teardown();
  void *retval;
  if(pthread_join(datalog_thread, &retval)){
    syslog(LOG_ERR, "Could not join datalog thread.");
    return STATUS_FAILURE;
  }
  if(retval){
    syslog(LOG_ERR, "Datalog thread exited with nonzero status.");
    return STATUS_FAILURE;
  }

  return STATUS_SUCCESS;
}
