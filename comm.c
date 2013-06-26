
/** @file
 * Implementation of the MAVLink communication infrastructure.
 */

/* *** Includes *** */

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
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

#ifndef RADIO_STREAM_PATH
#define RADIO_STREAM_PATH "/dev/ttyS2"
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

/*
static mavlink_message_handler_t msg_handlers[256];
///< Array with all registered message handlers, indexed by msgid.

mav_component_t mav_components[MAX_MAV_COMPONENTS];
///< Array with all registered MAVLink components, compid == 0 if slot free.

int announce_target_component = -1;
int announce_mav_component_index;
int announce_param_index;

/* *** Prototypes *** */

/// Send data over a MAVLink channel.
static inline void 
mavlink_send_uart_bytes(mavlink_channel_t, const uint8_t*, size_t);

/// Send data over the RADIO_COMM_CHANNEL.
static inline void 
radio_send_bytes(const uint8_t*, size_t);

/// Send data over the SENSOR_HEAD_COMM_CHANNEL.
static inline void 
sensor_head_send_bytes(const uint8_t*, size_t);

/// Start a message send over a MAVLink channel.
static inline void
mavlink_start_uart_send(mavlink_channel_t chan, size_t len);

/// End a message send over a MAVLink channel.
static inline void
mavlink_end_uart_send(mavlink_channel_t chan, size_t len);


/*
/// Find given parameter definition.
static param_def_t *
find_param_def(uint8_t compid, const char *param_id, int *param_index, 
	       int *param_count);

/// MAVLink message handler for PARAM_REQUEST_LIST message.
static void param_request_list_handler(mavlink_message_t *msg);

/// MAVLink message handler for PARAM_SET message.
static void param_set_handler(mavlink_message_t *msg);


/* *** Include MAVLink helper functions *** */

#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes
#include "mavlink/v1.0/mavlink_helpers.h"


/* *** Public functions *** */

/*
void 
param_announce() {
  //See if there is need for announcement
  if (announce_target_component < 0) 
    return;
  
  //Get the current component and parameter definition
  mav_component_t *comp = mav_components + announce_mav_component_index;
  param_def_t *param_def = comp->param_defs + announce_param_index;

  //Create the PARAM_VALUE message
  mavlink_message_t msg;    
  mavlink_msg_param_value_pack(pdva_config.sysid, comp->compid, &msg, 
			       param_def->id, param_get(param_def).param_float,
			       param_def->type, comp->param_count, 
			       announce_param_index);
  
  //Send the message
  uint8_t buf[MAVLINK_MSG_ID_PARAM_VALUE_LEN];
  size_t len = mavlink_msg_to_send_buffer(buf, &msg);
  mavlink_send_uart_bytes(RADIO_COMM_CHANNEL, buf, len);
  
  //Move to next parameter and check end of parameter list
  if (++announce_param_index >= comp->param_count) {
    //Check if more components are to be announced
    if (announce_target_component != 0)
      announce_target_component = -1;
    else {
      //Move to next component and check for end of component list
      if (++announce_mav_component_index >= MAX_MAV_COMPONENTS)
	announce_target_component = -1;
    }
  }
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

  //Read from stream until it would block or an error occurs
  for (int data = fgetc(radio); data != EOF; data = fgetc(radio)) {
    
    if(mavlink_parse_char(RADIO_COMM_CHANNEL, data, &msg, &status)) {
      //Retrieve message handler
      mavlink_message_handler_t handler = msg_handlers[msg.msgid];
      
      //Handle the message if a handler is registered
      if (handler)
	handler(&msg);
    }
  }
  
  //Treat error condition
  if (ferror(radio)) {
    switch (errno) {
    case EAGAIN: 
    case EINTR:
      break;
      
    default:
      syslog(LOG_ERR, "Error during RADIO_COMM_CHANNEL read: %m (%s)%d", 
	     __FILE__, __LINE__);
    }
  }

  //Clear the end-of-file and error indicators
  clearerr(radio);
}
*/

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
  sensor_head = open(SENSOR_HEAD_STREAM_PATH, O_RDWR);
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
  
  uint32_t max_speed_hz = SPI_MAX_SPEED_HZ;
  if (ioctl(sensor_head, SPI_IOC_WR_MODE, &max_speed_hz))
    syslog(LOG_ERR, "Error setting SPI port maximum speed: %m (%s)%d",
	   __FILE__, __LINE__);

#endif // not FAKE_SPI
  
  return STATUS_SUCCESS;
}

void teardown_comm() {
  //Close the RADIO_COMM_CHANNEL
  close(radio);

  //Close the SENSOR_HEAD_COMM_CHANNEL
  close(sensor_head);
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
    write(sensor_head, sensor_head_send_buffer, len);
  }
}

/*
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

static void param_request_list_handler(mavlink_message_t *msg) {
  //Retrieve the message payload
  mavlink_param_request_list_t payload;
  mavlink_msg_param_request_list_decode(msg, &payload);

  //Check if we are the target system
  if (payload.target_system != pdva_config.sysid)
    return;

  
  if (payload.target_component == 0) {
    //Announce from all components
    announce_target_component = 0;
    announce_mav_component_index = 0;
    announce_param_index = 0;
    return;
  }
  
  //Find the index of the target component
  for (int i = 0; i < MAX_MAV_COMPONENTS; i++) {
    if (mav_components[i].compid == payload.target_component) {
      announce_target_component = payload.target_component;
      announce_mav_component_index = i;
      announce_param_index = 0;
      return;
    }
  }
    
  //Could not find target component, do nothing.
  syslog(LOG_DEBUG, "Attempt to request parameter list for inexistent "
	 "component #%d.", payload.target_component);
}

static void param_set_handler(mavlink_message_t *msg) {
  //Retrieve the message payload
  mavlink_param_set_t payload;
  mavlink_msg_param_set_decode(msg, &payload);

  //Check if we are the target system
  if (payload.target_system != pdva_config.sysid)
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
  
  //Update the parameter and broadcast the new value
  if (update_param(param_def, new_value) == STATUS_SUCCESS) {
    mavlink_message_t msg;    
    mavlink_msg_param_value_pack(pdva_config.sysid, payload.target_component,
				 &msg, payload.param_id,
				 param_get(param_def).param_float,
				 payload.param_type, param_count, param_index);

    uint8_t buf[MAVLINK_MSG_ID_PARAM_VALUE_LEN];
    size_t len = mavlink_msg_to_send_buffer(buf, &msg);
    mavlink_send_uart_bytes(RADIO_COMM_CHANNEL, buf, len);
  }
  
  //Restore blocked signals
  sigprocmask(SIG_SETMASK, &old_set, NULL);
}
*/
