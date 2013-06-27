#ifndef PDVA__COMM_H
#define PDVA__COMM_H

/** @file
 * Contains the MAVLink communication infrastructure.
 */


/* *** Includes *** */

#include "mavlink_bridge.h"
#include "param.h"
#include "pdva-pilot.h"


/* *** Types *** */

typedef void (*mavlink_message_handler_t)(mavlink_message_t *);


/* *** Global variables *** */

extern mavlink_system_t mavlink_system;
///< Stores the identification and status of the mavlink system/component.


/* *** Functions *** */
/*
/// Announce parameters as a response to PARAM_REQUEST_LIST message.
void param_announce();

/// Register a mavlink message handler.
/// @return The previous handler.
mavlink_message_handler_t
register_message_handler(uint8_t msgid, mavlink_message_handler_t handler);

/// Read from communication buffers.
void recv_comm();

/// Register a MAVLink component.
void register_mav_component(uint8_t compid, param_def_t *params_def);
*/

/// Get sensor head data from SENSOR_HEAD_COMM_CHANNEL.
ret_status_t
read_sensor_head(mavlink_sensor_head_data_t*);

/// Setup the communication module.
ret_status_t setup_comm();

/// Free resources associated with the communication module.
void teardown_comm();


#endif // not PDVA__COMM_H