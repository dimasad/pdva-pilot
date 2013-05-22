#ifndef PDVA__COMM_H
#define PDVA__COMM_H

/** @file
 * Contains the MAVLink communication infrastructure.
 */


/* *** Includes *** */

#include "param.h"
#include "pdva-pilot.h"


/* *** Types *** */

typedef void (*mavlink_message_handler_t)(mavlink_message_t *);


/* *** Functions *** */

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

/// Setup the communication module.
ret_status_t setup_comm();

/// Free resources associated with the communication module.
void teardown_comm();



#endif // not PDVA__COMM_H
