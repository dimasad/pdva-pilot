#ifndef PDVA__COMM_H
#define PDVA__COMM_H

/** @file
 * Contains the MAVLink communication infrastructure.
 */

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* *** Includes *** */

#include "mavlink_bridge.h"
#include "param.h"
#include "pdva-pilot.h"

/* *** Macros *** */

#ifndef SPI_MAX_SPEED_HZ
#define SPI_MAX_SPEED_HZ 500000
///< Maximum SPI transfer speed, in Hertz.
#endif // not SPI_MAX_SPEED_HZ


/* *** Types *** */

typedef void (*mavlink_message_handler_t)(mavlink_message_t *);


/* *** Global variables *** */

extern mavlink_system_t mavlink_system;
///< Stores the identification and status of the mavlink system/component.

/// Configuration structure.
extern pdva_pilot_config_t pdva_config;

/* *** Functions *** */

/// Handle all available messages on the RADIO_COMM_CHANNEL.
void 
radio_handle_all();

/// Block until data is available in the radio channel or a signal arrives.
void
radio_poll();

/// Register a mavlink message handler.
/// @return The previous registered handler.
mavlink_message_handler_t
radio_register_handler(uint8_t msgid, mavlink_message_handler_t handler);

/// Get sensor head data from SENSOR_HEAD_COMM_CHANNEL.
ret_status_t
sensor_head_read_write(mavlink_sensor_head_data_t*, uint8_t *,
                       mavlink_sensor_head_command_t*);

/// Setup the communication module.
ret_status_t
setup_comm();

/// Free resources associated with the communication module.
void
teardown_comm();


#ifdef __cplusplus
}
#endif //__cplusplus

#endif // not PDVA__COMM_H
