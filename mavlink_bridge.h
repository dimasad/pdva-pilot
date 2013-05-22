#ifndef PDVA__MAVLINK_BRIDGE_H
#define PDVA__MAVLINK_BRIDGE_H

/** @file
 * MAVLink adapter header.
 * Include this file instead of "mavlink.h".
 */


#include <stddef.h>
#include <stdint.h>

#include "pdva-pilot.h"

#define MAVLINK_COMM_NUM_BUFFERS 2
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes
#define MAVLINK_SEPARATE_HELPERS
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS


#include "mavlink/v1.0/mavlink_types.h"


extern mavlink_system_t mavlink_system;

/**
 * MAVLink convenience function for sending data.
 * See "mavlink_helpers.h" for more info.
 */
void 
mavlink_send_uart_bytes(mavlink_channel_t, const uint8_t*, size_t);

#include "mavlink/v1.0/pdvapilot/mavlink.h"

/// Alias for the MAVLink channel corresponding to the radio modem.
#define RADIO_COMM_CHANNEL MAVLINK_COMM_0

/// Alias for the MAVLink channel corresponding to the sensor head.
#define SENSOR_HEAD_COMM_CHANNEL MAVLINK_COMM_1

#endif // not PDVA__MAVLINK_BRIDGE_H
