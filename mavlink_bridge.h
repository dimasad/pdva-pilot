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
#define MAVLINK_SEPARATE_HELPERS
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_START_UART_SEND mavlink_start_uart_send
#define MAVLINK_END_UART_SEND mavlink_end_uart_send

#include "mavlink/v1.0/pdvapilot/mavlink.h"

/// Alias for the MAVLink channel corresponding to the radio modem.
#define RADIO_COMM_CHANNEL MAVLINK_COMM_0

/// Alias for the MAVLink channel corresponding to the sensor head.
#define SENSOR_HEAD_COMM_CHANNEL MAVLINK_COMM_1

#endif // not PDVA__MAVLINK_BRIDGE_H
