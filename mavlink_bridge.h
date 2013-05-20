#ifndef PDVA__MAVLINK_BRIDGE_H
#define PDVA__MAVLINK_BRIDGE_H

/** @file
 * MAVLink adapter header.
 * Include this file instead of "mavlink.h".
 */


#include <stdint.h>

#include "pdva-pilot.h"


#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEND_UART_BYTES mavlink_send_uart_bytes

#include "mavlink/v1.0/mavlink_types.h"


extern mavlink_system_t mavlink_system;

/**
 * MAVLink convenience function for sending data.
 * See "mavlink_helpers.h" for more info.
 */
ret_status_t mavlink_send_uart_bytes(mavlink_channel_t chan, 
				     const uint8_t* buff, size_t len);


#include "mavlink/v1.0/common/mavlink.h"

#endif // not PDVA__MAVLINK_BRIDGE_H
