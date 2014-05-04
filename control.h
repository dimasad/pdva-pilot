#ifndef PDVA__CONTROL_H
#define PDVA__CONTROL_H

/** @file
 * Interface of the feedback control module.
 */


/* *** Includes *** */

#include "pdva-pilot.h"
#include "param.h"

/* *** Macros *** */

#ifndef CONTROL_TIMER_PERIOD_NS
#define CONTROL_TIMER_PERIOD_NS 500000000L
///< Period of the control loop in nanosecods.
#endif // not CONTROL_TIMER_PERIOD_NS


/* *** Types *** */

enum control_configuration {
  ALTITUDE_FROM_POWER = 0,
  ALTITUDE_FROM_PITCH = 1
};

/* *** Functions *** */

/// The number of times the control loop has been called.
unsigned control_loop_ticks();

/// Get the latest sensor head and control data for datalog.
void get_datalog_data(
       sensor_t *sensor_data, attitude_t *attitude_data, gps_t *gps_data,
       control_t *control_data, double *time_gps);

/// Get the latest sensor head and control data for telemetry.
void get_telemetry_data(
       mavlink_sensor_head_data_t *data,
       mavlink_sensor_head_command_t *control_data);

/// Setup the control module.
ret_status_t setup_control();

/// Start the control loop.
ret_status_t start_control();

/// Free resources associated with the control module.
void teardown_control();

// Convert sensor head data to double format in SI units
void sensor_head_data_convert(mavlink_sensor_head_data_t *data,
        sensor_t *sensor_data, attitude_t *attitude_data, gps_t *gps_data);

// Convert sensor head command from double format to PWM
void sensor_head_command_convert(mavlink_sensor_head_command_t *data,
        control_t *control_data);

#endif // not PDVA__CONTROL_H
