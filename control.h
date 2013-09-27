#ifndef PDVA__CONTROL_H
#define PDVA__CONTROL_H

/** @file
 * Interface of the feedback control module.
 */


/* *** Includes *** */

#include "pdva-pilot.h"

/* *** Macros *** */

#ifndef CONTROL_TIMER_PERIOD_NS
#define CONTROL_TIMER_PERIOD_NS 500000000L
///< Period of the control loop in nanosecods.
#endif // not CONTROL_TIMER_PERIOD_NS

#define CONTROL_TIMER_PERIOD_S (CONTROL_TIMER_PERIOD_NS / 1e9)

/* *** Types *** */

/// Control data structure.
typedef struct {
  unsigned aileron;
  unsigned elevator;
  unsigned throttle;
  unsigned rudder;
} control_out_t;

enum control_configuration {
  ALTITUDE_FROM_POWER = 0,
  ALTITUDE_FROM_PITCH = 1
};

/* *** Functions *** */

/// The number of times the control loop has been called.
unsigned control_loop_ticks();

/// Get the latest sensor head and control data available.
void get_sensor_and_control_data(
       mavlink_sensor_head_data_t *sensor, control_out_t *control);

/// Setup the control module.
ret_status_t setup_control();

/// Start the control loop.
ret_status_t start_control();

/// Free resources associated with the control module.
void teardown_control();

#endif // not PDVA__CONTROL_H
