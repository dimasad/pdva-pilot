#ifndef PDVA__CONTROL_H
#define PDVA__CONTROL_H

/** @file
 * Interface of the feedback control module.
 */


/* *** Includes *** */

#include "pdva-pilot.h"


/* *** Types *** */


/* *** Functions *** */

/// Setup the control module.
ret_status_t setup_control();

/// Start the control loop.
ret_status_t start_control();

/// Free resources associated with the control module.
void teardown_control();


#endif // not PDVA__CONTROL_H
