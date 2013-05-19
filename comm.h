#ifndef PDVA_COMM_H
#define PDVA_COMM_H

/** @file
 * Contains the MAVLink communication infrastructure.
 */


/* *** Includes *** */

#include "param.h"
#include "pdva-pilot.h"


/* *** Macros *** */


/* *** Types *** */


/* *** Functions *** */

/// Setup the communication module.
void process_messages(mavlink_channel_t chan);
ret_status_t setup_comm();
void teardown_comm();

void update_parameters();


#endif // not PDVA_COMM_H
