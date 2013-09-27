#ifndef PDVA__DATALOG_H
#define PDVA__DATALOG_H

/** @file
 * Contains the datalogging infrastructure.
 */

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* *** Includes *** */

#include <stdio.h>

#include "mavlink_bridge.h"
#include "pdva-pilot.h"

/* *** Macros *** */
/// Path to the datalogging directory
#define DATALOG_DIR "/var/log/pdva"
/// Maximum length for the path string
#define MAX_PATH_LENGTH 50


/* *** Types *** */

/// Configuration structure of the pdva-pilot instance.
typedef struct {
  FILE *sensor;
  FILE *attitude;
  FILE *gps;
  FILE *control;
} datalog_t;


/* *** Functions *** */

/// Function executed by the datalog thread.
void *
datalogging(void *);

/// Initizalize a datalog object.
ret_status_t
datalog_init(datalog_t *log);

/// Alarm handler for the datalog timer.
void
datalog_alarm_handler(union sigval);

/// Free the resources associated with a datalog object.
void
datalog_destroy(datalog_t *log);


#ifdef __cplusplus
}
#endif //__cplusplus

#endif // not PDVA__DATALOG_H
