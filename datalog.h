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


/* *** Types *** */

/// Configuration structure of the pdva-pilot instance.
typedef struct {
  FILE *sensor;
  FILE *control;
} datalog_t;


/* *** Functions *** */

/// Initizalize a datalog object.
ret_status_t
datalog_init(datalog_t *log, const char *sensor, const char *control);

/// Free the resources associated with a datalog object.
void
datalog_destroy(datalog_t *log);

/// Write the sensor head data to the log.
void
log_sensor_head(datalog_t *log, mavlink_sensor_head_data_t *data);


#ifdef __cplusplus
}
#endif //__cplusplus

#endif // not PDVA__DATALOG_H
