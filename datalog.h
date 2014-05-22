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
#include <semaphore.h>

#include "mavlink_bridge.h"
#include "pdva-pilot.h"
#include "control.h"

/* *** Macros *** */
/// Path to the datalogging directory
#define DATALOG_DIR "/home/root/log"
/// Maximum length for the path string
#define MAX_LENGTH 50
/// Period between writes to file (ms)
#define DATALOG_WRITE_MS 5000
/// Maximum line lengths for each file
#define SENSOR_LINE_LEN 230
#define ATTITUDE_LINE_LEN 130
#define GPS_LINE_LEN 200
#define CONTROL_LINE_LEN 80


/* *** Types *** */

/// Datalog structure.
typedef struct {
  FILE *sensor;
  FILE *attitude;
  FILE *gps;
  FILE *control;
  FILE *telecommand;
} datalog_t;

/// Structure for each datalog file.
typedef struct {
  pthread_t thread;
  FILE * file;
  sem_t sem;
  char *buffer;
  int count;
} datalog_file_t;

/// Digital filter structure.
/// Used to implement a[0]*y[k]+...+a[n]*y[k-n] = b[0]*x[k]+...+b[n]*x[k-n]
typedef struct {
  int n;       ///< Filter order.
  int v;       ///< Number of variables (number of circular buffers).
  int k;       ///< Current index for the circular buffers.
  double *a;   ///< Denominator coefficients (from a[0] to a[n]).
  double *b;   ///< Numerator coefficients (from b[0] to b[n]).
  double *X;   ///< v circular buffers that store the last n+1 input values.
  double *Y;   ///< v circular buffers that store the last n+1 output values.
  int enable; ///< When FALSE, filter is not used and y[k] = x[k].
} filter_t;


/* *** Functions *** */

/// Function executed by the writer thread of each file.
void *
writer(void * arg);

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

/// Initialize the digital filter structure.
void
filter_init(filter_t *filter, int n, int v, double *a, double *b);

/// Free the resources associated with a filter object.
void
filter_destroy(filter_t *filter);

/// Update the filter (one iteraction).
double *
filter_update(filter_t *filter, double *x);


#ifdef __cplusplus
}
#endif //__cplusplus

#endif // not PDVA__DATALOG_H
