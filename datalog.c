
/** @file
 * Implementation of the datalogging.
 */

/* *** Includes *** */

#include <fcntl.h>
#include <syslog.h>
#include <unistd.h>

#include "datalog.h"


/* *** Public functions *** */

ret_status_t
datalog_init(datalog_t *log, const char *sensor, const char *control) {
  //Initialize the stream pointers
  log->sensor = log->control = NULL;
  
  //Open the sensor log file
  log->sensor = fopen(sensor, "a");
  if (log->sensor == NULL) {
    syslog(LOG_ERR, "Error opening sensor log file `%s' %m (%s)%d", sensor,
           __FILE__, __LINE__);
    return STATUS_FAILURE;
  }

  //Open the control log file
  log->control = fopen(control, "a");
  if (log->control == NULL) {
    syslog(LOG_ERR, "Error opening control log file `%s' %m (%s)%d", sensor,
           __FILE__, __LINE__);
    return STATUS_FAILURE;
  }
  
  return STATUS_SUCCESS;
}

void
datalog_destroy(datalog_t *log) {
  if (fclose(log->sensor))
    syslog(LOG_ERR, "Error on sensor log close %m (%s)%d", __FILE__, __LINE__);
  
  if (fclose(log->control))
    syslog(LOG_ERR, "Error on control log close %m (%s)%d", __FILE__, __LINE__);
}

void
log_sensor_head(datalog_t *log, mavlink_sensor_head_data_t *data) {
  if (log->sensor == NULL)
    return;
  
  fprintf(log->sensor, "%d\t" "%d\t%d\t%d\t" "%d\t%d\t%d\t%d\t" "%d\t%d\t%d\t"
          "%d\t%d\t%d\t%d\t" "%d\t%d\t%d\t%d\t" "%d\t%d\t%d\t%d\t" 
          "%d\t%d\t%d\t%d\n", data->time_boot_ms, 
          data->acc[0], data->acc[1], data->acc[2],
          data->gyro[0], data->gyro[1], data->gyro[2], data->gyro_temp,
          data->mag[0], data->mag[1], data->mag[2],
          data->adc[0], data->adc[1], data->adc[2], data->adc[3],
          data->adc[4], data->adc[5], data->adc[6], data->adc[7],
          data->adc[8], data->adc[9], data->adc[10], data->adc[11],
          data->adc[12], data->adc[13], data->adc[14], data->adc[15]);
}
