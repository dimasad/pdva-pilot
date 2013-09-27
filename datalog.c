
/** @file
 * Implementation of the datalogging.
 */

/* *** Includes *** */

#include <fcntl.h>
#include <syslog.h>
#include <unistd.h>
#include <sys/stat.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>

#include "datalog.h"
#include "control.h"
#include "param.h"

/* *** Variables *** */

/// Configuration structure.
extern pdva_pilot_config_t pdva_config;

/* *** Internal variables *** */

/// Structure that contains the datalog files.
static datalog_t datalog;

/// Number of times the datalogging loop has run.
static uint64_t ticks = 0;

/// Datalog loop timer object.
static timer_t datalog_timer;

/// Local copy of the sensor data.
static mavlink_sensor_head_data_t sensor_data;

/// Local copy of the control output.
static control_out_t control_out;


/* *** Public functions *** */

void *
datalogging(void *arg) {

  //Setup the datalogs
  datalog_init(&datalog);

  //Get the attributes of the current thread
  pthread_attr_t attr;
  pthread_t this_thread = pthread_self();
  if(pthread_getattr_np(this_thread, &attr)){
    syslog(LOG_ERR, "Could not get datalog thread attributes.");
    return STATUS_FAILURE;
  }

  //Initialize the sigevent structure
  struct sigevent sevp;
  sevp.sigev_notify = SIGEV_THREAD;
  sevp.sigev_value.sival_int = 0;
  sevp.sigev_notify_function = &datalog_alarm_handler;
  sevp.sigev_notify_attributes = &attr;

  //Create timer
  if (timer_create(CLOCK_REALTIME, &sevp, &datalog_timer)) {
    syslog(LOG_ERR, "Error creating datalog timer: %m (%s)%d",
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }

  //Destroy the thread attributes (no longer needed)
  if(pthread_attr_destroy(&attr)){
    syslog(LOG_ERR, "Could not destroy datalog thread attributes.");
    return STATUS_FAILURE;
  }

  //Set timer
  struct itimerspec timer_spec;
  timer_spec.it_interval.tv_sec = pdva_config.datalog_timer_period.tv_sec;
  timer_spec.it_value.tv_sec = pdva_config.datalog_timer_period.tv_sec;
  timer_spec.it_interval.tv_nsec = pdva_config.datalog_timer_period.tv_nsec;
  timer_spec.it_value.tv_nsec = pdva_config.datalog_timer_period.tv_nsec;

  if (timer_settime(datalog_timer, 0, &timer_spec, NULL)) {
    syslog(LOG_ERR, "Error setting control loop timer: %m (%s)%d.",
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }


  pause();

  datalog_destroy(&datalog);

  return NULL;
}

ret_status_t
datalog_init(datalog_t *log) {
  //Initialize the stream pointers
  log->sensor = log->attitude = log->gps = log->control = NULL;

  //Create the datalog directory (if it doesn't already exist)
  mkdir(DATALOG_DIR, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);

  //Read the number of the last experiment
  int experiment = -1;
  FILE * experiment_file = fopen(DATALOG_DIR "/last_experiment", "r");
  if(experiment_file){
    fscanf(experiment_file, "%d", &experiment);
    if (fclose(experiment_file))
      syslog(LOG_ERR, "Error on last experiment close %m (%s)%d",
             __FILE__, __LINE__);
  }
  //Calculate the number of the current experiment
  experiment++;
  
  //Create a new directory for the current experiment
  char filename[MAX_PATH_LENGTH];
  snprintf(filename, MAX_PATH_LENGTH, "%s/%03d",
         DATALOG_DIR, experiment);
  mkdir(filename, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);


  //Create the sensor log file
  snprintf(filename, MAX_PATH_LENGTH, "%s/%03d/sensor",
         DATALOG_DIR, experiment);
  log->sensor = fopen(filename, "w");
  if (log->sensor == NULL) {
    syslog(LOG_ERR, "Error opening sensor log file `%s' %m (%s)%d", filename,
           __FILE__, __LINE__);
    return STATUS_FAILURE;
  }

  fprintf(log->sensor,
          "time\t"
          "acc0\tacc1\tacc2\t"
          "gyro0\tgyro1\tgyro2\tg_temp\t"
          "mag0\tmag1\tmag2\t"
          "adc0\tadc1\tadc2\tadc3\tadc4\tadc5\tadc6\tadc7\t"
          "adc8\tadc9\tadc10\tadc11\tadc12\tadc13\tadc14\tadc15\t"
          "dyn_p\tstat_p\n");

  //Create the attitude log file
  snprintf(filename, MAX_PATH_LENGTH, "%s/%03d/attitude",
         DATALOG_DIR, experiment);
  log->attitude = fopen(filename, "w");
  if (log->attitude == NULL) {
    syslog(LOG_ERR, "Error opening attitude log file `%s' %m (%s)%d", filename,
           __FILE__, __LINE__);
    return STATUS_FAILURE;
  }

  fprintf(log->attitude,
          "time\t"
          "att0\tatt1\tatt2\t"
          "airsp\t"
          "alti\n");

  //Create the gps log file
  snprintf(filename, MAX_PATH_LENGTH, "%s/%03d/gps",
         DATALOG_DIR, experiment);
  log->gps = fopen(filename, "w");
  if (log->gps == NULL) {
    syslog(LOG_ERR, "Error opening gps log file `%s' %m (%s)%d", filename,
           __FILE__, __LINE__);
    return STATUS_FAILURE;
  }

  fprintf(log->gps,
          "time\t"
          "lat\tlon\talt\t"
          "hdg\t"
          "vel0\tvel1\tvel2\n");

  //Create the control log file
  snprintf(filename, MAX_PATH_LENGTH, "%s/%03d/control",
         DATALOG_DIR, experiment);
  log->control = fopen(filename, "w");
  if (log->control == NULL) {
    syslog(LOG_ERR, "Error opening control log file `%s' %m (%s)%d", filename,
           __FILE__, __LINE__);
    return STATUS_FAILURE;
  }

  fprintf(log->control,
          "time\t"
          "aileron\televator\tthrottle\trudder\n");
  
  //Write the number of the last experiment (current one)
  experiment_file = fopen(DATALOG_DIR "/last_experiment", "w");
  if(experiment_file == NULL){
    syslog(LOG_ERR, "Error opening last experiment file `%s' %m (%s)%d",
           DATALOG_DIR "/last_experiment", __FILE__, __LINE__);
    return STATUS_FAILURE;
  }
  fprintf(experiment_file, "%d\n", experiment);
  if (fclose(experiment_file))
    syslog(LOG_ERR, "Error on last experiment close %m (%s)%d",
           __FILE__, __LINE__);

  return STATUS_SUCCESS;
}

void datalog_alarm_handler(union sigval arg) {

  //Get the most recent data from the control thread
  get_sensor_and_control_data(&sensor_data, &control_out);

  //Write sensor file
  if (pdva_config.downsample.sensor &&
      ticks % pdva_config.downsample.sensor == 0)
    fprintf(datalog.sensor,
          "%u\t"
          "%u\t%u\t%u\t"
          "%u\t%u\t%u\t%u\t"
          "%u\t%u\t%u\t"
          "%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t"
          "%u\t%u\n",
          sensor_data.time_boot_ms,

          sensor_data.acc[0], sensor_data.acc[1], sensor_data.acc[2],

          sensor_data.gyro[0], sensor_data.gyro[1], sensor_data.gyro[2],
          sensor_data.gyro_temp,

          sensor_data.mag[0], sensor_data.mag[1], sensor_data.mag[2],

          sensor_data.adc[0], sensor_data.adc[1], sensor_data.adc[2],
          sensor_data.adc[3], sensor_data.adc[4], sensor_data.adc[5],
          sensor_data.adc[6], sensor_data.adc[7], sensor_data.adc[8],
          sensor_data.adc[9], sensor_data.adc[10], sensor_data.adc[11],
          sensor_data.adc[12], sensor_data.adc[13], sensor_data.adc[14],
          sensor_data.adc[15],

          sensor_data.dyn_press, sensor_data.stat_press);

  //Write attitude file
  if (pdva_config.downsample.attitude &&
      ticks % pdva_config.downsample.attitude == 0)
    fprintf(datalog.attitude,
          "%u\t"
          "%d\t%d\t%d\t"
          "%u\t"
          "%d\n",
          sensor_data.time_boot_ms,

          sensor_data.att_est[0], sensor_data.att_est[1],
          sensor_data.att_est[2],

          sensor_data.airspeed,

          sensor_data.altitude);

  //Write gps file
  if (pdva_config.downsample.gps &&
      ticks % pdva_config.downsample.gps == 0)
    fprintf(datalog.gps,
          "%u\t"
          "%d\t%d\t%d\t"
          "%d\t"
          "%d\t%d\t%d\n",
          sensor_data.time_boot_ms,

          sensor_data.lat_gps, sensor_data.lon_gps, sensor_data.alt_gps,

          sensor_data.hdg_gps,

          sensor_data.vel_gps[0], sensor_data.vel_gps[1],
          sensor_data.vel_gps[2]);

  //Write control file
  if (pdva_config.downsample.control &&
      ticks % pdva_config.downsample.control == 0)
    fprintf(datalog.control,
          "%u\t"
          "%u\t%u\t%u\t%u\n",
          sensor_data.time_boot_ms,

          control_out.aileron, control_out.elevator,
          control_out.throttle, control_out.rudder);


  //Increment the tick counter
  ticks++;
}

void
datalog_destroy(datalog_t *log) {
  timer_delete(datalog_timer);

  if (fclose(log->sensor))
    syslog(LOG_ERR, "Error on sensor log close %m (%s)%d",
           __FILE__, __LINE__);

  if (fclose(log->attitude))
    syslog(LOG_ERR, "Error on attitude log close %m (%s)%d",
           __FILE__, __LINE__);

  if (fclose(log->gps))
    syslog(LOG_ERR, "Error on gps log close %m (%s)%d",
           __FILE__, __LINE__);
  if (fclose(log->control))
    syslog(LOG_ERR, "Error on control log close %m (%s)%d",
           __FILE__, __LINE__);
}


