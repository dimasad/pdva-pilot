
/** @file
 * Implementation of the datalogging.
 */

/* *** Includes *** */

#include <stdlib.h>
#include <fcntl.h>
#include <syslog.h>
#include <unistd.h>
#include <sys/stat.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>

#include "datalog.h"
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

/// Low-pass filter structures.
filter_t filter_sensor, filter_attitude, filter_gps, filter_control;


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
    return (void *) STATUS_FAILURE;
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
    return (void *) STATUS_FAILURE;
  }

  //Destroy the thread attributes (no longer needed)
  if(pthread_attr_destroy(&attr)){
    syslog(LOG_ERR, "Could not destroy datalog thread attributes.");
    return (void *) STATUS_FAILURE;
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
    return (void *) STATUS_FAILURE;
  }


printf("before pause\n");
    pause();
printf("after pause\n");

  datalog_destroy(&datalog);

  return NULL;
}

ret_status_t
datalog_init(datalog_t *log) {
  //Initialize the stream pointers
  log->sensor = log->attitude = log->gps =
  log->control = log->telecommand = NULL;

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
  char filename[MAX_LENGTH];
  snprintf(filename, MAX_LENGTH, "%s/%03d",
         DATALOG_DIR, experiment);
  mkdir(filename, S_IRWXU | S_IRGRP | S_IXGRP | S_IROTH | S_IXOTH);


  //Create the sensor log file
  snprintf(filename, MAX_LENGTH, "%s/%03d/sensor",
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
          "dyn_p\tstat_p\n");

  //Create the attitude log file
  snprintf(filename, MAX_LENGTH, "%s/%03d/attitude",
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
  snprintf(filename, MAX_LENGTH, "%s/%03d/gps",
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
          "speed\t"
          "posfix\tnosv\thdop\n");

  //Create the control log file
  snprintf(filename, MAX_LENGTH, "%s/%03d/control",
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

  //Create the telecommand log file
  snprintf(filename, MAX_LENGTH, "%s/%03d/telecommand",
         DATALOG_DIR, experiment);
  log->telecommand = fopen(filename, "w");
  if (log->telecommand == NULL) {
    syslog(LOG_ERR, "Error opening telecommand log file `%s' %m (%s)%d",
           filename, __FILE__, __LINE__);
    return STATUS_FAILURE;
  }

  fprintf(log->telecommand,
          "time\t"
          "command\n");
  
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

  //Initialize filters
  filter_init(&filter_sensor, pdva_config.downsample.sensor.n,
                              NUM_VAR_SENSOR,
                              pdva_config.downsample.sensor.a,
                              pdva_config.downsample.sensor.b);
  filter_init(&filter_attitude, pdva_config.downsample.attitude.n,
                                NUM_VAR_ATTITUDE,
                                pdva_config.downsample.attitude.a,
                                pdva_config.downsample.attitude.b);
  filter_init(&filter_gps, pdva_config.downsample.gps.n,
                           NUM_VAR_GPS,
                           pdva_config.downsample.gps.a,
                           pdva_config.downsample.gps.b);
  filter_init(&filter_control, pdva_config.downsample.control.n,
                               NUM_VAR_CONTROL,
                               pdva_config.downsample.control.a,
                               pdva_config.downsample.control.b);

  return STATUS_SUCCESS;
}

void datalog_alarm_handler(union sigval arg) {
  double var_sensor[NUM_VAR_SENSOR],
         var_attitude[NUM_VAR_ATTITUDE],
         var_gps[NUM_VAR_GPS],
         var_control[NUM_VAR_CONTROL];
  double *new_sensor, *new_attitude, *new_gps, *new_control;
//printf("DatalogAlarm!!!\n");
  //Get the most recent data from the control thread
  get_sensor_and_control_data(&sensor_data, &control_out);

  data_to_filter(&sensor_data, &control_out,
                 var_sensor, var_attitude, var_gps, var_control);

  new_sensor = filter_update(&filter_sensor, var_sensor);
  new_attitude = filter_update(&filter_attitude, var_attitude);
  new_gps = filter_update(&filter_gps, var_gps);
  new_control = filter_update(&filter_control, var_control);

  filter_to_data(&sensor_data, &control_out,
                 new_sensor, new_attitude, new_gps, new_control);

  //Write sensor file
  if (pdva_config.downsample.sensor.M &&
      ticks % pdva_config.downsample.sensor.M == 0)
    fprintf(datalog.sensor,
          "%d\t"
          "%d\t%d\t%d\t"
          "%d\t%d\t%d\t%d\t"
          "%d\t%d\t%d\t"
          "%d\t%d\n",
          sensor_data.time_gps_ms,

          sensor_data.acc[0], sensor_data.acc[1], sensor_data.acc[2],

          sensor_data.gyro[0], sensor_data.gyro[1], sensor_data.gyro[2],
          sensor_data.gyro_temp,

          sensor_data.mag[0], sensor_data.mag[1], sensor_data.mag[2],

          sensor_data.dyn_press, sensor_data.stat_press);

  //Write attitude file
  if (pdva_config.downsample.attitude.M &&
      ticks % pdva_config.downsample.attitude.M == 0)
    fprintf(datalog.attitude,
          "%d\t"
          "%f\t%f\t%f\t"
          "%u\t"
          "%d\n",
          sensor_data.time_gps_ms,

          sensor_data.att_est[0], sensor_data.att_est[1],
          sensor_data.att_est[2],

          sensor_data.airspeed,

          sensor_data.altitude);

  //Write gps file
  if (pdva_config.downsample.gps.M &&
      ticks % pdva_config.downsample.gps.M == 0)
    fprintf(datalog.gps,
          "%d\t"
          "%d\t%d\t%d\t"
          "%d\t"
          "%u\t"
          "%d\t%d\t%d\n",
          sensor_data.time_gps_ms,

          sensor_data.lat_gps, sensor_data.lon_gps, sensor_data.alt_gps,

          sensor_data.hdg_gps,

          sensor_data.speed_gps,

          sensor_data.pos_fix_gps, sensor_data.nosv_gps, sensor_data.hdop_gps);

  //Write control file
  if (pdva_config.downsample.control.M &&
      ticks % pdva_config.downsample.control.M == 0)
    fprintf(datalog.control,
          "%d\t"
          "%u\t%u\t%u\t%u\n",
          sensor_data.time_gps_ms,

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

  if (fclose(log->telecommand))
    syslog(LOG_ERR, "Error on telecommand log close %m (%s)%d",
           __FILE__, __LINE__);

  filter_destroy(&filter_sensor);
  filter_destroy(&filter_attitude);
  filter_destroy(&filter_gps);
  filter_destroy(&filter_control);
}


void
filter_init(filter_t *filter, int n, int v, double *a, double *b){
  int i;

  filter->n = n;
  filter->v = v;
  filter->k = 0;
  filter->enable = FALSE;
  filter->a = (double *) malloc( (n+1) * sizeof(double) );
  filter->b = (double *) malloc( (n+1) * sizeof(double) );
  filter->X = (double *) malloc( (n+1) * v * sizeof(double) );
  filter->Y = (double *) malloc( (n+1) * v * sizeof(double) );
  for(i=0; i<(n+1); ++i){
    filter->a[i] = a[i];
    filter->b[i] = b[i];
  }
  for(i=0; i<(n+1)*v; ++i){
    filter->X[i] = 0.0;
    filter->Y[i] = 0.0;
  }
}

void
filter_destroy(filter_t *filter){
  free(filter->a);
  free(filter->b);
  free(filter->X);
  free(filter->Y);
}


/// Update the filter (one iteraction).
double *
filter_update(filter_t *f, double *x){
  int i, j, l;

  if(f->enable){

    for(i=0; i < f->v; ++i){
      f->X[f->k * f->v + i] = x[i];
      f->Y[f->k * f->v + i] = 0.0;
    }

    // Calculate the sum b[0]*x[k]+...+b[n]*x[k-n]
    for(j=0; j < f->n+1; ++j){
      l = (f->n+1 + f->k - j) % (f->n+1);
      for(i=0; i < f->v; ++i){
        f->Y[f->k * f->v + i] += f->b[j] * f->X[l * f->v + i];
      }
    }

    // Subtract the sum a[1]*y[k-1]+...+a[n]*y[k-n]
    for(j=1; j < f->n+1; ++j){
      l = (f->n+1 + f->k - j) % (f->n+1);
      for(i=0; i < f->v; ++i){
        f->Y[f->k * f->v + i] -= f->a[j] * f->Y[l * f->v + i];
      }
    }

    // Divide by a[0]
    for(i=0; i < f->v; ++i){
      f->Y[f->k * f->v + i] /= f->a[0];
    }

  }else{ // not f->enable

    for(i=0; i < f->v; ++i){
      f->X[f->k * f->v + i] = x[i];
      f->Y[f->k * f->v + i] = x[i];
    }

  }

  l = f->k;
  f->k++;
  if(f->k == f->n+1){
    f->k = 0;
    f->enable = TRUE;
  }

  return &f->Y[l * f->v];

}


/// Convert sensor head and control data to double arrays for filtering
void data_to_filter(mavlink_sensor_head_data_t * sensor_data,
       control_out_t * control_out, double * var_sensor,
       double * var_attitude, double * var_gps, double * var_control){
  int i;

  for(i=0;i<3;++i)
    var_sensor[i] = (double) sensor_data->acc[i];
  for(i=0;i<3;++i)
    var_sensor[3+i] = (double) sensor_data->gyro[i];
  var_sensor[6] = (double) sensor_data->gyro_temp;
  for(i=0;i<3;++i)
    var_sensor[7+i] = (double) sensor_data->mag[i];
  var_sensor[10] = (double) sensor_data->dyn_press;
  var_sensor[11] = (double) sensor_data->stat_press;

  for(i=0;i<3;++i)
    var_attitude[i] = (double) sensor_data->att_est[i];
  var_attitude[3] = (double) sensor_data->airspeed;
  var_attitude[4] = (double) sensor_data->altitude;

  var_gps[0] = (double) sensor_data->lat_gps;
  var_gps[1] = (double) sensor_data->lon_gps;
  var_gps[2] = (double) sensor_data->alt_gps;
  var_gps[3] = (double) sensor_data->hdg_gps;
  var_gps[4] = (double) sensor_data->speed_gps;
  var_gps[5] = (double) sensor_data->pos_fix_gps;
  var_gps[6] = (double) sensor_data->nosv_gps;
  var_gps[7] = (double) sensor_data->hdop_gps;

  var_control[0] = (double) control_out->aileron;
  var_control[1] = (double) control_out->elevator;
  var_control[2] = (double) control_out->throttle;
  var_control[3] = (double) control_out->rudder;

}

/// Convert double arrays back to sensor head and control data.
void filter_to_data(mavlink_sensor_head_data_t * sensor_data,
       control_out_t * control_out, double * var_sensor,
       double * var_attitude, double * var_gps, double * var_control){

  int i;

  for(i=0;i<3;++i)
    sensor_data->acc[i] = (int) var_sensor[i];
  for(i=0;i<3;++i)
    sensor_data->gyro[i] = (int) var_sensor[3+i];
  sensor_data->gyro_temp = (short) var_sensor[6];
  for(i=0;i<3;++i)
    sensor_data->mag[i] = (int) var_sensor[7+i];
  sensor_data->dyn_press = (int) var_sensor[10];
  sensor_data->stat_press = (int) var_sensor[11];

  for(i=0;i<3;++i)
    sensor_data->att_est[i] = (float) var_attitude[i];
  sensor_data->airspeed = (unsigned) var_attitude[3];
  sensor_data->altitude = (int) var_attitude[4];

  sensor_data->lat_gps = (int) var_gps[0];
  sensor_data->lon_gps = (int) var_gps[1];
  sensor_data->alt_gps = (int) var_gps[2];
  sensor_data->hdg_gps = (short) var_gps[3];
  sensor_data->speed_gps = (unsigned short) var_gps[4];
  sensor_data->pos_fix_gps = (char) var_gps[5];
  sensor_data->nosv_gps = (char) var_gps[6];
  sensor_data->hdop_gps = (short) var_gps[7];


  control_out->aileron = (unsigned) var_control[0];
  control_out->elevator = (unsigned) var_control[1];
  control_out->throttle = (unsigned) var_control[2];
  control_out->rudder = (unsigned) var_control[3];

}


