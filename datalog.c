
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
static sensor_t sensor;

/// Local copy of the attitude data.
static attitude_t attitude;

/// Local copy of the GPS data.
static gps_t gps;

/// Local copy of the control output.
static control_t control;

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

  int i;
  double control_timer_period = pdva_config.control_timer_period.tv_sec
                              +pdva_config.control_timer_period.tv_nsec / 1e9;
  double datalog_timer_period = pdva_config.datalog_timer_period.tv_sec
                              +pdva_config.datalog_timer_period.tv_nsec / 1e9;

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
          "%% Sensor log file\n"
          "%% Experiment %d\n"
          "%% Control Timer Period %f s\n"
          "%% Datalog Timer Period %f s\n"
          "%% Downsample factor %d (period %f s)\n"
          "%% Filter order %d\n"
          "%% Filter numerator coefficients:",
          experiment, control_timer_period, datalog_timer_period,
          pdva_config.downsample.sensor.M,
          pdva_config.downsample.sensor.M * datalog_timer_period,
          pdva_config.downsample.sensor.n);

  for(i=0;i<=pdva_config.downsample.sensor.n;++i)
    fprintf(log->sensor, " %f", pdva_config.downsample.sensor.b[i]);

  fprintf(log->sensor,
          "\n"
          "%% Filter denominator coefficients:");

  for(i=0;i<=pdva_config.downsample.sensor.n;++i)
    fprintf(log->sensor, " %f", pdva_config.downsample.sensor.a[i]);

  fprintf(log->sensor,
          "\n"
          "%% \n"
          "%% Variables:\n"
          "%% \n"
          "%%   double acc[3]; ///< Accelerometer readings (m/s^2)\n"
          "%%   double gyro[3]; ///< Gyrometer readings (rad/s)\n"
          "%%   double gyro_temp; ///< Gyrometer temperature (K)\n"
          "%%   double mag[3]; ///< Magnetometer readings (T)\n"
          "%%   double dyn_press; ///< Dynamic pressure (Pa)\n"
          "%%   double stat_press; ///< Static pressure (Pa)\n"
          "%% \n"
          "%% \n"
          "t\t"
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
          "%% Attitude log file\n"
          "%% Experiment %d\n"
          "%% Control Timer Period %f s\n"
          "%% Datalog Timer Period %f s\n"
          "%% Downsample factor %d (period %f s)\n"
          "%% Filter order %d\n"
          "%% Filter numerator coefficients:",
          experiment, control_timer_period, datalog_timer_period,
          pdva_config.downsample.attitude.M,
          pdva_config.downsample.attitude.M * datalog_timer_period,
          pdva_config.downsample.attitude.n);

  for(i=0;i<=pdva_config.downsample.attitude.n;++i)
    fprintf(log->attitude, " %f", pdva_config.downsample.attitude.b[i]);

  fprintf(log->attitude,
          "\n"
          "%% Filter denominator coefficients:");

  for(i=0;i<=pdva_config.downsample.attitude.n;++i)
    fprintf(log->attitude, " %f", pdva_config.downsample.attitude.a[i]);

  fprintf(log->attitude,
          "\n"
          "%% \n"
          "%% Variables:\n"
          "%% \n"
          "%%   double att_est[3]; ///< Estimated attitude angles (roll, pitch, yaw in radians)\n"
          "%%   double airspeed; ///< Airspeed (m/s)\n"
          "%%   double altitude; ///< Altitude above sea level (m)\n"
          "%% \n"
          "%% \n"
          "t\t"
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
          "%% GPS log file\n"
          "%% Experiment %d\n"
          "%% Control Timer Period %f s\n"
          "%% Datalog Timer Period %f s\n"
          "%% Downsample factor %d (period %f s)\n"
          "%% Filter order %d\n"
          "%% Filter numerator coefficients:",
          experiment, control_timer_period, datalog_timer_period,
          pdva_config.downsample.gps.M,
          pdva_config.downsample.gps.M * datalog_timer_period,
          pdva_config.downsample.gps.n);

  for(i=0;i<=pdva_config.downsample.gps.n;++i)
    fprintf(log->gps, " %f", pdva_config.downsample.gps.b[i]);

  fprintf(log->gps,
          "\n"
          "%% Filter denominator coefficients:");

  for(i=0;i<=pdva_config.downsample.gps.n;++i)
    fprintf(log->gps, " %f", pdva_config.downsample.gps.a[i]);

  fprintf(log->gps,
          "\n"
          "%% \n"
          "%% Variables:\n"
          "%% \n"
          "%%   double lat_gps; ///< GPS latitude (deg)\n"
          "%%   double lon_gps; ///< GPS longitude (deg)\n"
          "%%   double alt_gps; ///< GPS altitude above MSL (m)\n"
          "%%   double hdg_gps; ///< GPS heading (radians)\n"
          "%%   double speed_gps; ///< GPS groundspeed (m/s)\n"
          "%%   double pos_fix_gps; ///< GPS Position Fix Status\n"
          "%%   double nosv_gps; ///< GPS Number of Satellites Used\n"
          "%%   double hdop_gps; ///< GPS Horizontal Dilution of Precision\n"
          "%% \n"
          "%% \n"
          "t\ttime\t"
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
          "%% Control log file\n"
          "%% Experiment %d\n"
          "%% Control Timer Period %f s\n"
          "%% Datalog Timer Period %f s\n"
          "%% Downsample factor %d (period %f s)\n"
          "%% Filter order %d\n"
          "%% Filter numerator coefficients:",
          experiment, control_timer_period, datalog_timer_period,
          pdva_config.downsample.control.M,
          pdva_config.downsample.control.M * datalog_timer_period,
          pdva_config.downsample.control.n);

  for(i=0;i<=pdva_config.downsample.control.n;++i)
    fprintf(log->control, " %f", pdva_config.downsample.control.b[i]);

  fprintf(log->control,
          "\n"
          "%% Filter denominator coefficients:");

  for(i=0;i<=pdva_config.downsample.control.n;++i)
    fprintf(log->control, " %f", pdva_config.downsample.control.a[i]);

  fprintf(log->control,
          "\n"
          "%% \n"
          "%% Variables:\n"
          "%% \n"
          "%%   uint16_t aileron; ///< Aileron command, 0 to 65535\n"
          "%%   uint16_t elevator; ///< Elevator command, 0 to 65535\n"
          "%%   uint16_t throttle; ///< Throttle command, 0 to 65535\n"
          "%%   uint16_t rudder; ///< Rudder command, 0 to 65535\n"
          "%% \n"
          "%% \n"
          "t\t"
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
          "%% Telecommand log file\n"
          "%% Experiment %d\n"
          "%% \n"
          "%% \n"
          "time\t"
          "command\n",
          experiment);
  
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
                              sizeof(sensor_t)/sizeof(double),
                              pdva_config.downsample.sensor.a,
                              pdva_config.downsample.sensor.b);
  filter_init(&filter_attitude, pdva_config.downsample.attitude.n,
                                sizeof(attitude_t)/sizeof(double),
                                pdva_config.downsample.attitude.a,
                                pdva_config.downsample.attitude.b);
  filter_init(&filter_gps, pdva_config.downsample.gps.n,
                           sizeof(gps_t)/sizeof(double),
                           pdva_config.downsample.gps.a,
                           pdva_config.downsample.gps.b);
  filter_init(&filter_control, pdva_config.downsample.control.n,
                               sizeof(control_t)/sizeof(double),
                               pdva_config.downsample.control.a,
                               pdva_config.downsample.control.b);

  return STATUS_SUCCESS;
}

void datalog_alarm_handler(union sigval arg) {
  mavlink_sensor_head_command_t control_out;
  sensor_t *new_sensor;
  attitude_t *new_attitude;
  gps_t *new_gps;
  control_t *new_control;
  double time, time_gps;
//printf("DatalogAlarm!!!\n");
  //Get the most recent data from the control thread
  get_datalog_data(&sensor, &attitude, &gps, &control, &time, &time_gps);

  new_sensor = (sensor_t *) filter_update(&filter_sensor, (double *) &sensor);
  new_attitude = (attitude_t *) filter_update(&filter_attitude, (double *) &attitude);
  new_gps = (gps_t *) filter_update(&filter_gps, (double *) &gps);
  new_control = (control_t *) filter_update(&filter_control, (double *) &control);

  sensor_head_command_convert(&control_out,
        new_control);


  //Write sensor file
  if (pdva_config.downsample.sensor.M &&
      ticks % pdva_config.downsample.sensor.M == 0)
    fprintf(datalog.sensor,
          "%f\t"
          "%f\t%f\t%f\t"
          "%f\t%f\t%f\t%f\t"
          "%f\t%f\t%f\t"
          "%f\t%f\n",
          time,

          new_sensor->acc[0], new_sensor->acc[1], new_sensor->acc[2],

          new_sensor->gyro[0], new_sensor->gyro[1], new_sensor->gyro[2],
          new_sensor->gyro_temp,

          new_sensor->mag[0], new_sensor->mag[1], new_sensor->mag[2],

          new_sensor->dyn_press, new_sensor->stat_press);

  //Write attitude file
  if (pdva_config.downsample.attitude.M &&
      ticks % pdva_config.downsample.attitude.M == 0)
    fprintf(datalog.attitude,
          "%f\t"
          "%f\t%f\t%f\t"
          "%f\t"
          "%f\n",
          time,

          new_attitude->att_est[0], new_attitude->att_est[1],
          new_attitude->att_est[2],

          new_attitude->airspeed,

          new_attitude->altitude);

  //Write gps file
  if (pdva_config.downsample.gps.M &&
      ticks % pdva_config.downsample.gps.M == 0)
    fprintf(datalog.gps,
          "%f\t%f\t"
          "%f\t%f\t%f\t"
          "%f\t"
          "%f\t"
          "%f\t%f\t%f\n",
          time, time_gps,

          new_gps->lat_gps, new_gps->lon_gps, new_gps->alt_gps,

          new_gps->hdg_gps,

          new_gps->speed_gps,

          new_gps->pos_fix_gps, new_gps->nosv_gps, new_gps->hdop_gps);

  //Write control file
  if (pdva_config.downsample.control.M &&
      ticks % pdva_config.downsample.control.M == 0)
    fprintf(datalog.control,
          "%f\t"
          "%u\t%u\t%u\t%u\n",
          time,

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

