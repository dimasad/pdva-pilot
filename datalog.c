
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

/// Mutex used to for datalog alarm handler
pthread_mutex_t datalog_mutex;

/// Structures used by each writer thread
datalog_file_t sensor_file, attitude_file, gps_file, control_file;

/* *** Internal variables *** */

/// Buffers to write to datalog files
static char *sensor_buffer, *attitude_buffer, *gps_buffer, *control_buffer;

/// Size of the buffers
static int sensor_size, attitude_size, gps_size, control_size;

/// Current index for the buffers
static int p = 0;

/// Number of bytes used in each buffer
static int sensor_count, attitude_count, gps_count, control_count;

/// Number of ticks since last write.
static int ticks_write = 0;

/// Number of ticks between write operations.
static int write_interval = 1;

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

/// Function executed by the writer thread of each file.
void *
writer(void * arg) {
  datalog_file_t * file = (datalog_file_t *) arg;

  while(1) {
    if(sem_wait(&file->sem))
      syslog(LOG_ERR, "Error waiting on semaphore: %m (%s)%d",
	   __FILE__, __LINE__);


    //Write file
    if(fwrite(file->buffer, file->count, 1, file->file) != 1)
      syslog(LOG_ERR, "Error writing on file: %m (%s)%d",
	   __FILE__, __LINE__);
  }
  return NULL;
}

void *
datalogging(void *arg) {

  //Setup the datalogs
  datalog_init(&datalog);
/*
  //Get the attributes of the current thread
  pthread_attr_t attr;
  pthread_t this_thread = pthread_self();
  if(pthread_getattr_np(this_thread, &attr)){
    syslog(LOG_ERR, "Could not get datalog thread attributes.");
    return (void *) STATUS_FAILURE;
  }
*/
  //Initialize the sigevent structure
  struct sigevent sevp;
  sevp.sigev_notify = SIGEV_THREAD;
  sevp.sigev_value.sival_int = 0;
  sevp.sigev_notify_function = &datalog_alarm_handler;
  sevp.sigev_notify_attributes = NULL;

  //Create timer
  if (timer_create(CLOCK_REALTIME, &sevp, &datalog_timer)) {
    syslog(LOG_ERR, "Error creating datalog timer: %m (%s)%d",
	   __FILE__, __LINE__);
    return (void *) STATUS_FAILURE;
  }
/*
  //Destroy the thread attributes (no longer needed)
  if(pthread_attr_destroy(&attr)){
    syslog(LOG_ERR, "Could not destroy datalog thread attributes.");
    return (void *) STATUS_FAILURE;
  }
*/
  //Set timer
  struct itimerspec timer_spec;
  timer_spec.it_interval.tv_sec = pdva_config.datalog_timer_period.tv_sec;
  timer_spec.it_value.tv_sec = 0;
  timer_spec.it_interval.tv_nsec = pdva_config.datalog_timer_period.tv_nsec;
  timer_spec.it_value.tv_nsec = pdva_config.control_timer_period.tv_nsec/2;

  if (timer_settime(datalog_timer, 0, &timer_spec, NULL)) {
    syslog(LOG_ERR, "Error setting datalog loop timer: %m (%s)%d.",
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

  sensor_size = (pdva_config.datalog_write_ms * SENSOR_LINE_LEN /
                (1000.0 * datalog_timer_period));
  attitude_size = (pdva_config.datalog_write_ms * ATTITUDE_LINE_LEN /
                (1000.0 * datalog_timer_period));
  gps_size = (pdva_config.datalog_write_ms * GPS_LINE_LEN /
                (1000.0 * datalog_timer_period));
  control_size = (pdva_config.datalog_write_ms * CONTROL_LINE_LEN /
                (1000.0 * datalog_timer_period));

  sensor_buffer = (char *) malloc(2 * sensor_size * sizeof(char));
  attitude_buffer = (char *) malloc(2 * attitude_size * sizeof(char));
  gps_buffer = (char *) malloc(2 * gps_size * sizeof(char));
  control_buffer = (char *) malloc(2 * control_size * sizeof(char));

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
          "%%   uint8_t seq; ///< Last message sequential number\n"
          "%%   uint8_t ok; ///< Last message received successfully\n"
          "%%   double att_est[3]; ///< Estimated attitude angles (roll, pitch, yaw in radians)\n"
          "%%   double airspeed; ///< Airspeed (m/s)\n"
          "%%   double altitude; ///< Altitude above sea level (m)\n"
          "%% \n"
          "%% \n"
          "t\t"
          "seq\tok\t"
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

  sensor_count = attitude_count = gps_count = control_count = 0;

  write_interval = (int) (pdva_config.datalog_write_ms /
                          (1000.0 * datalog_timer_period));

  //Initialize datalog mutex
  if (pthread_mutex_init(&datalog_mutex, NULL)) {
    syslog(LOG_ERR, "Error initializing datalog mutex: %m (%s)%d",
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }


  sensor_file.file = datalog.sensor;
  if(sem_init(&sensor_file.sem, 0, 0))
    syslog(LOG_ERR, "Error initializing sensor semaphore: %m (%s)%d",
	   __FILE__, __LINE__);
  sensor_file.buffer = NULL;
  sensor_file.count = 0;

  if(pthread_create(&sensor_file.thread, NULL, writer, &sensor_file))
    syslog(LOG_ERR, "Error creating sensor thread: %m (%s)%d",
	   __FILE__, __LINE__);


  attitude_file.file = datalog.attitude;
  if(sem_init(&attitude_file.sem, 0, 0))
    syslog(LOG_ERR, "Error initializing attitude semaphore: %m (%s)%d",
	   __FILE__, __LINE__);
  attitude_file.buffer = NULL;
  attitude_file.count = 0;

  if(pthread_create(&attitude_file.thread, NULL, writer, &attitude_file))
    syslog(LOG_ERR, "Error creating attitude thread: %m (%s)%d",
	   __FILE__, __LINE__);


  gps_file.file = datalog.gps;
  if(sem_init(&gps_file.sem, 0, 0))
    syslog(LOG_ERR, "Error initializing gps semaphore: %m (%s)%d",
	   __FILE__, __LINE__);
  gps_file.buffer = NULL;
  gps_file.count = 0;

  if(pthread_create(&gps_file.thread, NULL, writer, &gps_file))
    syslog(LOG_ERR, "Error creating gps thread: %m (%s)%d",
	   __FILE__, __LINE__);


  control_file.file = datalog.control;
  if(sem_init(&control_file.sem, 0, 0))
    syslog(LOG_ERR, "Error initializing control semaphore: %m (%s)%d",
	   __FILE__, __LINE__);
  control_file.buffer = NULL;
  control_file.count = 0;

  if(pthread_create(&control_file.thread, NULL, writer, &control_file))
    syslog(LOG_ERR, "Error creating control thread: %m (%s)%d",
	   __FILE__, __LINE__);


  return STATUS_SUCCESS;
}

void datalog_alarm_handler(union sigval arg) {
  mavlink_sensor_head_command_t control_out;
  sensor_t *new_sensor;
  attitude_t *new_attitude;
  gps_t *new_gps;
  control_t *new_control;
  double time, time_gps;
  uint8_t seq, ok;
  int a;

  double datalog_timer_period = pdva_config.datalog_timer_period.tv_sec
                              +pdva_config.datalog_timer_period.tv_nsec / 1e9;

  //Lock datalog mutex
  if (pthread_mutex_trylock(&datalog_mutex)) {
    printf("Mutex NOT ok\n");
    return;
  }

//printf("DatalogAlarm!!!\n");
  //Get the most recent data from the control thread
  get_datalog_data(&sensor, &attitude, &gps, &control, &time, &time_gps, &seq, &ok);

  new_sensor = (sensor_t *) filter_update(&filter_sensor, (double *) &sensor);
  new_attitude = (attitude_t *) filter_update(&filter_attitude, (double *) &attitude);
  new_gps = (gps_t *) filter_update(&filter_gps, (double *) &gps);
  new_control = (control_t *) filter_update(&filter_control, (double *) &control);

  sensor_head_command_convert(&control_out,
        new_control);


  //Write sensor buffer
  if (pdva_config.downsample.sensor.M &&
      ticks % pdva_config.downsample.sensor.M == 0) {
    a = snprintf(&sensor_buffer[p*sensor_size + sensor_count],
                 sensor_size - sensor_count,
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
    if(a<0 || a>= sensor_size - sensor_count) {
      syslog(LOG_ERR, "Error writing on sensor buffer: %m (%s)%d",
	   __FILE__, __LINE__);
      return;
    }
    sensor_count += a;
  }

  //Write attitude buffer
  if (pdva_config.downsample.attitude.M &&
      ticks % pdva_config.downsample.attitude.M == 0) {
    a = snprintf(&attitude_buffer[p*attitude_size + attitude_count],
                 attitude_size - attitude_count,
          "%f\t"
          "%u\t%u\t"
          "%f\t%f\t%f\t"
          "%f\t"
          "%f\n",
          time,

          seq, ok,

          new_attitude->att_est[0], new_attitude->att_est[1],
          new_attitude->att_est[2],

          new_attitude->airspeed,

          new_attitude->altitude);
    if(a<0 || a>= attitude_size - attitude_count) {
      syslog(LOG_ERR, "Error writing on attitude buffer: %m (%s)%d",
	   __FILE__, __LINE__);
      return;
    }
    attitude_count += a;
  }

  //Write gps buffer
  if (pdva_config.downsample.gps.M &&
      ticks % pdva_config.downsample.gps.M == 0) {
    a = snprintf(&gps_buffer[p*gps_size + gps_count],
                 gps_size - gps_count,
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
    if(a<0 || a>= gps_size - gps_count) {
      syslog(LOG_ERR, "Error writing on gps buffer: %m (%s)%d",
	   __FILE__, __LINE__);
      return;
    }
    gps_count += a;
  }

  //Write control buffer
  if (pdva_config.downsample.control.M &&
      ticks % pdva_config.downsample.control.M == 0) {
    a = snprintf(&control_buffer[p*control_size + control_count],
             control_size - control_count,
          "%f\t"
          "%u\t%u\t%u\t%u\n",
          time,

          control_out.aileron, control_out.elevator,
          control_out.throttle, control_out.rudder);

    if(a<0 || a>= control_size - control_count) {
      syslog(LOG_ERR, "Error writing on control buffer: %m (%s)%d",
	   __FILE__, __LINE__);
      return;
    }
    control_count += a;
  }

  //Increment the tick counter
  ticks++;
  ticks_write++;

  if(ticks_write >= write_interval){
    sensor_file.buffer = &sensor_buffer[p*sensor_size];
    sensor_file.count = sensor_count;
    sensor_count = 0;

    attitude_file.buffer = &attitude_buffer[p*attitude_size];
    attitude_file.count = attitude_count;
    attitude_count = 0;

    gps_file.buffer = &gps_buffer[p*gps_size];
    gps_file.count = gps_count;
    gps_count = 0;

    control_file.buffer = &control_buffer[p*control_size];
    control_file.count = control_count;
    control_count = 0;

    if(sem_post(&sensor_file.sem))
      syslog(LOG_ERR, "Error posting sensor semaphore: %m (%s)%d",
	   __FILE__, __LINE__);
    if(sem_post(&attitude_file.sem))
      syslog(LOG_ERR, "Error posting attitude semaphore: %m (%s)%d",
	   __FILE__, __LINE__);
    if(sem_post(&gps_file.sem))
      syslog(LOG_ERR, "Error posting gps semaphore: %m (%s)%d",
	   __FILE__, __LINE__);
    if(sem_post(&control_file.sem))
      syslog(LOG_ERR, "Error posting control semaphore: %m (%s)%d",
	   __FILE__, __LINE__);

    p = (p+1) % 2;
    ticks_write = 0;
  }


  //Unlock datalog mutex
  if (pthread_mutex_unlock(&datalog_mutex)) {
    syslog(LOG_ERR, "Error unlocking datalog mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }
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

  if(pthread_mutex_destroy(&datalog_mutex)){
    syslog(LOG_ERR, "Error destroying datalog mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }
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

