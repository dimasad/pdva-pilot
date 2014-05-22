
/** @file
 * Implementation of the feedback control.
 */

/* *** Includes *** */

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <math.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>

#include "comm.h"
#include "control.h"
#include "mavlink_bridge.h"
#include "param.h"
#include "pid.h"

/* *** Prototypes *** */

static void alarm_handler(int signum, siginfo_t *info, void *context);
static inline void calculate_control();

/* *** Variables *** */

/// Configuration structure.
extern pdva_pilot_config_t pdva_config;

/// Mutex used to protect the internal variables
pthread_mutex_t mutex;

/* *** Internal variables *** */

/// Number of times the control loop has run.
static uint64_t ticks = 0;

/// Latest sensor data.
static mavlink_sensor_head_data_t sensor_head_data;

/// Control loop timer object.
static timer_t timer;

/// Aileron PID controller.
static pid_controller_t aileron_pid;

/// Elevator PID controller.
static pid_controller_t elevator_pid;

/// Throttle PID controller.
static pid_controller_t throttle_pid;

/// Rudder PID controller.
static pid_controller_t rudder_pid;

/// Desired roll angle PID controller.
static pid_controller_t roll_pid;

/// Desired pitch angle PID controller.
static pid_controller_t pitch_pid;

/// Aileron feedforward gain.
static double aileron_ff = 0;

/// Elevator feedforward gain.
static double elevator_ff = 0;

/// Throttle feedforward gain.
static double throttle_ff = 0;

/// Rudder feedforward gain.
static double rudder_ff = 0;

/// Multiloop control configuration.
uint8_t control_configuration = ALTITUDE_FROM_POWER;

/// Roll angle reference (setpoint).
static double roll_ref = 0;

/// Pitch angle reference (setpoint).
static double pitch_ref = 0;

/// Yaw angle reference (setpoint).
static double yaw_ref = 0;

/// Altitude reference (setpoint).
static double altitude_ref = 0;

/// Airspeed reference (setpoint).
static double airspeed_ref = 0;

/// Altitude in meters above mean sea level
static double altitude = 0;

/// Airspeed in m/s.
static double airspeed = 0;

/// Control output
static mavlink_sensor_head_command_t control_out =
       {.aileron = 0, .elevator = 0, .throttle = 0, .rudder = 0};

/// Sensor variables
static sensor_t sensor;

/// Attitude variables
static attitude_t attitude;

/// GPS variables
static gps_t gps;

/// Control variables
static control_t control;

/// Timestamp of system startup
static double time_startup;

/// Timestamp of the last message received
static double time_msg_received;

/// Last message sequential number
static uint8_t msg_seq;

/// Last message was received successfully
static uint8_t msg_ok;


/* *** Public functions *** */

/// The number of times the control loop has been called.
unsigned control_loop_ticks() {
  //Block all signals
  sigset_t oldmask, newmask;
  sigfillset(&newmask);
  sigprocmask(SIG_SETMASK, &newmask, &oldmask);

  //Lock mutex
  if (pthread_mutex_lock(&mutex)) {
    syslog(LOG_ERR, "Error locking mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }

  //Get the tick count
  uint64_t ret = ticks;

  //Unlock mutex
  if (pthread_mutex_unlock(&mutex)) {
    syslog(LOG_ERR, "Error unlocking mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }

  //Restore the signal mask
  sigprocmask(SIG_SETMASK, &oldmask, NULL);

  return ret;
}

/// Get the latest sensor head and control data for datalog.
void get_datalog_data(
       sensor_t *sensor_data, attitude_t *attitude_data, gps_t *gps_data,
       control_t *control_data, double *time, double *time_gps,
       uint8_t *seq, uint8_t *ok) {

  //Block all signals
  sigset_t oldmask, newmask;
  sigfillset(&newmask);
  sigprocmask(SIG_SETMASK, &newmask, &oldmask);

  //Lock mutex
  if (pthread_mutex_lock(&mutex)) {
    syslog(LOG_ERR, "Error locking mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }

  //Get the sensor data
  memcpy(sensor_data, &sensor, sizeof(sensor_t));

  //Get the attitude data
  memcpy(attitude_data, &attitude, sizeof(attitude_t));

  //Get the GPS data
  memcpy(gps_data, &gps, sizeof(gps_t));

  //Get the control data
  memcpy(control_data, &control, sizeof(control_t));

  //Get time, sequential number and status
  *time = time_msg_received;
  *time_gps = sensor_head_data.time_gps_ms / 1e3;
  *seq = msg_seq;
  *ok = msg_ok;

  //Unlock mutex
  if (pthread_mutex_unlock(&mutex)) {
    syslog(LOG_ERR, "Error unlocking mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }

  //Restore the signal mask
  sigprocmask(SIG_SETMASK, &oldmask, NULL);  
}

/// Get the latest sensor head and control data for telemetry.
void get_telemetry_data(
       mavlink_sensor_head_data_t *data,
       mavlink_sensor_head_command_t *control_data) {

  //Block all signals
  sigset_t oldmask, newmask;
  sigfillset(&newmask);
  sigprocmask(SIG_SETMASK, &newmask, &oldmask);

  //Lock mutex
  if (pthread_mutex_lock(&mutex)) {
    syslog(LOG_ERR, "Error locking mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }

  //Get the sensor head data
  memcpy(data, &sensor_head_data, sizeof(mavlink_sensor_head_data_t));

  //Get the control data
  memcpy(control_data, &control_out, sizeof(mavlink_sensor_head_command_t));

  //Unlock mutex
  if (pthread_mutex_unlock(&mutex)) {
    syslog(LOG_ERR, "Error unlocking mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }

  //Restore the signal mask
  sigprocmask(SIG_SETMASK, &oldmask, NULL);  
}

ret_status_t 
setup_control() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  time_startup = tv.tv_sec + 1e-6 * tv.tv_usec;

  double control_timer_period = pdva_config.control_timer_period.tv_sec
                              +pdva_config.control_timer_period.tv_nsec / 1e9;

  double kp = 0.001, ki = 0.001;

  //Initialize the pid controllers
  pid_init(&aileron_pid, 0, 1, kp, ki, 0, control_timer_period);
  pid_init(&elevator_pid, 0, 1, kp, ki, 0, control_timer_period);
  pid_init(&throttle_pid, 0, 1, kp, ki, 0, control_timer_period);
  pid_init(&rudder_pid, 0, 1, kp, ki, 0, control_timer_period);
  pid_init(&roll_pid, -M_PI_2, M_PI_2, kp, ki, 0, control_timer_period);
  pid_init(&pitch_pid, -M_PI_2, M_PI_2, kp, ki, 0, control_timer_period);
  
  //Setup the interrupt handler
  struct sigaction alarm_action;
  alarm_action.sa_sigaction = &alarm_handler;
  alarm_action.sa_flags = SA_RESTART;
  sigfillset(&alarm_action.sa_mask);
  sigaction(SIGALRM, &alarm_action, NULL);
  
  //Create timer
  if (timer_create(CLOCK_REALTIME, NULL, &timer)) {
    syslog(LOG_ERR, "Error creating control loop timer: %m (%s)%d",
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }

  //Initialize mutex
  if (pthread_mutex_init(&mutex, NULL)) {
    syslog(LOG_ERR, "Error initializing mutex: %m (%s)%d",
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }
  
  return STATUS_SUCCESS;
}

ret_status_t
start_control() {
  struct itimerspec timer_spec;
  timer_spec.it_interval.tv_sec = pdva_config.control_timer_period.tv_sec;
  timer_spec.it_value.tv_sec = pdva_config.control_timer_period.tv_sec;
  timer_spec.it_interval.tv_nsec = pdva_config.control_timer_period.tv_nsec;
  timer_spec.it_value.tv_nsec = pdva_config.control_timer_period.tv_nsec;
  
  if (timer_settime(timer, 0, &timer_spec, NULL)) {
    syslog(LOG_ERR, "Error setting control loop timer: %m (%s)%d.",
	   __FILE__, __LINE__);
    return STATUS_FAILURE;
  }
  
  return STATUS_SUCCESS;
}

void 
teardown_control() {
  timer_delete(timer);
  if(pthread_mutex_destroy(&mutex)){
    syslog(LOG_ERR, "Error destroying mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }
}


/* *** Internal functions *** */

static void 
alarm_handler(int signum, siginfo_t *info, void *context) {
  if (signum != SIGALRM)
    return;

  //Lock mutex
  if (pthread_mutex_lock(&mutex)) {
    syslog(LOG_ERR, "Error locking mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }

  //Increment the tick counter
  ticks++;
  
  //Communicate with sensor head
  if (sensor_head_read_write(&sensor_head_data, &msg_seq, &control_out)) {
    msg_ok = 0;
    //How to proceed when failed to obtain sensor head measurements?
    //Unlock mutex
    if (pthread_mutex_unlock(&mutex)) {
      syslog(LOG_ERR, "Error unlocking mutex: %m (%s)%d",
  	   __FILE__, __LINE__);
    }
    return;

  }
  msg_ok = 1;
  struct timeval tv;
  gettimeofday(&tv, NULL);
  time_msg_received = tv.tv_sec + 1e-6 * tv.tv_usec - time_startup;

  // Convert sensor head data to double format in SI units
  sensor_head_data_convert(&sensor_head_data, &sensor, &attitude, &gps);
  
  
  //Calculate the control action
  calculate_control();

  // Convert sensor head command from double format to PWM
  sensor_head_command_convert(&control_out, &control);

  //Unlock mutex
  if (pthread_mutex_unlock(&mutex)) {
    syslog(LOG_ERR, "Error unlocking mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }
}

static inline void 
calculate_control() {
  switch (control_configuration) {
  case ALTITUDE_FROM_POWER:
    control.throttle = pid_update(&throttle_pid,
			      altitude_ref - attitude.altitude);
    
    pitch_ref = pid_update(&pitch_pid,
			   airspeed_ref - attitude.airspeed);
    break;
  case ALTITUDE_FROM_PITCH:
    pitch_ref = pid_update(&pitch_pid,
			   altitude_ref - attitude.altitude);
    
    control.throttle = pid_update(&throttle_pid,
			      airspeed_ref - attitude.airspeed);
    break;
  }

  roll_ref = pid_update(&roll_pid,
			yaw_ref - attitude.att_est[2]);
    
  control.aileron = pid_update(&aileron_pid,
			   roll_ref - attitude.att_est[0]);
  
  control.elevator = pid_update(&elevator_pid,
			    pitch_ref - attitude.att_est[1]);
  control.elevator += fabs(roll_ref) * elevator_ff;
  
  control.rudder = pid_update(&rudder_pid, - sensor.acc[1]);
  control.rudder += control.aileron * rudder_ff;

}

// Convert sensor head data to double format in SI units
void sensor_head_data_convert(mavlink_sensor_head_data_t *data,
        sensor_t *sensor_data, attitude_t *attitude_data, gps_t *gps_data) {
  int i;

  // Sensor variables
  // acc
  for(i=0;i<3;++i)
    sensor_data->acc[i] = pdva_config.gain.sensor.acc[i] * data->acc[i]
                       + pdva_config.offset.sensor.acc[i];
  // gyro
  for(i=0;i<3;++i)
    sensor_data->gyro[i] = pdva_config.gain.sensor.gyro[i] * data->gyro[i]
                        + pdva_config.offset.sensor.gyro[i];
  // gyro_temp
  sensor_data->gyro_temp = pdva_config.gain.sensor.gyro_temp * data->gyro_temp
                        + pdva_config.offset.sensor.gyro_temp;
  // mag
  for(i=0;i<3;++i)
    sensor_data->mag[i] = pdva_config.gain.sensor.mag[i] * data->mag[i]
                       + pdva_config.offset.sensor.mag[i];
  // dyn_press
  sensor_data->dyn_press = pdva_config.gain.sensor.dyn_press * data->dyn_press
                        + pdva_config.offset.sensor.dyn_press;
  // stat_press
  sensor_data->stat_press = pdva_config.gain.sensor.stat_press * data->stat_press
                         + pdva_config.offset.sensor.stat_press;

  // Attitude variables
  // att_est
  for(i=0;i<3;++i)
    attitude_data->att_est[i] = pdva_config.gain.attitude.att_est[i] * data->att_est[i]
                             + pdva_config.offset.attitude.att_est[i];
  // airspeed
  attitude_data->airspeed = pdva_config.gain.attitude.airspeed * data->airspeed
                         + pdva_config.offset.attitude.airspeed;
  // altitude
  attitude_data->altitude = pdva_config.gain.attitude.altitude * data->altitude
                         + pdva_config.offset.attitude.altitude;

  // GPS variables
  // lat_gps
  gps_data->lat_gps = pdva_config.gain.gps.lat_gps * data->lat_gps
                   + pdva_config.offset.gps.lat_gps;
  // lon_gps
  gps_data->lon_gps = pdva_config.gain.gps.lon_gps * data->lon_gps
                   + pdva_config.offset.gps.lon_gps;
  // alt_gps
  gps_data->alt_gps = pdva_config.gain.gps.alt_gps * data->alt_gps
                   + pdva_config.offset.gps.alt_gps;
  // hdg_gps
  gps_data->hdg_gps = pdva_config.gain.gps.hdg_gps * data->hdg_gps
                   + pdva_config.offset.gps.hdg_gps;
  // speed_gps
  gps_data->speed_gps = pdva_config.gain.gps.speed_gps * data->speed_gps
                     + pdva_config.offset.gps.speed_gps;
  // pos_fix_gps
  gps_data->pos_fix_gps = pdva_config.gain.gps.pos_fix_gps * data->pos_fix_gps
                       + pdva_config.offset.gps.pos_fix_gps;
  // nosv_gps
  gps_data->nosv_gps = pdva_config.gain.gps.nosv_gps * data->nosv_gps
                    + pdva_config.offset.gps.nosv_gps;
  // hdop_gps
  gps_data->hdop_gps = pdva_config.gain.gps.hdop_gps * data->hdop_gps
                    + pdva_config.offset.gps.hdop_gps;

}

// Convert sensor head command from double format to PWM
void sensor_head_command_convert(mavlink_sensor_head_command_t *data,
        control_t *control_data) {

  // Control variables
  // aileron
  data->aileron = pdva_config.gain.control.aileron * control_data->aileron
               + pdva_config.offset.control.aileron;
  // elevator
  data->elevator = pdva_config.gain.control.elevator * control_data->elevator
                + pdva_config.offset.control.elevator;
  // throttle
  data->throttle = pdva_config.gain.control.throttle * control_data->throttle
                + pdva_config.offset.control.throttle;
  // rudder
  data->rudder = pdva_config.gain.control.rudder * control_data->rudder
              + pdva_config.offset.control.rudder;

}

