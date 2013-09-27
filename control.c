
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

/// Aileron output
static double aileron_out = 0;

/// Elevator output
static double elevator_out = 0;

/// Throttle output
static double throttle_out = 0;

/// Rudder output
static double rudder_out = 0;

/// Altitude in meters above mean sea level
static double altitude = 0;

/// Airspeed in m/s.
static double airspeed = 0;

/// Control output
static control_out_t control_out =
       {.aileron = 0, .elevator = 0, .throttle = 0, .rudder = 0};



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

void
get_sensor_and_control_data(
       mavlink_sensor_head_data_t *sensor, control_out_t *control) {

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
  memcpy(sensor, &sensor_head_data, sizeof(mavlink_sensor_head_data_t));

  //Get the control data
  memcpy(control, &control_out, sizeof(control_out_t));

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
  //Initialize the pid controllers
  pid_init(&aileron_pid, 0, 1, 1, 0, 0, CONTROL_TIMER_PERIOD_S);
  pid_init(&elevator_pid, 0, 1, 1, 0, 0, CONTROL_TIMER_PERIOD_S);
  pid_init(&throttle_pid, 0, 1, 1, 0, 0, CONTROL_TIMER_PERIOD_S);
  pid_init(&rudder_pid, 0, 1, 1, 0, 0, CONTROL_TIMER_PERIOD_S);
  pid_init(&roll_pid, -M_PI_2, M_PI_2, 1, 0, 0, CONTROL_TIMER_PERIOD_S);
  pid_init(&pitch_pid, -M_PI_2, M_PI_2, 1, 0, 0, CONTROL_TIMER_PERIOD_S);
  
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
  timer_spec.it_interval.tv_sec = 0;
  timer_spec.it_value.tv_sec = 0;
  timer_spec.it_interval.tv_nsec = CONTROL_TIMER_PERIOD_NS;
  timer_spec.it_value.tv_nsec = CONTROL_TIMER_PERIOD_NS;
  
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
  
  //Get readings from sensor head
  if (sensor_head_read(&sensor_head_data)) {
    //How to proceed when failed to obtain sensor head measurements?
  }
  
  //Calculate the control action
  calculate_control();

  //Unlock mutex
  if (pthread_mutex_unlock(&mutex)) {
    syslog(LOG_ERR, "Error unlocking mutex: %m (%s)%d",
	   __FILE__, __LINE__);
  }
  
  //Write control action to the sensor head
  mavlink_msg_sensor_head_command_send(SENSOR_HEAD_COMM_CHANNEL,
				       control_out.aileron,
				       control_out.elevator,
				       control_out.throttle,
				       control_out.rudder, 0, 0, 0, 0);
}

static inline void 
calculate_control() {
  switch (control_configuration) {
  case ALTITUDE_FROM_POWER:
    throttle_out = pid_update(&throttle_pid,
			      altitude_ref - sensor_head_data.altitude * 0.01);
    
    pitch_ref = pid_update(&pitch_pid,
			   airspeed_ref - sensor_head_data.airspeed * 0.01);
    break;
  case ALTITUDE_FROM_PITCH:
    pitch_ref = pid_update(&pitch_pid,
			   altitude_ref - sensor_head_data.altitude * 0.01);
    
    throttle_out = pid_update(&throttle_pid,
			      airspeed_ref - sensor_head_data.airspeed * 0.01);
    break;
  }

  roll_ref = pid_update(&roll_pid,
			yaw_ref - sensor_head_data.att_est[2] * 0.001);
    
  aileron_out = pid_update(&aileron_pid,
			   roll_ref - sensor_head_data.att_est[0] * 0.001);
  
  elevator_out = pid_update(&elevator_pid,
			    pitch_ref - sensor_head_data.att_est[1] * 0.001);
  elevator_out += fabs(roll_ref) * elevator_ff;
  
  rudder_out = pid_update(&rudder_pid, - sensor_head_data.acc[1]);
  rudder_out += aileron_out * rudder_ff;

  control_out.aileron = aileron_out * 65535;
  control_out.elevator = elevator_out * 65535;
  control_out.throttle = throttle_out * 65535;
  control_out.rudder = rudder_out * 65535;
}

