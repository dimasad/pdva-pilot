
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

#include "comm.h"
#include "control.h"
#include "mavlink_bridge.h"
#include "param.h"
#include "pid.h"


/* *** Macros *** */

#ifndef CONTROL_TIMER_PERIOD_NS
#define CONTROL_TIMER_PERIOD_NS 500000000L
///< Period of the control loop in nanosecods.
#endif // not CONTROL_TIMER_PERIOD_NS

#define CONTROL_TIMER_PERIOD_S (CONTROL_TIMER_PERIOD_NS / 1e9)

/* *** Prototypes *** */

static void alarm_handler(int signum, siginfo_t *info, void *context);
static inline void calculate_control();


/* *** Internal variables *** */

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
uint8_t control_configuration = ALTITUDE_FROM_THROTTLE;

/// Yaw angle reference.
static double yaw_ref = 0;

/// Altitude reference.
static double altitude_ref = 0;

/// Airspeed reference.
static double airspeed_ref = 0;

/// Aileron output (range 0.0 to 1.0)
static double aileron_out = 0;

/// Elevator output (range 0.0 to 1.0)
static double elevator_out = 0;

/// Throttle output (range 0.0 to 1.0)
static double throttle_out = 0;

/// Rudder output (range 0.0 to 1.0)
static double rudder_out = 0;


/* *** Public functions *** */

ret_status_t 
setup_control() {
  //Initialize the pid controllers
  pid_init(&aileron_pid, 0, 1, 1, INFINITY, 0);
  pid_init(&elevator_pid, 0, 1, 1, INFINITY, 0);
  pid_init(&throttle_pid, 0, 1, 1, INFINITY, 0);
  pid_init(&rudder_pid, 0, 1, 1, INFINITY, 0);
  pid_init(&roll_pid, -M_PI_2, M_PI_2, 1, INFINITY, 0);
  pid_init(&pitch_pid, -M_PI_2, M_PI_2, 1, INFINITY, 0);
  
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
}


/* *** Internal functions *** */

static void 
alarm_handler(int signum, siginfo_t *info, void *context) {
  if (signum != SIGALRM)
    return;

  //Get readings from sensor head
  if (read_sensor_head(&sensor_head_data)) {
    //How to proceed when failed to obtain sensor head measurements?
  }
  
  //Calculate the control action
  calculate_control();

  //Write control action to the sensor head
  mavlink_msg_servo_output_raw_send(SENSOR_HEAD_COMM_CHANNEL, 0, 0,
				    aileron_pid.action*65535,
				    elevator_pid.action*65535,
				    throttle_pid.action*65535,
				    rudder_pid.action*65535, 0, 0, 0, 0);
}

static inline void 
calculate_control() {
  if (control_configuration == ALTITUDE_FROM_THROTTLE) {
    double roll_ref = pid_update(&roll_pid, yaw_ref,
				 sensor_head_data.att_est[2], 
				 CONTROL_TIMER_PERIOD_S);    
  } else {
    
  }
}
