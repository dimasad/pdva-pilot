
/** @file
 * Implementation of the PID control library.
 */

/* *** Includes *** */

#include <syslog.h>

#include "pid.h"


/* *** Public functions *** */

void 
pid_init(pid_controller_t *pid, double min_action, double max_action,
	 double kp, double ki, double kd, double Ts) {
  //Set the parameters
  pid->min_action = min_action;
  pid->max_action = max_action;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->Ts = Ts;

  //Initialize the internal state
  pid->last_error = 0.0;
  pid->istate = 0.0;
}

double 
pid_update(pid_controller_t *pid, double error) {
  //Calculate the proportional action
  double paction = error * pid->kp;

  //Calculate the integral action
  double iaction = pid->istate + error * 0.5 * pid->Ts * pid->ki;
  
  //Calculate the derivative action
  double daction = (error - pid->last_error) * pid->kd / pid->Ts;
  
  //Update the internal state
  pid->last_error = error;
  pid->istate += error * pid->Ts * pid->ki;
  pid->istate = SATURATE(pid->istate, pid->min_action, pid->max_action);
  
  //Return and save the control action  
  return pid->action = SATURATE(paction + iaction + daction,
				pid->min_action, pid->max_action);
}
