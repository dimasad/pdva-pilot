
/** @file
 * Implementation of the PID control library.
 */

/* *** Includes *** */

#include <syslog.h>

#include "pid.h"


/* *** Public functions *** */

void 
pid_init(pid_controller_t *pid, double min_action, double max_action,
	 double Kc, double Ti, double Td) {
  //Set the parameters
  pid->min_action = min_action;
  pid->max_action = max_action;
  pid->Kc = Kc;
  pid->Ti = Ti;
  pid->Td = Td;

  //Initialize the internal state
  pid->last_error = 0.0;
  pid->istate = 0.0;
  pid->mode = PID_AUTOMATIC;
}

double 
pid_update(pid_controller_t *pid, double ref, double meas, double Ts) {
  double error = ref - meas;

  //Calculate the proportional action
  double paction = error * pid->Kc;
  
  //Calculate the derivative action
  double daction = (error - pid->last_error) * pid->Td * pid->Kc / Ts;

  //Calculate the integral action
  double iaction = pid->istate + error * 0.5 * Ts * pid->Kc / pid->Ti;
  
  //Update the internal state
  pid->last_error = error;
  pid->istate += error * Ts * pid->Kc / pid->Ti;
  
  //Return and save the control action  
  return pid->action = paction + iaction + daction;
}
