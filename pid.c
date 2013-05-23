
/** @file
 * Implementation of the PID control library.
 */

/* *** Includes *** */

#include "pid.h"


/* *** Public functions *** */

float 
pid_update(pid_controller_t *pid, float ref, float meas) {
  syslog(LOG_WARNING, "Manual/automatic mode not yet implemented.")

  //Calculate the error
  float error = ref - meas;

  //Update the derivative action
  float findiff = meas - pid->meas;
  pid->daction = pid->daction*pid->df_pole + (1 -pid->df_pole)*pid->kd*findiff;
  
  //Store the measurement value
  pid->meas = meas;
  
  //Calculate the control action
  pid->action = pid->kp*error + pid->daction + pid->iaction + pid->ki*error;
  
  //Perform saturation and anti-windup
  if (pid->action <= pid->min_action)
    pid->action = pid->min_action;
  else if(pid->action >= pid->max_action)
    pid->action = pid->max_action;
  else
    pid->daction += pid->ki * error;
  
  return pid->action;
}
