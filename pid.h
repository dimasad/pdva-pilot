#ifndef PDVA__PID_H
#define PDVA__PID_H

/** @file
 * Interface of the PID control library.
 */

/* *** Macros *** */

#define SATURATE(val, min, max) (val <= min ? min : (val >= max ? max : val))


/* *** Types *** */

typedef enum {
  PID_MANUAL,
  PID_AUTOMATIC
} pid_mode_t;

/// A PID controller definition.
typedef struct pid_controller {
  double min_action; ///< Lower limit of the control action.
  double max_action; ///< Upper limit of the control action.
  double action; ///< Last action value (manipulated variable).
  double kp; ///< Proportional gain.
  double ki; ///< Integral gain.
  double kd; ///< Derivative gain.
  double Ts; ///< Sampling time.
  double last_error; ///< Last value of the error.
  double istate; ///< State of the integral action.
} pid_controller_t;


/* *** Functions *** */

void 
pid_init(pid_controller_t *pid, double min_action, double max_action,
	 double kp, double ki, double kd, double Ts);

double 
pid_update(pid_controller_t *pid, double error);

/*
/// Go to automatic mode with a bumpless transfer to a given action.
void 
pid_set_auto(pid_controller_t *pid, double action);

/// Set PID to manual mode.
void 
pid_set_manual(pid_controller_t *pid);
*/

#endif // not PDVA__PID_H
