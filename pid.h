
/** @file
 * Interface of the PID control library.
 */

/* *** Types *** */

typedef enum {
  PID_MANUAL,
  PID_AUTOMATIC
} pid_mode_t;

/// A PID controller definition.
typedef struct pid_controller {
  float min_action; ///< Lower limit of the control action.
  float max_action; ///< Upper limit of the control action.
  float kp; ///< Proportional gain.
  float ki; ///< Integral gain, kp*Ts/Ti.
  float kd; ///< Derivative gain, kp*Td/Ts.
  float df_pole; ///< Pole of the derivative filter in the Z-plane.
  float meas; ///< Current value of the measurement (process variable).
  float action; ///< Current control action value (manipulated variable).
  float iaction; ///< Current integral action.
  float daction; ///< Current derivative action.
  pid_mode_t mode; ///< Current mode of operation.
} pid_controller_t;


/* *** Functions *** */

float 
pid_update(pid_controller_t *pid, float ref, float meas);

void 
pid_init(pid_controller_t *pid, float min_action, float max_action,
	 float kp, float ti, float td, float df_pole, float period);

void 
pid_tune(pid_controller_t *pid, float kp, float ti, float td, float df_pole, 
	 float period);

/// Go to automatic mode with a bumpless transfer to a given action.
void 
pid_set_auto(pid_controller_t *pid, float action);

/// Set PID to manual mode.
void 
pid_set_manual(pid_controller_t *pid);

