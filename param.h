#ifndef PDVA__PARAM_H
#define PDVA__PARAM_H

/** @file
 * Contains the parameter configuration infrastructure.
 */

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

/* *** Includes *** */

#include <stddef.h>
#include <stdint.h>

#include <glib.h>
#include <libconfig.h>

#include "mavlink_bridge.h"
#include "pdva-pilot.h"


/* *** Macros *** */
/// Maximum length of parameter id string
#define MAX_LENGTH_PARAM_ID 16


/* *** Types *** */
/// Sensor structure
typedef struct {
  double acc[3]; ///< Accelerometer readings (m/s^2)
  double gyro[3]; ///< Gyrometer readings (rad/s)
  double gyro_temp; ///< Gyrometer temperature (K)
  double mag[3]; ///< Magnetometer readings (T)
  double dyn_press; ///< Dynamic pressure (Pa)
  double stat_press; ///< Static pressure (Pa)
} sensor_t;

/// Attitude structure
typedef struct {
  double att_est[3]; ///< Estimated attitude angles (roll, pitch, yaw in radians)
  double airspeed; ///< Airspeed (m/s)
  double altitude; ///< Altitude above sea level (m)
} attitude_t;

/// GPS structure
typedef struct {
  double lat_gps; ///< GPS latitude (deg)
  double lon_gps; ///< GPS longitude (deg)
  double alt_gps; ///< GPS altitude above MSL (m)
  double hdg_gps; ///< GPS heading (radians)
  double speed_gps; ///< GPS groundspeed (m/s)
  double pos_fix_gps; ///< GPS Position Fix Status
  double nosv_gps; ///< GPS Number of Satellites Used
  double hdop_gps; ///< GPS Horizontal Dilution of Precision
} gps_t;

/// Control structure
typedef struct {
  double aileron; ///< Aileron command
  double elevator; ///< Elevator command
  double throttle; ///< Throttle command
  double rudder; ///< Rudder command
} control_t;

/// All variables structure
typedef struct {
  sensor_t sensor; ///< Sensor variables
  attitude_t attitude; ///< Attitude variables
  gps_t gps; ///< GPS variables
  control_t control; ///< Control variables
} sensor_head_t;


/// Downsample structure for a single file
typedef struct {
  int M;     ///< Downsampling factor.
  int n;     ///< Order of the digital low-pass filter
  double *a; ///< Denominator coefficients (from a[0] to a[n]).
  double *b; ///< Numerator coefficients (from b[0] to b[n]).
} downsample_t;

/// Downsample structure for datalogging
typedef struct {
  downsample_t sensor;
  downsample_t attitude;
  downsample_t gps;
  downsample_t control;
} datalog_downsample_t;

/// Configuration structure of the pdva-pilot instance.
typedef struct {
  struct timespec control_timer_period;
  struct timespec datalog_timer_period;
  uint32_t spi_speed_hz;
  int datalog_write_ms;
  datalog_downsample_t downsample;
  sensor_head_t gain;
  sensor_head_t offset;
  uint8_t sysid;
} pdva_pilot_config_t;

/// Union of acceptable mavlink parameter types.
typedef union {
  float param_float;
  int32_t param_int32;
  uint32_t param_uint32;
  int16_t param_int16;
  uint16_t param_uint16;
  int8_t param_int8;
  uint8_t param_uint8;
} param_value_union_t;


/// Parameter set callback, should return STATUS_SUCCESS if successfully set.
typedef ret_status_t (*param_setter_t)(enum MAV_PARAM_TYPE type,
				       const char *id, void *data,
				       param_value_union_t new_value);

/// Parameter get callback.
typedef param_value_union_t (*param_getter_t)(enum MAV_PARAM_TYPE type,
                                              const char *id, void *data);


/// Definition of a runtime parameter.
typedef struct {
  enum MAV_PARAM_TYPE type;
  char id[MAX_LENGTH_PARAM_ID + 1];
  void *data;
  param_getter_t getter;
  param_setter_t setter;
} param_t;

/// The parameter handler object
typedef struct {
  GArray *param_array;
  GHashTable *id_tbl;
} param_handler_t;


/* *** Functions *** */

/// Get current value of a given parameter
param_value_union_t
param_get(param_t *param);

/// Set the parameter value
ret_status_t
param_set(param_t *param, param_value_union_t new_value);

/// Initialize the param_handler_t object.
void
param_handler_init(param_handler_t *handler, size_t reserved_size);

/// Free the resources associted with a param_handler_t object.
void
param_handler_destroy(param_handler_t *handler);

/// The number of parameters registered with the handler.
uint16_t
param_count(param_handler_t *handler);

/// Lookup a parameter in the parameter handler.
/// If `index < 0` the parameter is looked up by id, otherwise by index.
/// @param[in] handler The parameter handler object.
/// @param[in] id The parameter id.
/// @param[in,out] index The index of the requested parameter. 
/// @return Pointer to requested parameter or NULL if parameter not found.
param_t *
param_lookup(param_handler_t *handler, const char *id, int16_t *index);

/// Register a parameter definition with the handler
void
param_register(param_handler_t *handler, enum MAV_PARAM_TYPE type,
	       const char *id, void *data, param_getter_t getter, 
               param_setter_t setter);

/// Load runtime parameters from given file.
ret_status_t
param_load(param_handler_t *handler, const char *file);

/// Save runtime parameters into given file.
ret_status_t
param_save(param_handler_t *handler, const char *file);

/// Free the resources associted with a pdva_pilot_config_t object.
void 
pdva_config_destroy(pdva_pilot_config_t *);

/// Free the resources associted with a downsample_t object.
void
downsample_destroy(downsample_t *);

/// Initialize pdva_pilot_config_t object.
void 
pdva_config_init(pdva_pilot_config_t *pdva_config);

/// Load pdva-pilot configuration from file.
ret_status_t
pdva_config_load(pdva_pilot_config_t *pdva_config, const char *file);

/// Load downsample configuration from file.
void
downsample_config_load(config_t *config, downsample_t *down, char *name);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // not PDVA__PARAM_H
