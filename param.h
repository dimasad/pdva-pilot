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

#include "mavlink_bridge.h"
#include "pdva-pilot.h"


/* *** Macros *** */
/// Maximum length of parameter id string
#define MAX_LENGTH_PARAM_ID 16


/* *** Types *** */

/// Configuration structure of the pdva-pilot instance.
typedef struct {
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

/// Initialize pdva_pilot_config_t object.
void 
pdva_config_init(pdva_pilot_config_t *pdva_config);

/// Load pdva-pilot configuration from file.
ret_status_t
pdva_config_load(pdva_pilot_config_t *pdva_config, const char *file);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif // not PDVA__PARAM_H
