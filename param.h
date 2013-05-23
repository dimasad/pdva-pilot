#ifndef PDVA__PARAM_H
#define PDVA__PARAM_H

/** @file
 * Contains the parameter configuration infrastructure.
 */


/* *** Includes *** */

#include <stdint.h>

#include "mavlink_bridge.h"
#include "pdva-pilot.h"


/* *** Macros *** */

/// Marks the end of a list of param_def_t elements.
#define PARAM_DEF_LIST_END {MAV_PARAM_TYPE_ENUM_END, "", NULL, NULL}

/// True if a param_def_t is the PARAM_DEF_LIST_END sentinel.
#define IS_PARAM_DEF_LIST_END(pdef) ((pdef).type == MAV_PARAM_TYPE_ENUM_END)


/* *** Types *** */

/// Global configuration structure of each the pdva-pilot instance.
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


/// Parameter update callback, should return 0 if successfully updated.
typedef ret_status_t (*param_updater_t)(enum MAV_PARAM_TYPE type,
					const char* id, void* location,
					param_value_union_t new_value);

/// Definition of a runtime parameter.
typedef struct {
  enum MAV_PARAM_TYPE type;
  char id[16];
  void *location;
  param_updater_t updater;
} param_def_t;


/* *** Global variables *** */

extern pdva_pilot_config_t pdva_config;


/* *** Functions *** */

/// Get current value of a given parameter
param_value_union_t
param_get(param_def_t *param_def);

/// Load runtime parameters from given file.
ret_status_t
param_load(param_def_t params[], const char* file);

/// Save runtime parameters into given file.
ret_status_t
param_save(param_def_t params[], const char* file);

/// Free the resources associted with the pdva_pilot_config_t object.
void 
pdva_config_destroy(pdva_pilot_config_t *);

/// Initialize pdva_pilot_config_t structure.
void 
pdva_config_init(pdva_pilot_config_t *pdva_config);

/// Load pdva-pilot configuration from file.
ret_status_t
pdva_config_load(pdva_pilot_config_t *pdva_config, const char *file);

/// Updates the parameter to a new value.
ret_status_t
update_param(param_def_t* param_def, param_value_union_t new_value);

#endif // not PDVA__PARAM_H
