#ifndef PDVA_PILOT_H
#define PDVA_PILOT_H

/** @file
 * General definitions and global structures of the pdva-pilot.
 */


/* *** Includes *** */

#include <stdint.h>
#include "mavlink/v1.0/common/mavlink.h"


/* *** Macros *** */

#ifndef PDVA_CONFIG_DIR
#define PDVA_CONFIG_DIR "/etc/pdva"
///< Path of the pdva-pilot configuration directory.
#endif // not PDVA_CONFIG_DIR

/// Marks the end of a list of param_def_t elements.
#define PARAM_DEF_LIST_END {MAV_PARAM_TYPE_ENUM_END, "", NULL, NULL}

/// True if a param_def_t is the PARAM_DEF_LIST_END sentinel.
#define IS_PARAM_DEF_LIST_END(pdef) (pdef.type == MAV_PARAM_TYPE_ENUM_END)


/* *** Types *** */

/// Exit status of pdva-pilot functions: 0 if success nonzero otherwise.
typedef int exit_status_t;

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
typedef int (*param_updater_t)(enum MAV_PARAM_TYPE type,
			       const char* id, void* location,
			       param_value_union_t new_value);

/// Definition of a runtime parameter.
typedef struct {
  enum MAV_PARAM_TYPE type;
  char id[16];
  void *location;
  param_updater_t updater;
} param_def_t;


/* *** Functions *** */

/// Updates the parameter to a new value.
exit_status_t
update_param(param_def_t *param_def, param_value_union_t new_value);

/// Default parameter updater callback.
exit_status_t
default_param_updater(enum MAV_PARAM_TYPE type, const char* id,
		      void* location, param_value_union_t new_value);

#endif // not PDVA_PILOT_H
