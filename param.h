#ifndef PDVA__PARAM_H
#define PDVA__PARAM_H

/** @file
 * Contains the parameter configuration infrastructure.
 */


#include <stdint.h>
#include "mavlink/v1.0/common/mavlink.h"


#ifndef MAV_PARAM_FILE
#define MAV_PARAM_FILE "/etc/pdva/mav_params.cfg"
///< The location of the runtime parameter configuration file.
#endif // not MAV_PARAM_FILE


/// Represents the definition of a runtime parameter.
typedef struct {
  enum MAV_PARAM_TYPE type;
  const char id[16];
  void *destination;
} mav_param_t;


/// Marks the end of a mav_param_t list.
#define MAV_PARAM_LIST_END {MAV_PARAM_TYPE_ENUM_END, "", NULL}

/// True if a mav_param_t is the end of a list
#define IS_MAV_PARAM_LIST_END(param) (param.type == MAV_PARAM_TYPE_ENUM_END)


/// Load the parameters from file.
int mav_param_load(mav_param_t params[]);

#endif // not PDVA__PARAM_H
