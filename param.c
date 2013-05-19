
/** @file
 * Implementation of the parameter configuration infrastructure.
 */

/* *** Includes *** */

#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <libconfig.h>

#include "param.h"


/* *** Function prototypes *** */

static ret_status_t
update_param(param_def_t* param_def, param_value_union_t new_value);


/* *** Public functions *** */

ret_status_t
default_param_updater(enum MAV_PARAM_TYPE type, const char* id, 
		      void* location, param_value_union_t new_value) {
  switch (type) {
  case MAV_PARAM_TYPE_UINT8:
    *(uint8_t*)location = new_value.param_uint8;
    break;
  case MAV_PARAM_TYPE_UINT16:
    *(uint16_t*)location = new_value.param_uint16;
    break;
  case MAV_PARAM_TYPE_UINT32:
    *(uint32_t*)location = new_value.param_uint32;
    break;
  case MAV_PARAM_TYPE_INT8:
    *(int8_t*)location = new_value.param_int8;
    break;
  case MAV_PARAM_TYPE_INT16:
    *(int16_t*)location = new_value.param_int16;
    break;
  case MAV_PARAM_TYPE_INT32:
    *(int32_t*)location = new_value.param_int32;
    break;
  case MAV_PARAM_TYPE_REAL32:
    *(float*)location = new_value.param_float;
    break;
    
  default:
    syslog(LOG_WARNING, "Unsupported parameter type %d (%s)%d.",
	   type, __FILE__, __LINE__);
    return STATUS_FAILURE;
  }
  
  return STATUS_SUCCESS;
}

ret_status_t
param_load(param_def_t params[], const char* file) {
  //Create configuration file object
  config_t config;
  config_init(&config);

  //Read configuration file
  if (!config_read_file(&config, file)) {
    syslog(LOG_ERR, "Error reading parameters file `%s': %s at line %d (%s)%d.",
	   file, config_error_text(&config), config_error_line(&config), 
	   __FILE__, __LINE__);
    goto param_load_error;
  }
  
  //Load values into parameters
  for (int i = 0; !IS_PARAM_DEF_LIST_END(params[i]); i++) {
    long lvalue;
    double dvalue;
    param_value_union_t pvalue;
    
    switch (params[i].type) {
    case MAV_PARAM_TYPE_UINT8:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_uint8 = lvalue;
	if (lvalue < 0 || lvalue > UINT8_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    case MAV_PARAM_TYPE_UINT16:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_uint16 = lvalue;
	if (lvalue < 0 || lvalue > UINT16_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    case MAV_PARAM_TYPE_UINT32:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_uint32 = lvalue;
	if (lvalue < 0 || lvalue > UINT32_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    case MAV_PARAM_TYPE_INT8:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_int8 = lvalue;
	if (lvalue < INT8_MIN || lvalue > INT8_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    case MAV_PARAM_TYPE_INT16:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_int16 = lvalue;
	if (lvalue < INT16_MIN || lvalue > INT16_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    case MAV_PARAM_TYPE_INT32:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_int32 = lvalue;
	if (lvalue < INT32_MIN || lvalue > INT32_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    case MAV_PARAM_TYPE_REAL32:
      if (config_lookup_float(&config, params[i].id, &dvalue)) {
	pvalue.param_float = dvalue;
	update_param(params + i, pvalue);
      }
      break;
      
    default:
      syslog(LOG_WARNING, "Unsupported parameter type %d (%s)%d.",
	     params[i].type, __FILE__, __LINE__);
    }   
  } 
  
  config_destroy(&config);
  return STATUS_SUCCESS;
  
 param_load_error:
  config_destroy(&config);
  return STATUS_FAILURE;
}

ret_status_t
param_save(param_def_t params[], const char* file) {
  //Create configuration file object
  config_t config;
  config_init(&config);

  //Read configuration file
  if (!config_read_file(&config, file)) {
    syslog(LOG_ERR, "Error reading parameters file `%s' %s (%s)%d.",
	   file, config_error_text(&config), __FILE__, __LINE__);
    goto param_save_error;
  }
  
  //Get pointer into root configuration setting.
  config_setting_t* root = config_root_setting(&config);
  
  //Load values into parameters
  for (int i = 0; !IS_PARAM_DEF_LIST_END(params[i]); i++) {
    config_setting_t* setting = config_setting_get_member(root, params[i].id);
    
    switch (params[i].type) {
    case MAV_PARAM_TYPE_UINT8:
      if(!setting)
	setting = config_setting_add(root, params[i].id, CONFIG_TYPE_INT);
      config_setting_set_int(setting, *(uint8_t*)params[i].location);
      break;
    case MAV_PARAM_TYPE_UINT16:
      if(!setting)
	setting = config_setting_add(root, params[i].id, CONFIG_TYPE_INT);
      config_setting_set_int(setting, *(uint16_t*)params[i].location);
      break;
    case MAV_PARAM_TYPE_UINT32:
      if(!setting)
	setting = config_setting_add(root, params[i].id, CONFIG_TYPE_INT);
      config_setting_set_int(setting, *(uint32_t*)params[i].location);
      break;
    case MAV_PARAM_TYPE_INT8:
      if(!setting)
	setting = config_setting_add(root, params[i].id, CONFIG_TYPE_INT);
      config_setting_set_int(setting, *(int8_t*)params[i].location);
      break;
    case MAV_PARAM_TYPE_INT16:
      if(!setting)
	setting = config_setting_add(root, params[i].id, CONFIG_TYPE_INT);
      config_setting_set_int(setting, *(int16_t*)params[i].location);
      break;
    case MAV_PARAM_TYPE_INT32:
      if(!setting)
	setting = config_setting_add(root, params[i].id, CONFIG_TYPE_INT);
      config_setting_set_int(setting, *(int32_t*)params[i].location);
      break;
    case MAV_PARAM_TYPE_REAL32:
      if(!setting)
	setting = config_setting_add(root, params[i].id, CONFIG_TYPE_FLOAT);
      config_setting_set_float(setting, *(float*)params[i].location);
      break;
      
    default:
      syslog(LOG_WARNING, "Unsupported parameter type %d (%s)%d.",
	     params[i].type, __FILE__, __LINE__);
    }   
  } 

  if(!config_write_file(&config, file))
    syslog(LOG_ERR, "Error writing parameters to `%s': %m (%s)%d.",
	   file, __FILE__, __LINE__);

  config_destroy(&config);
  return STATUS_SUCCESS;  

 param_save_error:
  config_destroy(&config);
  return STATUS_FAILURE;
}

void 
pdva_config_destroy(pdva_pilot_config_t *pdva_config) {
}

/// Initialize pdva_pilot_config_t structure.
void 
pdva_config_init(pdva_pilot_config_t *pdva_config) {
  pdva_config->sysid = 0;
}

/// Load pdva-pilot configuration from file.
ret_status_t
pdva_config_load(pdva_pilot_config_t *pdva_config, const char *file) {
  //Create configuration file object
  config_t config;
  config_init(&config);

  //Read configuration file
  if (!config_read_file(&config, file)) {
    syslog(LOG_ERR, "Error reading pdva-pilot configuration file `%s': %s "
	   "at line %d (%s)%d.", file, config_error_text(&config), 
	   config_error_line(&config), __FILE__, __LINE__);
    goto config_load_error;
  }
  
  long sysid;
  if (config_lookup_int(&config, "sysid", &sysid))
    pdva_config->sysid = sysid;

  config_destroy(&config);
  return STATUS_SUCCESS;

 config_load_error:
  config_destroy(&config);
  return STATUS_FAILURE;
}


/* *** Internal functions *** */

/// Updates the parameter to a new value.
static ret_status_t
update_param(param_def_t* param_def, param_value_union_t new_value) {
  param_updater_t callback = (param_def->updater ? 
			      param_def->updater : &default_param_updater);
  
  return callback(param_def->type, param_def->id, 
		  param_def->location, new_value);
}
