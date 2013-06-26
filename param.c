
/** @file
 * Implementation of the parameter configuration infrastructure.
 */

/* *** Includes *** */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include <libconfig.h>

#include "param.h"


/* *** Prototypes *** */

/// Default parameter setter callback.
ret_status_t
default_param_setter(enum MAV_PARAM_TYPE type, const char* id,
		     void* location, param_value_union_t new_value);


/* *** Public functions *** */

param_value_union_t
param_get(param_def_t *param_def) {
  param_value_union_t ret;
  switch (param_def->type) {
  case MAV_PARAM_TYPE_UINT8:
    ret.param_uint8 = *(uint8_t*)param_def->location;
    break;
  case MAV_PARAM_TYPE_UINT16:
    ret.param_uint16 = *(uint16_t*)param_def->location;
    break;
  case MAV_PARAM_TYPE_UINT32:
    ret.param_uint32 = *(uint32_t*)param_def->location;
    break;
  case MAV_PARAM_TYPE_INT8:
    ret.param_int8 = *(int8_t*)param_def->location;
    break;
  case MAV_PARAM_TYPE_INT16:
    ret.param_int16 = *(int16_t*)param_def->location;
    break;
  case MAV_PARAM_TYPE_INT32:
    ret.param_int32 = *(int32_t*)param_def->location;
    break;
  case MAV_PARAM_TYPE_REAL32:
    ret.param_float = *(float*)param_def->location;
    break;
    
  default:
    ret.param_float = NAN;
    syslog(LOG_ERR, "Unsupported parameter type %d (%s)%d.",
	   param_def->type, __FILE__, __LINE__);
  }
  
  return ret;
}

ret_status_t
param_set(param_def_t* param_def, param_value_union_t new_value) {
  param_setter_t callback = (param_def->setter ? 
			     param_def->setter : &default_param_setter);
  
  return callback(param_def->type, param_def->id, 
		  param_def->location, new_value);
}

void
param_handler_init(param_handler_t *handler, size_t max_param_count) {
  handler->param_count = 0;
  handler->max_param_count = max_param_count;
  handler->param_def = malloc(max_param_count * sizeof(param_def_t));
  handler->id_map = g_hash_table_new(g_str_hash, g_str_equal);
}

void
param_handler_destroy(param_handler_t *handler) {
  g_hash_table_destroy(handler->id_map);
  free(handler->param_def);
}

void
param_register(param_handler_t *handler, enum MAV_PARAM_TYPE type,
	       const char *id, void *location, param_setter_t setter) {
  //Check if there is preallocated space available
  if (handler->param_count >= handler->max_param_count) {
    syslog(LOG_ERR, "Not enough space to register parameter (%s)%d.", 
	   __FILE__, __LINE__);
    return;
  }
  
  //Get the next available parameter definition
  param_def_t *def = handler->param_def + handler->param_count++;

  //Copy the fields
  def->type = type;
  def->location = location;
  def->setter = setter;
  strncpy(def->id, id, MAX_LENGTH_PARAM_ID);
  def->id[MAX_LENGTH_PARAM_ID] = 0; //Sentinel nul byte

  //Insert the definition in the id map hash table
  g_hash_table_insert(handler->id_map, (gpointer)id, def);
}

ret_status_t
param_load(param_handler_t *handler, const char *file) {
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
  for (int i = 0; i < handler->param_count; i++) {
    long long llvalue;
    double dvalue;
    param_def_t *param_def = handler->param_def + i;
    
    //Lookup the value
    switch (param_def->type) {
    case MAV_PARAM_TYPE_UINT8:
    case MAV_PARAM_TYPE_UINT16:
    case MAV_PARAM_TYPE_UINT32:
    case MAV_PARAM_TYPE_INT8:
    case MAV_PARAM_TYPE_INT16:
    case MAV_PARAM_TYPE_INT32:
      if (!config_lookup_int64(&config, param_def->id, &llvalue))
	continue;
      break;
    case MAV_PARAM_TYPE_REAL32:
      if (!config_lookup_float(&config, param_def->id, &dvalue))
	continue;
      break;
    default:
      syslog(LOG_ERR, "Unsupported parameter type %d (%s)%d.",
	     param_def->type, __FILE__, __LINE__);
      continue;
    }
    
    //Set the parameter value
    switch (param_def->type) {
    case MAV_PARAM_TYPE_UINT8:
      *(uint8_t*)param_def->location = llvalue;
      break;
    case MAV_PARAM_TYPE_UINT16:
      *(uint16_t*)param_def->location = llvalue;
      break;
    case MAV_PARAM_TYPE_UINT32:
      *(uint32_t*)param_def->location = llvalue;
      break;
    case MAV_PARAM_TYPE_INT8:
      *(int8_t*)param_def->location = llvalue;
      break;
    case MAV_PARAM_TYPE_INT16:
      *(int16_t*)param_def->location = llvalue;
      break;
    case MAV_PARAM_TYPE_INT32:
      *(int32_t*)param_def->location = llvalue;
      break;
    case MAV_PARAM_TYPE_REAL32:
      *(float*)param_def->location = dvalue;
      break;
    }
  }
  
  config_destroy(&config);
  return STATUS_SUCCESS;
  
 param_load_error:
  config_destroy(&config);
  return STATUS_FAILURE;
}

ret_status_t
param_save(param_handler_t *handler, const char *file) {
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
  
  //Populate configuration tree
  for (int i = 0; i < handler->param_count; i++) {
    long long llvalue;
    double dvalue;
    param_def_t *param_def = handler->param_def + i;
    
    //Get the parameter value
    switch (param_def->type) {
    case MAV_PARAM_TYPE_UINT8:
      llvalue = *(uint8_t*)param_def->location;
      break;
    case MAV_PARAM_TYPE_UINT16:
      llvalue = *(uint16_t*)param_def->location;
      break;
    case MAV_PARAM_TYPE_UINT32:
      llvalue = *(uint32_t*)param_def->location;
      break;
    case MAV_PARAM_TYPE_INT8:
      llvalue = *(int8_t*)param_def->location;
      break;
    case MAV_PARAM_TYPE_INT16:
      llvalue = *(int16_t*)param_def->location;
      break;
    case MAV_PARAM_TYPE_INT32:
      llvalue = *(int32_t*)param_def->location;
      break;
    case MAV_PARAM_TYPE_REAL32:
      dvalue = *(float*)param_def->location;
      break;
      
    default:
      syslog(LOG_ERR, "Unsupported parameter type %d (%s)%d.",
	     param_def->type, __FILE__, __LINE__);
      continue;
    }
    
    //Add to the configuration tree
    config_setting_t *setting;
    switch (param_def->type) {
    case MAV_PARAM_TYPE_UINT8:
    case MAV_PARAM_TYPE_UINT16:
    case MAV_PARAM_TYPE_UINT32:
    case MAV_PARAM_TYPE_INT8:
    case MAV_PARAM_TYPE_INT16:
    case MAV_PARAM_TYPE_INT32:
      setting = config_setting_add(root, param_def->id, CONFIG_TYPE_INT64);
      config_setting_set_int64(setting, llvalue);
      break;
    case MAV_PARAM_TYPE_REAL32:
      setting = config_setting_add(root, param_def->id, CONFIG_TYPE_FLOAT);
      config_setting_set_float(setting, dvalue);
      break;
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
  
  int sysid;
  if (config_lookup_int(&config, "sysid", &sysid))
    pdva_config->sysid = sysid;
  
  config_destroy(&config);
  return STATUS_SUCCESS;

 config_load_error:
  config_destroy(&config);
  return STATUS_FAILURE;
}


/* *** Internal functions *** */

ret_status_t
default_param_setter(enum MAV_PARAM_TYPE type, const char* id, 
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
