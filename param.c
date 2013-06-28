
/** @file
 * Implementation of the parameter configuration infrastructure.
 */

/* *** Includes *** */

#include <math.h>
#include <stdbool.h>
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
param_handler_init(param_handler_t *handler, size_t reserved_size) {
  handler->id_tbl = g_hash_table_new(g_str_hash, g_str_equal);
  handler->param_array = g_array_sized_new(false, false, sizeof(param_def_t), 
                                           reserved_size);
}

void
param_handler_destroy(param_handler_t *handler) {
  g_hash_table_destroy(handler->id_tbl);
  g_array_free(handler->param_array, TRUE);
}

void
param_register(param_handler_t *handler, enum MAV_PARAM_TYPE type,
               const char *id, void *location, param_setter_t setter) {
  //Initialize the new definition structure
  param_def_t def = {
    .type = type,
    .location = location,
    .setter = setter,
    .id = {0},
  };
  strncpy(def.id, id, MAX_LENGTH_PARAM_ID);

  //Get the array index of the new definition
  unsigned index = handler->param_array->len;

  //Insert the definition in the array
  g_array_append_val(handler->param_array, def);
  
  //Insert the id in the hash table
  g_hash_table_insert(handler->id_tbl, (gpointer)id, GINT_TO_POINTER(index));
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
  for (int i = 0; i < handler->param_array->len; i++) {
    long long llvalue;
    double dvalue;
    param_def_t *param_def = &g_array_index(handler->param_array, 
                                            param_def_t, i);
    
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
  for (int i = 0; i < handler->param_array->len; i++) {
    long long llvalue;
    double dvalue;
    param_def_t *param_def = &g_array_index(handler->param_array, 
                                            param_def_t, i);
    
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
