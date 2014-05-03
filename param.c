
/** @file
 * Implementation of the parameter configuration infrastructure.
 */

/* *** Includes *** */

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include "param.h"
#include "control.h"
#include "datalog.h"


/* *** Prototypes *** */

/// Default parameter getter callback.
static param_value_union_t
default_param_getter(enum MAV_PARAM_TYPE type, const char *id, void *data);

/// Default parameter setter callback.
static ret_status_t
default_param_setter(enum MAV_PARAM_TYPE type, const char *id,
		     void *data, param_value_union_t new_value);


/* *** Public functions *** */

param_value_union_t
param_get(param_t *param) {
  param_getter_t callback = (param->getter ? 
                             param->getter : &default_param_getter);
  
  return callback(param->type, param->id, param->data);
}

ret_status_t
param_set(param_t* param, param_value_union_t new_value) {
  param_setter_t callback = (param->setter ? 
			     param->setter : &default_param_setter);
  
  return callback(param->type, param->id, param->data, new_value);
}

void
param_handler_init(param_handler_t *handler, size_t reserved_size) {
  handler->id_tbl = g_hash_table_new(g_str_hash, g_str_equal);
  handler->param_array = g_array_sized_new(false, false, sizeof(param_t), 
                                           reserved_size);
}

void
param_handler_destroy(param_handler_t *handler) {
  g_hash_table_destroy(handler->id_tbl);
  g_array_free(handler->param_array, TRUE);
}

uint16_t param_count(param_handler_t *handler) {
  return handler->param_array->len;
}

param_t *
param_lookup(param_handler_t *handler, const char *id, int16_t *index) {
  if (*index >= handler->param_array->len)
    return NULL;
  
  if (*index < 0) {
    gpointer index_ptr;
    if (g_hash_table_lookup_extended(handler->id_tbl, id, NULL, &index_ptr))
      *index  = GPOINTER_TO_UINT(index);
    else
      return NULL;
  }
  
  return &g_array_index(handler->param_array, param_t, *index);
}

void
param_register(param_handler_t *handler, enum MAV_PARAM_TYPE type,
               const char *id, void *data,
               param_getter_t getter, param_setter_t setter) {
  //Initialize the new definition structure
  param_t param = {
    .type = type,
    .data = data,
    .getter = getter,
    .setter = setter,
    .id = {0},
  };
  strncpy(param.id, id, MAX_LENGTH_PARAM_ID);

  //Get the array index of the new definition
  unsigned index = handler->param_array->len;
  
  //Insert the definition in the array
  g_array_append_val(handler->param_array, param);
  
  //Insert the id in the hash table
  g_hash_table_insert(handler->id_tbl, (gpointer)id, GUINT_TO_POINTER(index));
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
    param_t *param = &g_array_index(handler->param_array, param_t, i);
    
    //Lookup the value
    switch (param->type) {
    case MAV_PARAM_TYPE_UINT8:
    case MAV_PARAM_TYPE_UINT16:
    case MAV_PARAM_TYPE_UINT32:
    case MAV_PARAM_TYPE_INT8:
    case MAV_PARAM_TYPE_INT16:
    case MAV_PARAM_TYPE_INT32:
      if (!config_lookup_int64(&config, param->id, &llvalue))
	continue;
      break;
    case MAV_PARAM_TYPE_REAL32:
      if (!config_lookup_float(&config, param->id, &dvalue))
	continue;
      break;
    default:
      syslog(LOG_ERR, "Unsupported parameter type %d (%s)%d.",
	     param->type, __FILE__, __LINE__);
      continue;
    }
    
    //Set the parameter value
    switch (param->type) {
    case MAV_PARAM_TYPE_UINT8:
      param_set(param, (param_value_union_t){.param_uint8 = llvalue});
      break;
    case MAV_PARAM_TYPE_UINT16:
      param_set(param, (param_value_union_t){.param_uint16 = llvalue});
      break;
    case MAV_PARAM_TYPE_UINT32:
      param_set(param, (param_value_union_t){.param_uint32 = llvalue});
      break;
    case MAV_PARAM_TYPE_INT8:
      param_set(param, (param_value_union_t){.param_int8 = llvalue});
      break;
    case MAV_PARAM_TYPE_INT16:
      param_set(param, (param_value_union_t){.param_int16 = llvalue});
      break;
    case MAV_PARAM_TYPE_INT32:
      param_set(param, (param_value_union_t){.param_int32 = llvalue});
      break;
    case MAV_PARAM_TYPE_REAL32:
      param_set(param, (param_value_union_t){.param_float = dvalue});
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
  
  //Get pointer into root configuration setting.
  config_setting_t* root = config_root_setting(&config);
  
  //Populate configuration tree
  for (int i = 0; i < handler->param_array->len; i++) {
    long long llvalue;
    double dvalue;
    param_t *param = &g_array_index(handler->param_array, param_t, i);
    
    //Get the parameter value
    switch (param->type) {
    case MAV_PARAM_TYPE_UINT8:
      llvalue = param_get(param).param_uint8;
      break;
    case MAV_PARAM_TYPE_UINT16:
      llvalue = param_get(param).param_uint16;
      break;
    case MAV_PARAM_TYPE_UINT32:
      llvalue = param_get(param).param_uint32;
      break;
    case MAV_PARAM_TYPE_INT8:
      llvalue = param_get(param).param_int8;
      break;
    case MAV_PARAM_TYPE_INT16:
      llvalue = param_get(param).param_int16;
      break;
    case MAV_PARAM_TYPE_INT32:
      llvalue = param_get(param).param_int32;
      break;
    case MAV_PARAM_TYPE_REAL32:
      dvalue = param_get(param).param_float;
      break;
      
    default:
      syslog(LOG_ERR, "Unsupported parameter type %d (%s)%d.", param->type, 
             __FILE__, __LINE__);
      continue;
    }
    
    //Add to the configuration tree
    config_setting_t *setting;
    switch (param->type) {
    case MAV_PARAM_TYPE_UINT8:
    case MAV_PARAM_TYPE_UINT16:
    case MAV_PARAM_TYPE_UINT32:
    case MAV_PARAM_TYPE_INT8:
    case MAV_PARAM_TYPE_INT16:
    case MAV_PARAM_TYPE_INT32:
      setting = config_setting_add(root, param->id, CONFIG_TYPE_INT64);
      config_setting_set_int64(setting, llvalue);
      break;
    case MAV_PARAM_TYPE_REAL32:
      setting = config_setting_add(root, param->id, CONFIG_TYPE_FLOAT);
      config_setting_set_float(setting, dvalue);
      break;
    }
  }

  if(!config_write_file(&config, file))
    syslog(LOG_ERR, "Error writing parameters to `%s': %m (%s)%d.",
	   file, __FILE__, __LINE__);
  
  config_destroy(&config);
  return STATUS_SUCCESS;  
}

/// Free the resources associted with a pdva_pilot_config_t object.
void 
pdva_config_destroy(pdva_pilot_config_t *pdva_config) {
  downsample_destroy(&pdva_config->downsample.sensor);
  downsample_destroy(&pdva_config->downsample.attitude);
  downsample_destroy(&pdva_config->downsample.gps);
  downsample_destroy(&pdva_config->downsample.control);
}

/// Free the resources associted with a downsample_t object.
void
downsample_destroy(downsample_t * down){
  free(down->a);
  free(down->b);
}

/// Initialize pdva_pilot_config_t structure.
void 
pdva_config_init(pdva_pilot_config_t *pdva_config) {
  pdva_config->control_timer_period.tv_sec = 0;
  pdva_config->control_timer_period.tv_nsec = CONTROL_TIMER_PERIOD_NS;
  pdva_config->datalog_timer_period.tv_sec = 0;
  pdva_config->datalog_timer_period.tv_nsec = CONTROL_TIMER_PERIOD_NS;
  pdva_config->downsample.sensor.M = 1;
  pdva_config->downsample.sensor.n = 0;
  pdva_config->downsample.attitude.M = 1;
  pdva_config->downsample.attitude.n = 0;
  pdva_config->downsample.gps.M = 1;
  pdva_config->downsample.gps.n = 0;
  pdva_config->downsample.control.M = 1;
  pdva_config->downsample.control.n = 0;
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
  long long llvalue;
  if (config_lookup_int(&config, "sysid", &sysid))
    pdva_config->sysid = sysid;
  if (config_lookup_int64(&config, "control_timer_period_s", &llvalue))
    pdva_config->control_timer_period.tv_sec = llvalue;
  if (config_lookup_int64(&config, "control_timer_period_ns", &llvalue))
    pdva_config->control_timer_period.tv_nsec = llvalue;
  if (config_lookup_int64(&config, "datalog_timer_period_s", &llvalue))
    pdva_config->datalog_timer_period.tv_sec = llvalue;
  if (config_lookup_int64(&config, "datalog_timer_period_ns", &llvalue))
    pdva_config->datalog_timer_period.tv_nsec = llvalue;

  downsample_config_load(&config, &pdva_config->downsample.sensor,
         "sensor");
  downsample_config_load(&config, &pdva_config->downsample.attitude,
         "attitude");
  downsample_config_load(&config, &pdva_config->downsample.gps,
         "gps");
  downsample_config_load(&config, &pdva_config->downsample.control,
         "control");


  config_destroy(&config);
  return STATUS_SUCCESS;

 config_load_error:
  config_destroy(&config);
  return STATUS_FAILURE;
}


/// Load downsample configuration from file.
void
downsample_config_load(config_t *config, downsample_t *down, char *name){
  char param_name[MAX_LENGTH];
  int i;

  snprintf(param_name, MAX_LENGTH, "downsample_%s", name);
  config_lookup_int(config, param_name, &down->M);

  snprintf(param_name, MAX_LENGTH, "filter_%s_n", name);
  config_lookup_int(config, param_name, &down->n);


  down->a = (double *) malloc( (down->n+1) * sizeof(double) );
  down->b = (double *) malloc( (down->n+1) * sizeof(double) );

  for(i=0; i < down->n+1; ++i){
    down->a[i] = (i==0);
    down->b[i] = (i==0);
  }
  for(i=0; i < down->n+1; ++i){
    snprintf(param_name, MAX_LENGTH, "filter_%s_a_%d", name, i);
    config_lookup_float(config, param_name, &down->a[i]);
    snprintf(param_name, MAX_LENGTH, "filter_%s_b_%d", name, i);
    config_lookup_float(config, param_name, &down->b[i]);
  }

}

/* *** Internal functions *** */

static param_value_union_t
default_param_getter(enum MAV_PARAM_TYPE type, const char *id, void *data) {
  switch (type) {
  case MAV_PARAM_TYPE_UINT8:
    return (param_value_union_t){.param_uint8 = *(uint8_t*)data};
  case MAV_PARAM_TYPE_UINT16:
    return (param_value_union_t){.param_uint16 = *(uint16_t*)data};
  case MAV_PARAM_TYPE_UINT32:
    return (param_value_union_t){.param_uint32 = *(uint32_t*)data};
  case MAV_PARAM_TYPE_INT8:
    return (param_value_union_t){.param_int8 = *(int8_t*)data};
  case MAV_PARAM_TYPE_INT16:
    return (param_value_union_t){.param_int16 = *(int16_t*)data};
  case MAV_PARAM_TYPE_INT32:
    return (param_value_union_t){.param_int32 = *(int32_t*)data};
  case MAV_PARAM_TYPE_REAL32:
    return (param_value_union_t){.param_float = *(float*)data};
    
  default:
    syslog(LOG_ERR, "Unsupported parameter type %d (%s)%d.", type,
           __FILE__, __LINE__);
    return (param_value_union_t){.param_float = NAN};
  }
}

static ret_status_t
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
