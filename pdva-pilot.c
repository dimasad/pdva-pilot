#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

#include <libconfig.h>

#include "param.h"
#include "pdva-pilot.h"


#ifndef LOG_PERROR
#define LOG_PERROR 0
///< Linux extension of syslog options that copies messages to stderr.
#endif// not LOG_PERROR


float dummy1 = 0.1; ///< A dummy parameter for testing
uint8_t dummy2 = 4; ///< Another dummy parameter

param_def_t mav_params[] = {
  {MAV_PARAM_TYPE_REAL32, "dummy1", &dummy1, NULL},
  {MAV_PARAM_TYPE_UINT8, "dummy2", &dummy2, NULL},
  PARAM_DEF_LIST_END //Sentinel
};


exit_status_t
default_param_updater(enum MAV_PARAM_TYPE type, const char* id, 
		      void* location, param_value_union_t new_value) {
  switch (type) {
  MAV_PARAM_TYPE_UINT8:
    *(uint8_t*)location = new_value.param_uint8;
    break;
  MAV_PARAM_TYPE_UINT16:
    *(uint16_t*)location = new_value.param_uint16;
    break;
  MAV_PARAM_TYPE_UINT32:
    *(uint32_t*)location = new_value.param_uint32;
    break;
  MAV_PARAM_TYPE_INT8:
    *(int8_t*)location = new_value.param_int8;
    break;
  MAV_PARAM_TYPE_INT16:
    *(int16_t*)location = new_value.param_int16;
    break;
  MAV_PARAM_TYPE_INT32:
    *(int32_t*)location = new_value.param_int32;
    break;
  MAV_PARAM_TYPE_REAL32:
    *(float*)location = new_value.param_float;
    break;
    
  default:
    syslog(LOG_WARNING, "Unsupported parameter type %d (%s)%d.",
	   type, __FILE__, __LINE__);
    return -1;
  }
  
  return 0;
}

exit_status_t
update_param(param_def_t* param_def, param_value_union_t new_value) {
  param_updater_t callback = (param_def->updater ? 
			      param_def->updater : &default_param_updater);
  
  return callback(param_def->type, param_def->id, 
		  param_def->location, new_value);
}


/**
 * Load runtime parameters from file.
 */
exit_status_t
param_load(param_def_t params[], const char* file) {
  //Create configuration file object
  config_t config;
  config_init(&config);

  //Read configuration file
  if (!config_read_file(&config, file)) {
    syslog(LOG_ERR, "Error loading parameters from `%s' %s (%s)l%d.",
	   file, config_error_text(&config), __FILE__, __LINE__);
    goto param_load_error;
  }
  
  //Load values into parameters
  for (int i = 0; IS_PARAM_DEF_LIST_END(params[i]); i++) {
    long lvalue;
    double dvalue;
    param_value_union_t pvalue;
    
    switch (params[i].type) {
    MAV_PARAM_TYPE_UINT8:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_uint8 = lvalue;
	if (lvalue < 0 || lvalue > UINT8_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    MAV_PARAM_TYPE_UINT16:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_uint16 = lvalue;
	if (lvalue < 0 || lvalue > UINT16_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    MAV_PARAM_TYPE_UINT32:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_uint32 = lvalue;
	if (lvalue < 0 || lvalue > UINT32_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    MAV_PARAM_TYPE_INT8:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_int8 = lvalue;
	if (lvalue < INT8_MIN || lvalue > INT8_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    MAV_PARAM_TYPE_INT16:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_int16 = lvalue;
	if (lvalue < INT16_MIN || lvalue > INT16_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    MAV_PARAM_TYPE_INT32:
      if (config_lookup_int(&config, params[i].id, &lvalue)) {
	pvalue.param_int32 = lvalue;
	if (lvalue < INT32_MIN || lvalue > INT32_MAX)
	  syslog(LOG_WARNING, "Parameter value %ld for `%s' out of range "
		 "(%s)%d.", lvalue, params[i].id, __FILE__, __LINE__);
	else
	  update_param(params + i, pvalue);
      }
      break;
    MAV_PARAM_TYPE_REAL32:
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
  return 0;
  
 param_load_error:
  config_destroy(&config);
  return -1;
}


exit_status_t
setup() {
  //Configure logging
  openlog("pdva-pilot", LOG_CONS | LOG_PERROR | LOG_PID, LOG_USER);

  
  //Load parameters
  if (param_load(mav_params, PDVA_CONFIG_DIR "/mav_params.cfg"))
    syslog(LOG_WARNING, "Could not load MAV parameters, using default values.");
  
  return 0;
}

void 
teardown() {
  closelog();
}


int
main(int argc, char* argv[]) {
  if (setup()) {
    syslog(LOG_CRIT, "Failure in pdva-pilot setup, aborting.");
    return EXIT_FAILURE;
  }
  
  teardown();
  return EXIT_SUCCESS;
}
