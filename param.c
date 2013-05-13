#include <stdlib.h>
#include <syslog.h>
#include <libconfig.h>

#include "param.h"

/**
 * Load runtime parameters from `MAV_PARAM_FILE'.
 * @return 0 if success.
 */
int mav_param_load(mav_param_t params[]) {
  //Create configuration file object
  config_t config;
  config_init(&config);

  //Read configuration file
  if (!config_read_file(&config, MAV_PARAM_FILE)) {
    syslog(LOG_ERR, "Error reading MAV_PARAM_FILE `%s' %s (%s)l%d.",
	   MAV_PARAM_FILE, config_error_text(&config),
	   __FILE__, __LINE__);
    goto mav_param_load_error;
  }
  
  //Load values into parameters
  for (int i = 0; IS_MAV_PARAM_LIST_END(params[i]); i++) {
    long lvalue;
    double dvalue;
    
    switch (params[i].type) {
    MAV_PARAM_TYPE_UINT8:
      if (config_lookup_int(&config, params[i].id, &lvalue))
	*(uint8_t*)params[i].destination = lvalue;
      break;
    MAV_PARAM_TYPE_UINT16:
      if (config_lookup_int(&config, params[i].id, &lvalue))
	*(uint16_t*)params[i].destination = lvalue;
      break;
    MAV_PARAM_TYPE_UINT32:
      if (config_lookup_int(&config, params[i].id, &lvalue))
	*(uint32_t*)params[i].destination = lvalue;
      break;
    MAV_PARAM_TYPE_UINT64:
      if (config_lookup_int(&config, params[i].id, &lvalue))
	*(uint64_t*)params[i].destination = lvalue;
      break;
    MAV_PARAM_TYPE_INT8:
      if (config_lookup_int(&config, params[i].id, &lvalue))
	*(int8_t*)params[i].destination = lvalue;
      break;
    MAV_PARAM_TYPE_INT16:
      if (config_lookup_int(&config, params[i].id, &lvalue))
	*(int16_t*)params[i].destination = lvalue;
      break;
    MAV_PARAM_TYPE_INT32:
      if (config_lookup_int(&config, params[i].id, &lvalue))
	*(int32_t*)params[i].destination = lvalue;
      break;
    MAV_PARAM_TYPE_INT64:
      if (config_lookup_int(&config, params[i].id, &lvalue))
	*(int64_t*)params[i].destination = lvalue;
      break;
    MAV_PARAM_TYPE_REAL32:
      if (config_lookup_float(&config, params[i].id, &dvalue))
	*(float*)params[i].destination = dvalue;
      break;
    MAV_PARAM_TYPE_REAL64:
      if (config_lookup_float(&config, params[i].id, &dvalue))
	*(double*)params[i].destination = dvalue;
      break;
    }
  }
  
  config_destroy(&config);
  return 0;
  
 mav_param_load_error:
  config_destroy(&config);
  return -1;
}
