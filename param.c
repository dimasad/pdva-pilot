#include <stdlib.h>
#include <syslog.h>
#include "confuse.h"

#include "param.h"

int mav_param_load(mav_param_t params[]) {
  //Retrieve the size of the parameter list
  size_t nparams = 0;
  for (nparams = 0; IS_MAV_PARAM_LIST_END(params[nparams]); nparams++);
  
  //Allocate the configuration option description vectors
  cfg_opt_t *opts = malloc((nparams + 1) * sizeof(cfg_opt_t));
  if (opts == NULL) {
    syslog(LOG_ERR, "Error in malloc %s(%d): %m.", __FILE__, __LINE__);
    return -1;
  }

  //Fill in the option types
  opts[nparams] = CFG_END();
  for (int i = 0; i < nparams; i++) {
    switch (params[i].type) {
    MAV_PARAM_TYPE_UINT8:
    MAV_PARAM_TYPE_UINT16:
    MAV_PARAM_TYPE_UINT32:
    MAV_PARAM_TYPE_UINT64:
    MAV_PARAM_TYPE_INT8:
    MAV_PARAM_TYPE_INT16:
    MAV_PARAM_TYPE_INT32:
    MAV_PARAM_TYPE_INT64:
      opts[i] = CFG_INT(params[i].id, *params[i].dest, CFGF_NONE);
      break;  
    MAV_PARAM_TYPE_REAL32:
    MAV_PARAM_TYPE_REAL64:
      opts[i] = CFG_FLOAT(params[i].id, *params[i].dest, CFGF_NONE);
      break;
    }
  }
  
  cfg_t *cfg = cfg_init(opts, CFGF_NONE);
  free(opts);
  
  switch(cfg_parse(cfg, MAV_PARAM_FILE)) {
  case CFG_FILE_ERROR:
    syslog(LOG_ERR, "Could not open MAV_PARAM_FILE `%s' %m (%s)%d.", 
	   MAV_PARAM_FILE, __FILE__, __LINE__);
    goto mav_param_load_error;
  case CFG_PARSE_ERROR:
    syslog(LOG_ERR, "Error parsing MAV_PARAM_FILE `%s' %s (%s)%d.", 
	   MAV_PARAM_FILE, cfg_error(), __FILE__, __LINE__);
    goto mav_param_load_error;
  }
  
  cfg_free(cfg);
  return 0;
  
 mav_param_load_error:
  cfg_free(cfg);
  return -1;
}
