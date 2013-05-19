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


ret_status_t
setup() {
  //Configure logging
  openlog("pdva-pilot", LOG_CONS | LOG_PERROR | LOG_PID, LOG_USER);

  
  //Load parameters
  if (param_load(mav_params, PDVA_CONFIG_DIR "/mav_params.cfg"))
    syslog(LOG_WARNING, "Could not load parameters, using default values.");
  
  return STATUS_SUCCESS;
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
  
  param_save(mav_params, PDVA_CONFIG_DIR "/mav_params.cfg");

  teardown();
  return EXIT_SUCCESS;
}
