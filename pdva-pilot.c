#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

#include "param.h"


#ifndef LOG_PERROR
#define LOG_PERROR 0
///< Linux extension of syslog options that copies messages to stderr.
#endif// not LOG_PERROR


double dummy1 = 0.1; ///< A dummy parameter for testing
uint8_t dummy2 = 4; ///< Another dummy parameter

mav_param_t mav_params[] = {
  {MAV_PARAM_TYPE_REAL64, "dummy1", &dummy1},
  {MAV_PARAM_TYPE_UINT8, "dummy2", &dummy2},
  MAV_PARAM_LIST_END //Sentinel
};
size_t nparams = sizeof(mav_params)/sizeof(mav_param_t) - 1;

int setup() {
  //Configure logging
  openlog("pdva-pilog", LOG_CONS | LOG_PERROR | LOG_PID, LOG_USER);
  
  //Load parameters
  if (mav_param_load(mav_params))
    syslog(LOG_WARNING, "Could not load MAV parameters, using default values.");
  
  return 0;
}

void teardown() {
  closelog();
}


int main(int argc, char* argv[]) {
  if (setup()) {
    syslog(LOG_CRIT, "Failure in pdva-pilot setup, aborting.");
    return EXIT_FAILURE;
  }
  
  teardown();
  return EXIT_SUCCESS;
}
