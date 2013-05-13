#include <stdint.h>
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


int setup() {
  //Configure logging
  openlog("pdva-pilog", LOG_CONS | LOG_PERROR | LOG_PID, LOG_USER);

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
