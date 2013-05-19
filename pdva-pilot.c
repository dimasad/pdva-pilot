/* *** Includes *** */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <syslog.h>

#include <libconfig.h>

#include "comm.h"
#include "mavlink_bridge.h"
#include "param.h"
#include "pdva-pilot.h"



/* *** Macros *** */

#ifndef PDVA_CONFIG_DIR
#define PDVA_CONFIG_DIR "/etc/pdva"
///< Path of the pdva-pilot configuration directory.
#endif // not PDVA_CONFIG_DIR

#ifndef LOG_PERROR
#define LOG_PERROR 0
///< Linux extension of syslog options that copies messages to stderr.
#endif// not LOG_PERROR


/* *** Variables *** */

mavlink_system_t mavlink_system;
pdva_pilot_config_t pdva_config;


/* *** Mavlink parameters *** */

float dummy1 = 0.1; ///< A dummy parameter for testing.
uint8_t dummy2 = 4; ///< Another dummy parameter.

param_def_t mav_params[] = {
  {MAV_PARAM_TYPE_REAL32, "dummy1", &dummy1, NULL},
  {MAV_PARAM_TYPE_UINT8, "dummy2", &dummy2, NULL},
  PARAM_DEF_LIST_END //Sentinel
};


/* *** Internal functions *** */

ret_status_t
setup() {
  //Configure logging
  openlog("pdva-pilot", LOG_CONS | LOG_PERROR | LOG_PID, LOG_USER);

  //Load pdva-pilot configuration
  pdva_config_init(&pdva_config);
  if (pdva_config_load(&pdva_config, PDVA_CONFIG_DIR "/pdva-pilot.cfg"))
    syslog(LOG_WARNING, "Could not load pdva-pilot configuration, "
	   "using default values.");
  
  //Set the mavlink_system structure
  mavlink_system.sysid = pdva_config.sysid;
  mavlink_system.compid = MAV_COMP_ID_SYSTEM_CONTROL;
  mavlink_system.type = MAV_TYPE_FIXED_WING;
  mavlink_system.state = MAV_STATE_BOOT;
  mavlink_system.mode = MAV_MODE_PREFLIGHT;
  mavlink_system.nav_mode = 0;
  
  //Load parameters
  if (param_load(mav_params, PDVA_CONFIG_DIR "/mav_params.cfg"))
    syslog(LOG_WARNING, "Could not load parameters, using default values.");
  
  //Setup the communication module
  if (setup_comm())
    syslog(LOG_WARNING, "Could not setup communication module.");

  return STATUS_SUCCESS;
}

void 
teardown() {
  pdva_config_destroy(&pdva_config);
  teardown_comm();
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
