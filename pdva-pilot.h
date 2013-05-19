#ifndef PDVA_PILOT_H
#define PDVA_PILOT_H

/** @file
 * General definitions and global structures of the pdva-pilot.
 */

/* *** Types *** */

/// Return status of pdva-pilot functions: 0 if success nonzero otherwise.
typedef enum {
  STATUS_SUCCESS = 0,
  STATUS_FAILURE = -1,
} ret_status_t;


#endif // not PDVA_PILOT_H
