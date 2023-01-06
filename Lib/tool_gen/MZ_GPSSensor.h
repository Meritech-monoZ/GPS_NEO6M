/*
 * MZ_GPS_Sensor.h
 *  Created on: 11-July-2023
 *      Author: Piyush
 */

#ifndef MZ_GPSSENSOR_H_
#define MZ_GPSSENSOR_H_

#include "MZ_error_handler.h"

/** @fn mz_error_t gps_app_init(void)
 * @brief GPS Application initialization API.  START
 * 1. It call all necessary initializations before application start
 * 2. It create the main gps application
 * 3. It set all variables to its initialization states
 * @return MZ_OK/MZ_FAIL
 */
mz_error_t gps_app_init(void);


#endif /* MZ_GPSSENSOR_H_ */
