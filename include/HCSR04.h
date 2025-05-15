#ifndef HCSR04_H
#define HCSR04_H

#include <stdint.h>

/**
 * @brief Initialize HC-SR04 sensor
 */
void HCSR04_init(void);

/**
 * @function    HCSR04_GetDistance(void)
 * @brief       Returns the calculated distance in mm using the sensor model determined
 *              experimentally. 
 *              No I/O should be done in this function
 * @return      distance in mm
 */
unsigned int HCSR04_GetDistance(void);

/**
 * @function    HCSR04_GetTimeofFlight(void)
 * @brief       Returns the raw microsecond duration of the echo from the sensor.
 *              NO I/O should be done in this function.
 * @return      time of flight in uSec
 */
unsigned int HCSR04_GetTimeofFlight(void);

#endif // HCSR04_H