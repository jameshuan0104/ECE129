#ifndef SEN0545_H
#define	SEN0545_H

#include <stdint.h>

/**
 * @brief Enumerated types of rainfall intensity as returned by the SEN0545 sensor.
 */
typedef enum {
    RAIN_STATUS_NO_RAIN       = 0,  
    RAIN_STATUS_LIGHT_RAIN    = 1,   
    RAIN_STATUS_MODERATE_RAIN = 2,  
    RAIN_STATUS_HEAVY_RAIN    = 3,   
    RAIN_STATUS_ERROR         = 255  
} rain_status_t;

/**
 * @brief Enumerated system statuses reported by the SEN0545 sensor.
 */
typedef enum {
    SYSTEM_STATUS_NORMAL          = 0,  
    SYSTEM_STATUS_SPI_ERROR       = 1,   
    SYSTEM_STATUS_LED_A_FAILURE   = 2,   
    SYSTEM_STATUS_LED_B_FAILURE   = 3,  
    SYSTEM_STATUS_CALIBRATION_ERR = 4,   
    SYSTEM_STATUS_COMMS_ERROR     = 255  //Communication error (UART failure, invalid data, etc.)
} system_status_t;

/**
 * @brief Initialize UART and prepare communication with the SEN0545 sensor.
 */
void SEN0545_init(void);

/**
 * @brief Query the sensor to determine the current rainfall status.
 * @return A rain_status_t enum representing the rainfall intensity.
 */
rain_status_t SEN0545_rainfall_status(void);

/**
 * @brief Query the sensor to retrieve its internal system status.
 * @return A system_status_t enum representing the sensor’s health.
 */
system_status_t SEN0545_system_status(void);

#endif // SEN0545_H
