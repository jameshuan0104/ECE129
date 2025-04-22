#ifndef AHT20_H
#define AHT20_H

#include <stdint.h>

/**
 * @brief Initializes the AHT20 sensor.
 *
 * This function sets up the I2C communication and sends the necessary
 * command to initialize the sensor.
 */
void AHT20_init(void);

/**
 * @brief Reads temperature and humidity values from the AHT20 sensor.
 *
 * This function sends a trigger command to the AHT20 to start a measurement
 * and then reads the resulting data into internal variables.
 */
void AHT20_read_sensor(void);

/**
 * @brief Retrieves the most recent temperature reading.
 *
 * @return Temperature in degrees Celsius.
 */
float AHT20_get_temperature(void);

/**
 * @brief Retrieves the most recent humidity reading.
 *
 * @return Relative humidity as a percentage (0.0% to 100.0%).
 */
float AHT20_get_humidity(void);

#endif // AHT20_H
