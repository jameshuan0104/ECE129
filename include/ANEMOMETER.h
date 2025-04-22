#ifndef ANEMOMETER_H
#define ANEMOMETER_H

#include <stdint.h>

/**
 * @brief Initializes the anemometer ADC hardware.
 *
 * Configures ADC width and attenuation for the anemometer input pin.
 */
void ANEMOMETER_init(void);

/**
 * @brief Reads the current filtered ADC value from the anemometer.
 *
 * Applies both a median and exponential moving average (EMA) filter.
 * @return Filtered ADC value (12-bit resolution).
 */
uint16_t ANEMOMETER_read_adc(void);

/**
 * @brief Reads wind speed in km/h based on current ADC reading.
 *
 * Converts the filtered ADC value using a linear calibration model.
 * @return Estimated wind speed in km/h.
 */
float ANEMOMETER_read_kmh(void);

#endif // ANEMOMETER_H
