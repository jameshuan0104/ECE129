#include "ANEMOMETER.h"

#include <stdio.h>
#include <driver/adc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define ANEMO_ADC_CHANNEL ADC1_CHANNEL_6   ///< ADC channel for anemometer

// Internal static state
static uint16_t adc = 0;
static float kmh = 0;

//------------------- Median Filter ---------------------------------------------------

/**
 * @brief 3-point median filter to suppress outlier noise in ADC signal.
 *
 * Maintains a circular buffer of the last 3 samples and returns the median.
 *
 * @param new_sample New ADC reading
 * @return Median-filtered ADC value
 */
static uint16_t median_filter(uint16_t new_sample) {
    static uint16_t buffer[3] = {0};
    static uint8_t index = 0;

    buffer[index] = new_sample;
    index = (index + 1) % 3;

    uint16_t a = buffer[0];
    uint16_t b = buffer[1];
    uint16_t c = buffer[2];

    if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
    if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
    return c;
}

//------------------- Exponential Moving Average (EMA) Filter -------------------------

#define EMA_ALPHA 0.1f   ///< Smoothing factor (0 < alpha < 1)

static float ema_value = 0.0f;
static bool ema_initialized = false;

/**
 * @brief Applies exponential moving average (EMA) filter to ADC signal.
 *
 * Smooths out rapid fluctuations while retaining trend information.
 *
 * @param new_sample Median-filtered ADC value
 * @return Smoothed ADC value
 */
static uint16_t ema_filter(int new_sample) {
    if (!ema_initialized) {
        ema_value = (float)new_sample;
        ema_initialized = true;
    } else {
        ema_value = EMA_ALPHA * new_sample + (1.0f - EMA_ALPHA) * ema_value;
    }
    return (uint16_t)ema_value;
}

//------------------- Public Interface ------------------------------------------------

void ANEMOMETER_init(void){
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ANEMO_ADC_CHANNEL, ADC_ATTEN_DB_0);
}

uint16_t ANEMOMETER_read_adc(void) {
    adc = ema_filter(median_filter(adc1_get_raw(ANEMO_ADC_CHANNEL)));
    return adc;
}

//------------------- Conversion Logic ------------------------------------------------

#define M_SLOPE      0.01616f   ///< Slope for ADC to km/h conversion
#define B_INTERCEPT  4.17f      ///< Intercept for ADC to km/h conversion

/**
 * @brief Converts filtered ADC value to wind speed (km/h).
 *
 * Uses a linear model based on calibrated ADC-speed data.
 *
 * @return Estimated wind speed in km/h.
 */
float ANEMOMETER_read_kmh(void) {
    ANEMOMETER_read_adc();  // Updates `adc` static var
    if (adc == 0){
        return 0;
    }
    kmh = M_SLOPE * (float)adc + B_INTERCEPT;
    return kmh;
}
