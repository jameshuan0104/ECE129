#include "AHT20.h"

#include <stdio.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// AHT20 I2C address and I2C bus number
#define AHT20_I2C_ADDR 0x38
#define I2C_NUM I2C_NUM_0

// Global variables to hold sensor data (humidity and temperature)
static float humidity = 0.0f;
static float temperature = 0.0f;

/**
 * @brief Initializes the AHT20 sensor by sending an initialization command.
 */
void AHT20_init(void) {
    // Initialize I2C communication with AHT20
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_NUM, &conf);
    i2c_driver_install(I2C_NUM, conf.mode, 0, 0, 0);

    // Initialization command to AHT20 (0xBE)
    uint8_t init_buffer[1] = {0xBE};

    // Write initialization command to the AHT20 sensor over I2C
    i2c_master_write_to_device(I2C_NUM, AHT20_I2C_ADDR, init_buffer, 1, 100 / portTICK_PERIOD_MS);
    
    // Delay for sensor to initialize properly
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

/**
 * @brief Triggers the AHT20 sensor to start measurement and reads the data.
 */
void AHT20_read_sensor(void) {
    // Command to trigger measurement (0xAC, 0x33, 0x00)
    uint8_t trig_buffer[3] = {0xAC, 0x33, 0x00};
    
    // Buffer to store the sensor data (6 bytes)
    uint8_t data_buffer[6];

    // Write trigger command to AHT20 to start measurement
    i2c_master_write_to_device(I2C_NUM, AHT20_I2C_ADDR, trig_buffer, 3, 100 / portTICK_PERIOD_MS);
    
    // Wait for the measurement to complete (80ms is typical for AHT20)
    vTaskDelay(80 / portTICK_PERIOD_MS);

    // Read the 6 bytes of data from the sensor (humidity and temperature)
    i2c_master_read_from_device(I2C_NUM, AHT20_I2C_ADDR, data_buffer, 6, 100 / portTICK_PERIOD_MS);

    // Calculate the raw humidity data from the sensor response, first 20 bits is humidity
    uint32_t raw_humidity = (data_buffer[1] << 12) |
                            (data_buffer[2] << 4) |
                            (data_buffer[3] >> 4);

    // Calculate the raw temperature data from the sensor response, last 20 bits is temperature
    uint32_t raw_temperature = ((data_buffer[3] & 0x0F) << 16) |
                               (data_buffer[4] << 8) |
                               data_buffer[5];

    // Convert the raw humidity to percentage
    humidity = ((float)raw_humidity / 1048576.0f) * 100.0f;
    
    // Convert the raw temperature to Celsius (range from -40 ~ +85 ℃)
    temperature = ((float)raw_temperature / 1048576.0f) * 200.0f - 50.0f;
}

/**
 * @brief Returns the latest temperature reading from the sensor.
 * @return Temperature in Celsius.
 */
float AHT20_get_temperature(void) {
    return temperature;
}

/**
 * @brief Returns the latest humidity reading from the sensor.
 * @return Humidity as a percentage (0 to 100%).
 */
float AHT20_get_humidity(void) {
    return humidity;
}
