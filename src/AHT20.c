#include "AHT20.h"

#include <stdio.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// AHT20 I2C address and I2C bus number
#define AHT20_I2C_ADDR 0x38     ///< I2C address of the AHT20 sensor
#define I2C_NUM I2C_NUM_0       ///< I2C port used for communication

// Global variables to store the most recent sensor values
static float humidity = 0.0f;
static float temperature = 0.0f;

/**
 * @brief Initializes the AHT20 sensor by setting up I2C and sending an init command.
 *
 * Configures the I2C bus with specified SDA/SCL pins and clock speed,
 * installs the I2C driver, and sends the initialization command (0xBE) to the sensor.
 */
void AHT20_init(void) {
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

    uint8_t init_buffer[1] = {0xBE}; // Initialization command for AHT20

    // Send init command
    i2c_master_write_to_device(I2C_NUM, AHT20_I2C_ADDR, init_buffer, 1, 100 / portTICK_PERIOD_MS);

    // Allow sensor to settle
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

/**
 * @brief Triggers a measurement and reads 6 bytes of data from the AHT20 sensor.
 *
 * The function sends the trigger command (0xAC, 0x33, 0x00), waits for the
 * measurement to complete, and then reads the 6-byte result from the sensor.
 * It extracts and converts both humidity and temperature values.
 */
void AHT20_read_sensor(void) {
    uint8_t trig_buffer[3] = {0xAC, 0x33, 0x00};
    uint8_t data_buffer[6];

    // Trigger measurement
    i2c_master_write_to_device(I2C_NUM, AHT20_I2C_ADDR, trig_buffer, 3, 100 / portTICK_PERIOD_MS);

    // Wait for measurement to complete (typical 80 ms delay)
    vTaskDelay(80 / portTICK_PERIOD_MS);

    // Read 6 bytes of result data
    i2c_master_read_from_device(I2C_NUM, AHT20_I2C_ADDR, data_buffer, 6, 100 / portTICK_PERIOD_MS);

    // Extract 20-bit raw humidity value
    uint32_t raw_humidity = (data_buffer[1] << 12) |
                            (data_buffer[2] << 4) |
                            (data_buffer[3] >> 4);

    // Extract 20-bit raw temperature value
    uint32_t raw_temperature = ((data_buffer[3] & 0x0F) << 16) |
                               (data_buffer[4] << 8) |
                               data_buffer[5];

    // Convert raw values to real-world units
    humidity = ((float)raw_humidity / 1048576.0f) * 100.0f;
    temperature = ((float)raw_temperature / 1048576.0f) * 200.0f - 50.0f;
}

/**
 * @brief Returns the most recent temperature value in Celsius.
 *
 * @return Temperature as a float in °C.
 */
float AHT20_get_temperature(void) {
    return temperature;
}

/**
 * @brief Returns the most recent relative humidity value in percentage.
 *
 * @return Humidity as a float from 0.0 to 100.0%.
 */
float AHT20_get_humidity(void) {
    return humidity;
}
