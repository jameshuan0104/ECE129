#include "SEN0545.h"

#include <stdio.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define UART_NUM UART_NUM_1  // UART port used to communicate with the SEN0545 sensor
#define BUF_SIZE 1024        // Size of UART RX/TX buffer

/**
 * @brief Initialize UART2 to communicate with the SEN0545 rain sensor.
 */
void SEN0545_init() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Apply UART configuration
    esp_err_t err = uart_param_config(UART_NUM, &uart_config);
    if (err == ESP_OK) {
        printf("UART configured successfully for SEN0545\n");
    } else {
        printf("Failed to configure UART for SEN0545\n");
    }

    // Install UART driver with TX and RX buffers
    err = uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    if (err == ESP_OK) {
        printf("UART driver installed successfully for SEN0545\n");
    } else {
        printf("Failed to install UART driver for SEN0545\n");
    }

    // Set TX to GPIO10 and RX to GPIO9
    uart_set_pin(UART_NUM, GPIO_NUM_10, GPIO_NUM_9, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

/**
 * @brief Calculate 8-bit CRC for sensor communication.
 *
 * This is not currently used in your commands, but may be needed for other advanced packets.
 *
 * @param ptr Pointer to data buffer
 * @param len Length of data
 * @return uint8_t CRC value
 */
static uint8_t xCal_crc(uint8_t *ptr, uint32_t len) {
    uint8_t crc = 0xFF;
    uint8_t i;

    while (len--) {
        crc ^= *ptr++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x131;  // Polynomial used by DFRobot SEN0545
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Query the sensor for rainfall intensity.
 *
 * Possible response codes:
 * 0x00 = No rain
 * 0x01 = Light rain
 * 0x02 = Moderate rain
 * 0x03 = Heavy rain
 */
rain_status_t SEN0545_rainfall_status() {
    uint8_t command[5] = {0x3A, 0x01, 0x00, 0x00, 0x0D};  // Query rainfall status

    // Send command to sensor
    int bytes_written = uart_write_bytes(UART_NUM, command, sizeof(command));
    if (bytes_written != sizeof(command)) {
        return RAIN_STATUS_ERROR;
    }

    // Read sensor response
    uint8_t response[5];
    int len = uart_read_bytes(UART_NUM, response, sizeof(response), 100 / portTICK_PERIOD_MS);

    if ((xCal_crc(response + 1, 4) != 0) || (response[0] != 0x3A)){
        printf("Received %d bytes: ", len);
        for (int i = 0; i < len; i++) {
            printf("0x%02X ", response[i]);
        }
        printf("\n");
        return RAIN_STATUS_ERROR;
    }

    switch (response[2]) {
        case 0x00: 
            return RAIN_STATUS_NO_RAIN;
        case 0x01: 
            return RAIN_STATUS_LIGHT_RAIN;
        case 0x02: 
            return RAIN_STATUS_MODERATE_RAIN;
        case 0x03: 
            return RAIN_STATUS_HEAVY_RAIN;
        default:   
            return RAIN_STATUS_ERROR;
    }
}

/**
 * @brief Query the sensor for internal system status.
 *
 * Possible response codes:
 * 0x00 = System normal
 * 0x01 = SPI communication error
 * 0x02 = LED A damaged
 * 0x03 = LED B damaged
 * 0x04 = Optical calibration error
 */
system_status_t SEN0545_system_status() {
    uint8_t command[5] = {0x3A, 0x02, 0x00, 0x00, 0xC7};  // Query system status

    // Send command to sensor
    int bytes_written = uart_write_bytes(UART_NUM, command, sizeof(command));
    if (bytes_written != sizeof(command)) {
        return SYSTEM_STATUS_COMMS_ERROR;
    }

    // Read sensor response
    uint8_t response[5];
    int len = uart_read_bytes(UART_NUM, response, sizeof(response), 100 / portTICK_PERIOD_MS);

    if ((xCal_crc(response + 1, 4) != 0) || (response[0] != 0x3A)){
        printf("Received %d bytes: ", len);
        for (int i = 0; i < len; i++) {
            printf("0x%02X ", response[i]);
        }
        printf("\n");
        return SYSTEM_STATUS_COMMS_ERROR;
    }

    // Interpret system status
    switch (response[2]) {
        case 0x00: 
            return SYSTEM_STATUS_NORMAL;
        case 0x01: 
            return SYSTEM_STATUS_SPI_ERROR;
        case 0x02: 
            return SYSTEM_STATUS_LED_A_FAILURE;
        case 0x03: 
            return SYSTEM_STATUS_LED_B_FAILURE;
        case 0x04: 
            return SYSTEM_STATUS_CALIBRATION_ERR;
        default:   
            return SYSTEM_STATUS_COMMS_ERROR;
    }

}
