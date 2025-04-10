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
uint8_t xCal_crc(uint8_t *ptr, uint32_t len) {
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
void SEN0545_rainfall_status() {
    uint8_t command[5] = {0x3A, 0x01, 0x00, 0x00, 0x0D};  // Query rainfall status

    // Send command to sensor
    int bytes_written = uart_write_bytes(UART_NUM, command, sizeof(command));
    if (bytes_written != sizeof(command)) {
        printf("Failed to send command to sensor.\n");
        return;
    }

    // Read sensor response
    uint8_t response[5];
    int len = uart_read_bytes(UART_NUM, response, sizeof(response), 100 / portTICK_PERIOD_MS);

    if (len > 0) {
        printf("Received %d bytes: ", len);
        for (int i = 0; i < len; i++) {
            printf("0x%02X ", response[i]);
        }
        printf("\n");
    } else {
        printf("No response\n");
    }

    // Interpret and display rain status
    if (len == 5 && response[0] == 0x3A) {
        switch (response[2]) {
            case 0x00: printf("No rain detected.\n"); break;
            case 0x01: printf("Light rain detected.\n"); break;
            case 0x02: printf("Moderate rain detected.\n"); break;
            case 0x03: printf("Heavy rain detected.\n"); break;
            default:   printf("Unknown rain code: 0x%02X\n", response[2]);
        }
    } else {
        printf("Failed to read rain status.\n");
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
void SEN0545_system_status() {
    uint8_t command[5] = {0x3A, 0x02, 0x00, 0x00, 0xC7};  // Query system status

    // Send command to sensor
    int bytes_written = uart_write_bytes(UART_NUM, command, sizeof(command));
    if (bytes_written != sizeof(command)) {
        printf("Failed to send command to sensor.\n");
        return;
    }

    // Read response
    uint8_t response[5];
    int len = uart_read_bytes(UART_NUM, response, sizeof(response), 100 / portTICK_PERIOD_MS);

    if (len > 0) {
        printf("Received %d bytes: ", len);
        for (int i = 0; i < len; i++) {
            printf("0x%02X ", response[i]);
        }
        printf("\n");
    } else {
        printf("No response\n");
    }

    // Interpret system status
    if (len == 5 && response[0] == 0x3A) {
        switch (response[2]) {
            case 0x00: printf("System normal.\n"); break;
            case 0x01: printf("SPI communication error.\n"); break;
            case 0x02: printf("LED A damaged.\n"); break;
            case 0x03: printf("LED B damaged.\n"); break;
            case 0x04: printf("Optical system calibration error.\n"); break;
            default:   printf("Unknown system code: 0x%02X\n", response[2]);
        }
    } else {
        printf("Failed to read system state.\n");
    }
}
