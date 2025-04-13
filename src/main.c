#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <driver/adc.h>
#include <driver/i2c.h>

#include "SEN0545.h"
#include "AHT20.h"
#include "L298N.h"

#define SERIAL UART_NUM_0

// Global Task Handles for the print tasks
TaskHandle_t incTask1Handle = NULL;
TaskHandle_t incTask2Handle = NULL;
TaskHandle_t serial_sendHandle = NULL;
TaskHandle_t anemoHandle = NULL;
TaskHandle_t AHT20Handle = NULL;
TaskHandle_t SEN0545Handle = NULL;

// Variables to increment
static int var1 = 0, var2 = 0;
static int anemo_raw = 0;
static uint8_t serial_in[256];  
static char serial_out[256] = "idle\n"; 

void serial_init(){
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    // Configure UART parameters
    esp_err_t err0 = uart_param_config(SERIAL, &uart_config);
    if (err0 == ESP_OK) {
        printf("UART0 configured successfully for serial\n");
    } else {
        printf("Failed to configure UART0 for serial\n");
    }

    // Install UART driver
    err0 = uart_driver_install(SERIAL, 1024, 1024, 0, NULL, 0);
    if (err0 == ESP_OK) {
        printf("UART0 driver installed successfully for serial\n");
    } else {
        printf("Failed to install UART0 driver for serial\n");
    }
}

void serial_receive_task(void *param){
    while(1){
        int len = uart_read_bytes(SERIAL, param, 256, 100 / portTICK_PERIOD_MS);
        if (len > 1){
            serial_in[len] = '\0';  // Null-terminate received data
            if (strcmp((char *)serial_in, "takeoff_signal") == 0) {
                vTaskResume(AHT20Handle);
                vTaskResume(SEN0545Handle);
                printf("Weather check activated!\n");
                snprintf(serial_out, sizeof(serial_out), "%s", "WEATHER CHECK\n");
            }
            if (strcmp((char *)serial_in, "takeoff_cancel") == 0) {
                esp_restart();
                printf("Takeoff cancelled, system restart\n");
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);  
    }
}

void serial_send_task(void *param){
    while(1){
        uart_write_bytes(SERIAL, serial_out, strlen(serial_out));
        vTaskDelay(2000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
}

// Task for incrementing a variable
void increment_task1(void *param) {
    vTaskSuspend(incTask1Handle);
    int *variable = (int *)param;  // Pointer to the variable to increment
    while(1){
        (*variable)++;  // Increment the variable
        printf("var1=%d\n", *variable);
        if (*variable == 30){
            esp_restart();
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
}

void increment_task2(void *param) {
    int *variable = (int *)param;  // Pointer to the variable to increment
    while(1){
        (*variable)++;  // Increment the variable
        printf("var2=%d\n", *variable);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
}

void anemometer_task(void *param) {
    vTaskSuspend(anemoHandle);
    while(1){
        anemo_raw = adc1_get_raw(ADC1_CHANNEL_0);
        printf("ADC Raw Value: %d\n", anemo_raw);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void SEN0545_task(void *param){
    vTaskSuspend(SEN0545Handle);
    while(1){
        if (SEN0545_rainfall_status() != 0){
            snprintf(serial_out, sizeof(serial_out), "%s", "rain detected, cancel takeoff\n");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            esp_restart();
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void AHT20_task(void *param){
    vTaskSuspend(AHT20Handle);
    while(1){
        AHT20_read_sensor();
        if (AHT20_get_temperature() <= 0 || AHT20_get_temperature() >= 50){
            snprintf(serial_out, sizeof(serial_out), "%s", "bad humidity, cancel takeoff\n");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            esp_restart();
        }
        if (AHT20_get_humidity() <= 25 || AHT20_get_humidity() >= 85){
            snprintf(serial_out, sizeof(serial_out), "%s", "bad temperature, cancel takeoff\n");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            esp_restart();
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void motor_task(void *param){
    while(1){
        motor_A_start(100, 1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        motor_A_stop();
    }
}

void app_main(void) {
    serial_init();
    L298N_init();
    SEN0545_init();
    AHT20_init();

    // Create tasks for each variable
    xTaskCreate(anemometer_task, "anemometer task", 2048, &anemo_raw, 5, &anemoHandle);
    xTaskCreate(SEN0545_task, "SEN0545 task", 2048, NULL, 5, &SEN0545Handle);
    xTaskCreate(AHT20_task, "AHT20 task", 2048, NULL, 5, &AHT20Handle);
    xTaskCreate(increment_task1, "Increment Task 1", 2048, &var1, 3, &incTask1Handle);
    xTaskCreate(increment_task2, "Increment Task 2", 2048, &var2, 3, &incTask2Handle);
    xTaskCreate(serial_receive_task, "serial receive task", 2048, &serial_in, 5, NULL);
    xTaskCreate(serial_send_task, "serial send task", 2048, NULL, 5, &serial_sendHandle);
    xTaskCreate(motor_task, "motor_task", 2048, NULL, 5, NULL);

}
