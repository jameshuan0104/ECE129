#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/timers.h>

#include <driver/gpio.h>
#include <driver/uart.h>

#include "SEN0545.h"
#include "AHT20.h"
#include "ANEMOMETER.h"
#include "L298N.h"
#include "HCSR04.h"

#define SERIAL UART_NUM_0

// Global Task Handles for the print tasks
TaskHandle_t incTask1Handle = NULL;
TaskHandle_t incTask2Handle = NULL;

TaskHandle_t ANEMOHandle = NULL;
TaskHandle_t AHT20Handle = NULL;
TaskHandle_t SEN0545Handle = NULL;
TaskHandle_t HCSR04Handle = NULL;


static TimerHandle_t min_timer;
static QueueHandle_t uart_queue;

// Variables to increment
static int var1 = 0, var2 = 0;
static char serial_out[256] = "IDLE\n"; 

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

    uart_queue = xQueueCreate(10, sizeof(uart_event_t));
    if (uart_queue == NULL) {
        // Handle error: queue creation failed
        printf("Failed to create UART queue\n");
    }

    // Install UART driver
    err0 = uart_driver_install(SERIAL, 1024, 1024, 10, &uart_queue, 0);
    if (err0 == ESP_OK) {
        printf("UART0 driver installed successfully for serial\n");
    } else {
        printf("Failed to install UART0 driver for serial\n");
    }
}

void serial_receive_task(void *param){
    QueueHandle_t uart_queue = (QueueHandle_t)param;  // cast param back to queue handle
    uart_event_t event;
    uint8_t data[256];

    while(1){
        // Wait for UART event
        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(SERIAL, data, event.size, portMAX_DELAY);
                data[len] = '\0';  // Null-terminate received data
                if (strcmp((char *)data, "takeoff_signal") == 0) {
                    vTaskResume(AHT20Handle);
                    vTaskResume(SEN0545Handle);
                    vTaskResume(ANEMOHandle);
                    snprintf(serial_out, sizeof(serial_out), "%s", "WEATHER_CHECK\n");
                    xTimerStart(min_timer, 0);
                }
                if (strcmp((char *)data, "takeoff_cancel") == 0) {
                    esp_restart();
                }
                
            }
        }
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
        //printf("var2=%d\n", *variable);
        vTaskDelay(1000 / portTICK_PERIOD_MS);  // Delay for 1 second
    }
}

void ANEMOMETER_task(void *param) {
    vTaskSuspend(ANEMOHandle);
    while(1){
        uint16_t anemo_raw = ANEMOMETER_read_adc();
        float kmh = ANEMOMETER_read_kmh();
        if (kmh > 12){
            snprintf(serial_out, sizeof(serial_out), "%s", "high wind detected, cancel takeoff\n");
            vTaskDelay(5000 / portTICK_PERIOD_MS);
            esp_restart(); 
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
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
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void HCSR04_task(void *param){
    //vTaskSuspend(HCSR04Handle);
    while(1){
        printf("ultrasonic %d\n", HCSR04_GetDistance());
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void motor_task(void *param){
    while(1){
        motor_A_start(100, 1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        motor_A_stop();
    }
}

void min_timer_callback(TimerHandle_t xTimer){
    snprintf(serial_out, sizeof(serial_out), "%s", "OPENING_DOCK\n");
    vTaskSuspend(AHT20Handle);
    vTaskSuspend(ANEMOHandle);
    vTaskSuspend(SEN0545Handle);
    //resume motor task -> open dock doors 
}

void app_main(void) {
    serial_init();
    L298N_init();
    SEN0545_init();
    AHT20_init();
    HCSR04_init();
    ANEMOMETER_init();

    min_timer = xTimerCreate(
        "MinuteTimer",          // Name
        (pdMS_TO_TICKS(60000)),    // 1 min period
        pdFALSE,                 // Auto-reload
        (void*)0,               // Timer ID
        min_timer_callback   // Callback function
    );

    if (min_timer == NULL){
        printf("timer not create\n");
        esp_restart();
    }else{
        printf("timer created\n");
    }

    // Create tasks for each variable
    xTaskCreate(ANEMOMETER_task, "ANEMOMETER task", 2048, NULL, 4, &ANEMOHandle);
    xTaskCreate(SEN0545_task, "SEN0545 task", 2048, NULL, 4, &SEN0545Handle);
    xTaskCreate(AHT20_task, "AHT20 task", 2048, NULL, 4, &AHT20Handle);
    xTaskCreate(HCSR04_task, "HCSR04 task", 20248, NULL, 4, &HCSR04Handle);

    xTaskCreate(increment_task1, "Increment Task 1", 2048, &var1, 3, &incTask1Handle);
    xTaskCreate(increment_task2, "Increment Task 2", 2048, &var2, 3, &incTask2Handle);

    xTaskCreate(serial_receive_task, "serial receive task", 2048, (void *)uart_queue, 5, NULL);
    xTaskCreate(serial_send_task, "serial send task", 2048, NULL, 5, NULL);

    xTaskCreate(motor_task, "motor_task", 2048, NULL, 4, NULL);
}
