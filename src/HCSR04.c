#include "HCSR04.h"

#include <stdio.h>
#include <driver/gpio.h>
#include <driver/timer.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define ECHO GPIO_NUM_25
#define TRIG GPIO_NUM_26

static uint64_t echo_start = 0;
static uint64_t echo_end = 0;
static volatile unsigned int echo_time = 0;
static volatile unsigned int distance_mm = 0;
static volatile int current_state = 0;

static void IRAM_ATTR ECHO_ISR(void *arg)
{
    if (gpio_get_level(ECHO)==1){
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &echo_start);
    }else{
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &echo_end);
        echo_time = echo_end - echo_start; //raw microsecond
    }
}

static void IRAM_ATTR TIMER_ISR(void *arg) {
    // Clear interrupt
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);

    // 2) Read the current counter (absolute tick count)
    uint64_t curr;

    switch (current_state) {
        case 0:
            gpio_set_level(TRIG, 1);  // start pulse
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &curr);
            timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, curr + 10); // 10µs
            current_state = 1;
            break;

        case 1:
            gpio_set_level(TRIG, 0);  // end pulse
            timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &curr);
            timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, curr + 60000); // 60ms
            current_state = 0;
            break;
    }
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

}

void HCSR04_init(void){
    gpio_config_t trig_pin = {
        .pin_bit_mask = (1ULL << TRIG),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&trig_pin);

    gpio_set_level(TRIG, 0);

    gpio_config_t echo_pin = {
        .pin_bit_mask = (1ULL << ECHO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&echo_pin);

    //setup gpio echo pin interrupt and isr
    gpio_install_isr_service(0);
    gpio_set_intr_type(ECHO, GPIO_INTR_ANYEDGE);
    gpio_isr_handler_add(ECHO, ECHO_ISR, NULL);
    gpio_intr_enable(ECHO);

    timer_config_t timer_config = {
        .divider = 80, // 80 MHz / 80 = 1 MHz -> 1 tick = 1 µs
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = false,
    };

    timer_init(TIMER_GROUP_0, TIMER_0, &timer_config);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, TIMER_ISR, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 10); // first alarm
    timer_start(TIMER_GROUP_0, TIMER_0);
}


/**
 * @function    HCSR04_GetDistance(void)
 * @brief       Returns the calculated distance in mm using the sensor model determined
 *              experimentally. 
 *              No I/O should be done in this function
 * @return      distance in mm
 */
unsigned int HCSR04_GetDistance(void){
    distance_mm = (echo_time * 340) / 2000; // conversion to mm
    return distance_mm;
}

/**
 * @function    HCSR04_GetTimeofFlight(void)
 * @brief       Returns the raw microsecond duration of the echo from the sensor.
 *              NO I/O should be done in this function.
 * @return      time of flight in uSec
 */
unsigned int HCSR04_GetTimeofFlight(void){
    echo_time = echo_end - echo_start; //raw microsecond
    return echo_time;
}
