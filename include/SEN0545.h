    #ifndef SEN0545_H
    #define	SEN0545_H

    #include <stdint.h>

    typedef enum {
        RAIN_STATUS_NO_RAIN       = 0,
        RAIN_STATUS_LIGHT_RAIN    = 1,
        RAIN_STATUS_MODERATE_RAIN = 2,
        RAIN_STATUS_HEAVY_RAIN    = 3,
        RAIN_STATUS_ERROR         = 255
    } rain_status_t;

    typedef enum {
        SYSTEM_STATUS_NORMAL          = 0,
        SYSTEM_STATUS_SPI_ERROR       = 1,
        SYSTEM_STATUS_LED_A_FAILURE   = 2,
        SYSTEM_STATUS_LED_B_FAILURE   = 3,
        SYSTEM_STATUS_CALIBRATION_ERR = 4,
        SYSTEM_STATUS_COMMS_ERROR     = 255
    } system_status_t;

    void SEN0545_init(void);
    rain_status_t SEN0545_rainfall_status(void);
    system_status_t SEN0545_system_status(void);

    #endif //SEN0545_H