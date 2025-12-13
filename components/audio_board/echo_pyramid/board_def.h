#ifndef _AUDIO_BOARD_DEFINITION_H_
#define _AUDIO_BOARD_DEFINITION_H_

#include "driver/gpio.h"

// SD card disabled
#define FUNC_SDCARD_EN            (0)
#define SDCARD_OPEN_FILE_NUM_MAX  0
#define SDCARD_INTR_GPIO          -1

// LED disabled
#define FUNC_SYS_LEN_EN           (0)
#define GREEN_LED_GPIO            -1

// Codec: assume internal I2S only sink (no external codec control)
#define FUNC_AUDIO_CODEC_EN       (1)
#define PA_ENABLE_GPIO            -1
#define CODEC_ADC_I2S_PORT        (0)
#define CODEC_ADC_BITS_PER_SAMPLE (16)
#define CODEC_ADC_SAMPLE_RATE     (44100)
#define RECORD_HARDWARE_AEC       (false)
#define BOARD_PA_GAIN             (10)

extern audio_hal_func_t AUDIO_CODEC_ES8311_DEFAULT_HANDLE;

// Use I2S in standard format; device as slave or master is configured in app
#define AUDIO_CODEC_DEFAULT_CONFIG(){                   \
        .adc_input  = AUDIO_HAL_ADC_INPUT_LINE1,        \
        .dac_output = AUDIO_HAL_DAC_OUTPUT_ALL,         \
        .codec_mode = AUDIO_HAL_CODEC_MODE_DECODE,      \
        .i2s_iface = {                                  \
            .mode = AUDIO_HAL_MODE_SLAVE,               \
            .fmt = AUDIO_HAL_I2S_NORMAL,                \
            .samples = AUDIO_HAL_44K_SAMPLES,           \
            .bits = AUDIO_HAL_BIT_LENGTH_16BITS,        \
        },                                              \
};

// buttons
#define FUNC_BUTTON_EN            (1)
#define INPUT_KEY_NUM             (1)
#define BUTTON_PLAY_ID            GPIO_NUM_39

#define INPUT_KEY_DEFAULT_INFO() {                  \
    {                                               \
        .type = PERIPH_ID_BUTTON,                   \
        .user_id = INPUT_KEY_USER_ID_PLAY,          \
        .act_id = BUTTON_PLAY_ID,                   \
    }                                               \
}

/**
 * @brief SDCARD Function Definition
 */
 #define FUNC_SDCARD_EN             (0)
 #define SDCARD_OPEN_FILE_NUM_MAX    5
 #define SDCARD_INTR_GPIO            -1
 #define SDCARD_PWR_CTRL             -1
 
 #define ESP_SD_PIN_CLK              -1
 #define ESP_SD_PIN_CMD              -1
 #define ESP_SD_PIN_D0               -1
 #define ESP_SD_PIN_D3               -1
 
// ES8311 MCLK source: 0 = From MCLK pin, 1 = From SCLK (BCLK)
// echo_pyramid has no external MCLK (mck_io_num = -1), so use SCLK
// Use Si5351 Clock Generator
#define ES8311_MCLK_SOURCE          (0)  /* 0 From MCLK of esp32 1 From BCLK */


#endif

