#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>
#include "board.h"
#include "board_def.h"
#include "audio_error.h"

static const char *TAG = "ECHO_PYRAMID";




esp_err_t get_i2c_pins(i2c_port_t port, i2c_config_t *i2c_config)
{
    AUDIO_NULL_CHECK(TAG, i2c_config, return ESP_FAIL);
    if (port == I2C_NUM_0 || port == I2C_NUM_1) {
        i2c_config->sda_io_num = 25; // SDA
        i2c_config->scl_io_num = 21; // SCL
    } else {
        i2c_config->sda_io_num = 25;
        i2c_config->scl_io_num = 21;
        ESP_LOGE(TAG, "i2c port %d is not supported", port);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t get_i2s_pins(int port, board_i2s_pin_t *i2s_config)
{
    AUDIO_NULL_CHECK(TAG, i2s_config, return ESP_FAIL);
    if (port == 0) {
        i2s_config->bck_io_num = 19;   // i2s_sclk
        i2s_config->ws_io_num = 33;    // i2s_lrck
        i2s_config->data_out_num = 23; // i2s_dout
        i2s_config->data_in_num = 22;  // i2s_din
        i2s_config->mck_io_num = -1;   // NC, use internal mclk
    } else {
        memset(i2s_config, -1, sizeof(board_i2s_pin_t));
        ESP_LOGE(TAG, "i2s port %d is not supported", port);
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t get_spi_pins(spi_bus_config_t *spi_config, spi_device_interface_config_t *spi_device_interface_config)
{
    AUDIO_NULL_CHECK(TAG, spi_config, return ESP_FAIL);
    AUDIO_NULL_CHECK(TAG, spi_device_interface_config, return ESP_FAIL);
    spi_config->mosi_io_num = -1;
    spi_config->miso_io_num = -1;
    spi_config->sclk_io_num = -1;
    spi_config->quadwp_io_num = -1;
    spi_config->quadhd_io_num = -1;
    spi_device_interface_config->spics_io_num = -1;
    return ESP_OK;
}

int8_t get_sdcard_intr_gpio(void) { return -1; }
int8_t get_sdcard_open_file_num_max(void) { return 0; }
int8_t get_sdcard_power_ctrl_gpio(void) { return -1; }

int8_t get_headphone_detect_gpio(void) { return -1; }
int8_t get_pa_enable_gpio(void) { return -1; }

int8_t get_input_rec_id(void) { return -1; }
int8_t get_input_mode_id(void) { return -1; }
int8_t get_input_set_id(void) { return -1; }
int8_t get_input_play_id(void) { return BUTTON_PLAY_ID; }
int8_t get_input_volup_id(void) { return -1; }
int8_t get_input_voldown_id(void) { return -1; }

int8_t get_green_led_gpio(void) { return -1; }
int8_t get_blue_led_gpio(void) { return -1; }

int8_t get_es8311_mclk_src(void)
{
    return ES8311_MCLK_SOURCE;
}

