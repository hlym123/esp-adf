#include "esp_log.h"
#include "board.h"
#include "audio_mem.h"
#include "audio_error.h"
#include "periph_button.h"
#include "board_pins_config.h"

static const char *TAG = "AUDIO_BOARD";

static audio_board_handle_t board_handle = NULL;

audio_board_handle_t audio_board_init(void)
{
    if (board_handle) {
        ESP_LOGW(TAG, "The board has already been initialized!");
        return board_handle;
    }
    audio_hal_codec_config_t audio_codec_cfg = AUDIO_CODEC_DEFAULT_CONFIG();
    board_handle = (audio_board_handle_t) audio_calloc(1, sizeof(struct audio_board_handle));
    AUDIO_MEM_CHECK(TAG, board_handle, return NULL);

    // Use ES8311 codec for echo_pyramid board
    extern audio_hal_func_t AUDIO_CODEC_ES8311_DEFAULT_HANDLE;
    board_handle->audio_hal = audio_hal_init(&audio_codec_cfg, &AUDIO_CODEC_ES8311_DEFAULT_HANDLE);
    return board_handle;
}

esp_err_t audio_board_sdcard_init(esp_periph_set_handle_t set, periph_sdcard_mode_t mode)
{
    return ESP_FAIL;
}

display_service_handle_t audio_board_led_init(void)
{
    return NULL;
}

esp_err_t audio_board_key_init(esp_periph_set_handle_t set)
{
    periph_button_cfg_t btn_cfg = {
        .gpio_mask = (1ULL << get_input_play_id()),
    };
    esp_periph_handle_t button_handle = periph_button_init(&btn_cfg);
    AUDIO_NULL_CHECK(TAG, button_handle, return ESP_ERR_ADF_MEMORY_LACK);
    return esp_periph_start(set, button_handle);
}

audio_board_handle_t audio_board_get_handle(void)
{
    return board_handle;
}

esp_err_t audio_board_deinit(audio_board_handle_t audio_board)
{
    esp_err_t ret = ESP_OK;
    ret = audio_hal_deinit(audio_board->audio_hal);
    audio_board->audio_hal = NULL;
    audio_free(audio_board);
    board_handle = NULL;
    return ret;
}


