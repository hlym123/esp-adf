/* Play music from Bluetooth device

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_peripherals.h"

#include "nvs_flash.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "i2s_stream.h"
#include "input_key_service.h"
#include "filter_resample.h"
#include "periph_touch.h"
#include "board.h"
#include "a2dp_stream.h"
#include "i2c_bus.h"
#include "bsp/echo_pyramid.h"
#include "bsp/led_matrix.h"
#include "app/audio_analyzer.h"

static const char *TAG = "main";

static esp_periph_handle_t bt_periph = NULL;
static EchoPyramid* echo_pyramid = nullptr;
static Si5351* si5351 = nullptr;
static Aw87559* aw87559 = nullptr;
static void* led_matrix_handle = nullptr;  // 5x5 LED矩阵句柄

#define CONFIG_ESP_LYRATD_MSC_V2_1_BOARD 1 


static void touch_event_callback(TouchEvent event) {
    if (bt_periph == NULL) {
        return;
    }

    switch (event) {
        case TouchEvent::LEFT_SLIDE_UP:
            // 左侧向上滑动：下一首
            ESP_LOGI(TAG, "[Touch] Left slide up: Next track");
            periph_bt_avrc_next(bt_periph);
            // 显示右箭头
            if (led_matrix_handle != nullptr) {
                led_matrix_trigger_effect(led_matrix_handle, LED_MATRIX_EFFECT_RIGHT_ARROW);
            }
            break;
        case TouchEvent::LEFT_SLIDE_DOWN:
            // 左侧向下滑动：上一首
            ESP_LOGI(TAG, "[Touch] Left slide down: Previous track");
            periph_bt_avrc_prev(bt_periph);
            // 显示左箭头
            if (led_matrix_handle != nullptr) {
                led_matrix_trigger_effect(led_matrix_handle, LED_MATRIX_EFFECT_LEFT_ARROW);
            }
            break;
        case TouchEvent::RIGHT_SLIDE_UP:
            // 右侧向上滑动：音量减小
            ESP_LOGI(TAG, "[Touch] Right slide up: Volume down");
            periph_bt_volume_down(bt_periph);
            // 显示下箭头
            if (led_matrix_handle != nullptr) {
                led_matrix_trigger_effect(led_matrix_handle, LED_MATRIX_EFFECT_DOWN_ARROW);
            }
            break;
        case TouchEvent::RIGHT_SLIDE_DOWN:
            // 右侧向下滑动：音量增大
            ESP_LOGI(TAG, "[Touch] Right slide down: Volume up");
            periph_bt_volume_up(bt_periph);
            // 显示上箭头
            if (led_matrix_handle != nullptr) {
                led_matrix_trigger_effect(led_matrix_handle, LED_MATRIX_EFFECT_UP_ARROW);
            }
            break;
    }
}

// 获取下一个灯效模式
static LightMode GetNextLightMode(LightMode current_mode) {
    int mode = static_cast<int>(current_mode);
    int next_mode = (mode + 1) % 9; // 9种模式循环: OFF, STATIC, BREATHE, RAINBOW, CHASE, MUSIC_REACTIVE, MUSIC_REACTIVE_RED, MUSIC_REACTIVE_GREEN, MUSIC_REACTIVE_BLUE
    return static_cast<LightMode>(next_mode);
}

static esp_err_t input_key_service_cb(periph_service_handle_t handle, periph_service_event_t *evt, void *ctx)
{
    if (evt->type == INPUT_KEY_SERVICE_ACTION_CLICK_RELEASE) {
        ESP_LOGI(TAG, "[ * ] input key id is %d", (int)evt->data);
        switch ((int)evt->data) {
            case INPUT_KEY_USER_ID_PLAY:
                // 切换灯效
                if (echo_pyramid != nullptr) {
                    LightMode current_mode = echo_pyramid->getLightMode();
                    LightMode new_mode = GetNextLightMode(current_mode);
                    echo_pyramid->setLightMode(new_mode);
                    
                    // 在5x5 LED矩阵上显示水滴效果（中间向四周发散）
                    if (led_matrix_handle != nullptr) {
                        led_matrix_trigger_effect(led_matrix_handle, LED_MATRIX_EFFECT_RIPPLE);
                    }
                    
                    const char* mode_names[] = {"OFF", "MUSIC_REACTIVE", "MUSIC_REACTIVE_RED", "MUSIC_REACTIVE_GREEN", "MUSIC_REACTIVE_BLUE",
                                                "RAINBOW", "BREATHE", "CHASE", "STATIC"};
                    int mode_index = static_cast<int>(new_mode);
                    if (mode_index >= 0 && mode_index < 9) {
                        ESP_LOGI(TAG, "[ * ] [Button] Switch light mode to: %s", mode_names[mode_index]);
                    } else {
                        ESP_LOGI(TAG, "[ * ] [Button] Switch light mode to: mode %d", mode_index);
                    }
                }
                break;
            case INPUT_KEY_USER_ID_SET:
                ESP_LOGI(TAG, "[ * ] [Set] pause");
                periph_bt_pause(bt_periph);
                break;
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0))
            case INPUT_KEY_USER_ID_VOLUP:
                ESP_LOGI(TAG, "[ * ] [long Vol+] Vol+");
                periph_bt_volume_up(bt_periph);
                // 显示上箭头
                if (led_matrix_handle != nullptr) {
                    led_matrix_trigger_effect(led_matrix_handle, LED_MATRIX_EFFECT_UP_ARROW);
                }
                break;
            case INPUT_KEY_USER_ID_VOLDOWN:
                ESP_LOGI(TAG, "[ * ] [long Vol-] Vol-");
                periph_bt_volume_down(bt_periph);
                // 显示下箭头
                if (led_matrix_handle != nullptr) {
                    led_matrix_trigger_effect(led_matrix_handle, LED_MATRIX_EFFECT_DOWN_ARROW);
                }
                break;
#endif
        }
    } else if (evt->type == INPUT_KEY_SERVICE_ACTION_PRESS) {
        ESP_LOGI(TAG, "[ * ] input key id is %d", (int)evt->data);
        switch ((int)evt->data) {
            case INPUT_KEY_USER_ID_VOLUP:
                ESP_LOGI(TAG, "[ * ] [long Vol+] next");
                periph_bt_avrc_next(bt_periph);
                // 显示右箭头
                if (led_matrix_handle != nullptr) {
                    led_matrix_trigger_effect(led_matrix_handle, LED_MATRIX_EFFECT_RIGHT_ARROW);
                }
                break;
            case INPUT_KEY_USER_ID_VOLDOWN:
                ESP_LOGI(TAG, "[ * ] [long Vol-] Previous");
                periph_bt_avrc_prev(bt_periph);
                // 显示左箭头
                if (led_matrix_handle != nullptr) {
                    led_matrix_trigger_effect(led_matrix_handle, LED_MATRIX_EFFECT_LEFT_ARROW);
                }
                break;
        }

    }
    return ESP_OK;
}

extern "C" void app_main(void)
{
    audio_pipeline_handle_t pipeline;
    audio_element_handle_t bt_stream_reader, i2s_stream_writer, audio_analyzer_el;

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "[ 1 ] Init Bluetooth");
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    esp_bt_dev_set_device_name("EchoPyramid");

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0))
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
#else
    esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
#endif

    ESP_LOGI(TAG, "[ 2 ] Start codec chip");
    audio_board_handle_t board_handle = audio_board_init();

    ESP_LOGI(TAG, "[ 2.1 ] Init I2C bus");
    i2c_master_bus_handle_t i2c_bus = i2c_bus_get_master_handle(I2C_NUM_0);

    ESP_LOGI(TAG, "[ 2.2 ] Init Echo Pyramid");
    echo_pyramid = new EchoPyramid(i2c_bus, ECHO_PYRAMID_DEVICE_ADDR);
    
    ESP_LOGI(TAG, "[ 2.2.1 ] Init 5x5 LED Matrix (GPIO 27)");
    led_matrix_handle = led_matrix_init(27);
    if (led_matrix_handle == nullptr) {
        ESP_LOGW(TAG, "Failed to initialize LED matrix");
    }
    
    ESP_LOGI(TAG, "[ 2.3 ] Init Si5351");
    si5351 = new Si5351(i2c_bus); // before init codec chip    
    aw87559 = new Aw87559(i2c_bus);
    echo_pyramid->setLightMode(LightMode::MUSIC_REACTIVE);
    
    ESP_LOGI(TAG, "[ 2.4 ] Setup touch callbacks");
    echo_pyramid->addTouchEventCallback(touch_event_callback);
    echo_pyramid->startTouchDetection();

    ESP_LOGI(TAG, "[ 2.5 ] Start codec chip");
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

    ESP_LOGI(TAG, "[ 3 ] Create audio pipeline for playback");
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();
    pipeline = audio_pipeline_init(&pipeline_cfg);

    ESP_LOGI(TAG, "[4] Create i2s stream to write data to codec chip");
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = AUDIO_STREAM_WRITER;
    i2s_stream_writer = i2s_stream_init(&i2s_cfg);

    ESP_LOGI(TAG, "[4.1] Get Bluetooth stream");
    a2dp_stream_config_t a2dp_config = {
        .type = AUDIO_STREAM_READER,
        .user_callback = {0},
#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 0, 0))
        .audio_hal = board_handle->audio_hal,
#endif
    };
    bt_stream_reader = a2dp_stream_init(&a2dp_config);

    ESP_LOGI(TAG, "[4.1.1] Create audio analyzer for music reactive lighting");
    audio_analyzer_cfg_t analyzer_cfg = {
        .echo_pyramid = echo_pyramid,
    };
    audio_analyzer_el = audio_analyzer_init(&analyzer_cfg);
    if (audio_analyzer_el == NULL) {
        ESP_LOGE(TAG, "Failed to initialize audio analyzer");
        return;
    }

    ESP_LOGI(TAG, "[4.2] Register all elements to audio pipeline");
    audio_pipeline_register(pipeline, bt_stream_reader, "bt");
    audio_pipeline_register(pipeline, audio_analyzer_el, "analyzer");
    audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

    ESP_LOGI(TAG, "[4.3] Link it together [Bluetooth]-->bt_stream_reader-->[analyzer]-->[filter]-->i2s_stream_writer-->[codec_chip]");
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
    rsp_filter_cfg_t rsp_cfg = DEFAULT_RESAMPLE_FILTER_CONFIG();
    rsp_cfg.src_rate = 44100;
    rsp_cfg.src_ch = 2;
    rsp_cfg.dest_rate = 48000;
    rsp_cfg.dest_ch = 2;
    rsp_cfg.task_prio = 19;
    audio_element_handle_t filter = rsp_filter_init(&rsp_cfg);
    audio_pipeline_register(pipeline, filter, "filter");
    i2s_stream_set_clk(i2s_stream_writer, 48000, 16, 2);
    const char *link_tag[4] = {"bt", "analyzer", "filter", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 4);
#else
    const char *link_tag[3] = {"bt", "analyzer", "i2s"};
    audio_pipeline_link(pipeline, &link_tag[0], 3);
#endif

    ESP_LOGI(TAG, "[ 5 ] Initialize peripherals");
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    esp_periph_set_handle_t set = esp_periph_set_init(&periph_cfg);
    audio_board_key_init(set);

    ESP_LOGI(TAG, "[ 5.1 ] Create and start input key service");
    periph_service_handle_t input_ser = NULL;
#if (INPUT_KEY_NUM > 0)
    input_key_service_info_t input_key_info[] = INPUT_KEY_DEFAULT_INFO();
    input_key_service_cfg_t input_cfg = INPUT_KEY_SERVICE_DEFAULT_CONFIG();
    input_cfg.handle = set;
    input_ser = input_key_service_create(&input_cfg);
    input_key_service_add_key(input_ser, input_key_info, INPUT_KEY_NUM);
    periph_service_set_callback(input_ser, input_key_service_cb, NULL);
#else
    ESP_LOGI(TAG, "[ 5.1 ] No physical buttons, using touch control only");
#endif

    ESP_LOGI(TAG, "[5.2] Create Bluetooth peripheral");
    bt_periph = bt_create_periph();

    ESP_LOGI(TAG, "[5.3] Start all peripherals");
    esp_periph_start(set, bt_periph);

    ESP_LOGI(TAG, "[ 6 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);

    ESP_LOGI(TAG, "[6.1] Listening event from all elements of pipeline");
    audio_pipeline_set_listener(pipeline, evt);

    ESP_LOGI(TAG, "[ 7 ] Start audio_pipeline");
    audio_pipeline_run(pipeline);

    ESP_LOGI(TAG, "[ 8 ] Listen for all pipeline events");
    while (1) {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }

        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) bt_stream_reader
            && msg.cmd == AEL_MSG_CMD_REPORT_MUSIC_INFO) {
            audio_element_info_t music_info = {0};
            audio_element_getinfo(bt_stream_reader, &music_info);

            ESP_LOGI(TAG, "[ * ] Receive music info from Bluetooth, sample_rates=%d, bits=%d, ch=%d",
                     music_info.sample_rates, music_info.bits, music_info.channels);

            audio_element_set_music_info(i2s_stream_writer, music_info.sample_rates, music_info.channels, music_info.bits);
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
#else
            i2s_stream_set_clk(i2s_stream_writer, music_info.sample_rates, music_info.bits, music_info.channels);
#endif
            continue;
        }

        /* Stop when the last pipeline element (i2s_stream_writer in this case) receives stop event */
        if (msg.source_type == AUDIO_ELEMENT_TYPE_ELEMENT && msg.source == (void *) i2s_stream_writer
            && msg.cmd == AEL_MSG_CMD_REPORT_STATUS
            && (((int)msg.data == AEL_STATUS_STATE_STOPPED) || ((int)msg.data == AEL_STATUS_STATE_FINISHED))) {
            ESP_LOGW(TAG, "[ * ] Stop event received");
            break;
        }
    }

    ESP_LOGI(TAG, "[ 9 ] Stop audio_pipeline");
    audio_pipeline_stop(pipeline);
    audio_pipeline_wait_for_stop(pipeline);
    audio_pipeline_terminate(pipeline);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline);

    /* Stop all periph before removing the listener */
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_unregister(pipeline, bt_stream_reader);
    audio_pipeline_unregister(pipeline, audio_analyzer_el);
    audio_pipeline_unregister(pipeline, i2s_stream_writer);
#if (CONFIG_ESP_LYRATD_MSC_V2_1_BOARD || CONFIG_ESP_LYRATD_MSC_V2_2_BOARD)
    audio_pipeline_unregister(pipeline, filter);
    audio_element_deinit(filter);
#endif
    audio_pipeline_deinit(pipeline);
    audio_element_deinit(bt_stream_reader);
    audio_element_deinit(audio_analyzer_el);
    audio_element_deinit(i2s_stream_writer);
#if (INPUT_KEY_NUM > 0)
    periph_service_destroy(input_ser);
#endif
    esp_periph_set_destroy(set);
    
    // 清理5x5 LED矩阵
    if (led_matrix_handle != nullptr) {
        led_matrix_deinit(led_matrix_handle);
        led_matrix_handle = nullptr;
    }
    
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
}
