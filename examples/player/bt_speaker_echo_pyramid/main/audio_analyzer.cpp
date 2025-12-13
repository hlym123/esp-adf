#include "audio_analyzer.h"
#include "esp_log.h"
#include "audio_mem.h"
#include <string.h>

static const char *TAG = "AUDIO_ANALYZER";

typedef struct {
    EchoPyramid* echo_pyramid;
    int sample_rate;
    int channels;
    int bits_per_sample;
    int bytes_per_sample;
} audio_analyzer_t;

static esp_err_t _audio_analyzer_open(audio_element_handle_t el)
{
    ESP_LOGI(TAG, "audio_analyzer opened");
    return ESP_OK;
}

static esp_err_t _audio_analyzer_close(audio_element_handle_t el)
{
    ESP_LOGI(TAG, "audio_analyzer closed");
    return ESP_OK;
}

static audio_element_err_t _audio_analyzer_process(audio_element_handle_t el, char *el_buffer, int el_buf_len)
{
    audio_analyzer_t *analyzer = (audio_analyzer_t *)audio_element_getdata(el);
    
    // 从输入读取数据
    audio_element_err_t bytes_read = audio_element_input(el, el_buffer, el_buf_len);
    if (bytes_read <= 0) {
        return bytes_read;
    }
    
    if (analyzer != NULL && analyzer->echo_pyramid != NULL) {
        // 获取音频信息
        audio_element_info_t music_info = {0};
        audio_element_getinfo(el, &music_info);
        if (music_info.sample_rates > 0) {
            analyzer->sample_rate = music_info.sample_rates;
            analyzer->channels = music_info.channels;
            analyzer->bits_per_sample = music_info.bits;
            analyzer->bytes_per_sample = analyzer->bits_per_sample / 8;
        }
        
        // 只处理16位PCM数据
        if (analyzer->bytes_per_sample == 2 && analyzer->channels > 0 && bytes_read >= analyzer->bytes_per_sample) {
            // 将字节数据转换为 int16_t 样本
            int num_samples = bytes_read / analyzer->bytes_per_sample;
            int16_t *samples = (int16_t *)el_buffer;
            
            // 传递给 EchoPyramid 进行音频分析（用于音乐随动灯效）
            // 注意：这里需要确保 EchoPyramid 的 processAudioData 是线程安全的
            analyzer->echo_pyramid->processAudioData(samples, num_samples, analyzer->channels);
        }
    }
    
    // 透传数据到下一个 element（不修改音频流）
    return audio_element_output(el, el_buffer, bytes_read);
}

static esp_err_t _audio_analyzer_destroy(audio_element_handle_t el)
{
    audio_analyzer_t *analyzer = (audio_analyzer_t *)audio_element_getdata(el);
    if (analyzer) {
        audio_free(analyzer);
    }
    ESP_LOGI(TAG, "audio_analyzer destroyed");
    return ESP_OK;
}

audio_element_handle_t audio_analyzer_init(audio_analyzer_cfg_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "config is NULL");
        return NULL;
    }
    
    audio_analyzer_t *analyzer = (audio_analyzer_t *)audio_calloc(1, sizeof(audio_analyzer_t));
    if (analyzer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for audio_analyzer");
        return NULL;
    }
    
    analyzer->echo_pyramid = config->echo_pyramid;
    analyzer->sample_rate = 44100;  // 默认值
    analyzer->channels = 2;          // 默认值
    analyzer->bits_per_sample = 16;  // 默认值
    analyzer->bytes_per_sample = 2;  // 默认值
    
    audio_element_cfg_t cfg = DEFAULT_AUDIO_ELEMENT_CONFIG();
    cfg.open = _audio_analyzer_open;
    cfg.close = _audio_analyzer_close;
    cfg.process = _audio_analyzer_process;
    cfg.destroy = _audio_analyzer_destroy;
    cfg.tag = "analyzer";
    cfg.task_stack = 4096;
    cfg.task_prio = 5;
    cfg.task_core = 0;
    
    audio_element_handle_t el = audio_element_init(&cfg);
    if (el == NULL) {
        ESP_LOGE(TAG, "Failed to initialize audio element");
        audio_free(analyzer);
        return NULL;
    }
    
    audio_element_setdata(el, analyzer);
    ESP_LOGI(TAG, "audio_analyzer initialized");
    
    return el;
}
