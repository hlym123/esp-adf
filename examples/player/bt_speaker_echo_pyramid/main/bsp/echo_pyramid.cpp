#include "echo_pyramid.h"
#include "i2c_device.h"
#include <esp_log.h>
#include <math.h>

#define TAG "EchoPyramid"

// STM32 寄存器地址
#define REG_TOUCH1_STATUS       (0x00)
#define REG_TOUCH2_STATUS       (0x01)
#define REG_TOUCH3_STATUS       (0x02)
#define REG_TOUCH4_STATUS       (0x03)

#define REG_RGB1_BRIGHTNESS     (0x10)
#define REG_RGB2_BRIGHTNESS     (0x11)
#define REG_RGB_CH1_I1_COLOR    (0x20)
#define REG_RGB_CH2_I1_COLOR    (0x3C)
#define REG_RGB_CH3_I1_COLOR    (0x60)
#define REG_RGB_CH4_I1_COLOR    (0x7C)

#define NUM_RGB_STRIPS          (2)
#define NUM_LEDS_PER_STRIP      (14)
#define NUM_GROUPS_PER_STRIP    (2)
#define NUM_LEDS_PER_GROUP      (7)
#define NUM_RGB_CHANNELS        (4)

/**
 * STM32 RGB灯带控制器实现
 */
class EchoPyramid::Stm32Impl : public I2cDevice {
public:
    Stm32Impl(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
        uint8_t firmware_version = ReadReg(0xFE);
        ESP_LOGI(TAG, "Init Stm32 firmware version: 0x%02X", firmware_version);

        ESP_LOGI(TAG, "AW87559 reset");
        WriteReg(0xA0, 0x01);
        vTaskDelay(pdMS_TO_TICKS(100));
        WriteReg(REG_RGB1_BRIGHTNESS, 50);
        WriteReg(REG_RGB2_BRIGHTNESS, 50);
        clearAllRgb();
        setEffectMode(LightMode::BREATHE);
        startEffectTask();
    }

    ~Stm32Impl() {
        stopEffectTask();
    }

    void setRgbColor(uint8_t channel, uint8_t index, uint32_t color) {
        if (channel >= NUM_RGB_CHANNELS || index >= NUM_LEDS_PER_GROUP) return;
        setRgbColorInternal(channel, index, color);
    }

    void setRgbChannelColor(uint8_t channel, uint32_t color) {
        if (channel >= NUM_RGB_CHANNELS) return;
        for (uint8_t i = 0; i < NUM_LEDS_PER_GROUP; i++) {
            setRgbColorInternal(channel, i, color);
        }
    }

    void setAllRgbColor(uint32_t color) {
        for (uint8_t c = 0; c < NUM_RGB_CHANNELS; c++) {
            setRgbChannelColor(c, color);
        }
    }

    void clearAllRgb() {
        setAllRgbColor(0x00000000);
    }

    void setRgbStripBrightness(uint8_t strip, uint8_t brightness) {
        if (strip >= NUM_RGB_STRIPS) return;
        if (brightness > 100) brightness = 100;
        uint8_t reg_addr = (strip == 0) ? REG_RGB1_BRIGHTNESS : REG_RGB2_BRIGHTNESS;
        WriteReg(reg_addr, brightness);
    }

    void setEffectMode(LightMode mode) {
        effect_mode_ = mode;
    }

    LightMode getEffectMode() const {
        return effect_mode_.load();
    }

    void setEffectColor(uint32_t color) {
        effect_color_ = color;
    }

    void processAudioData(const int16_t* samples, size_t num_samples, int channels) {
        LightMode mode = effect_mode_.load();
        if (mode != LightMode::MUSIC_REACTIVE && 
            mode != LightMode::MUSIC_REACTIVE_RED &&
            mode != LightMode::MUSIC_REACTIVE_GREEN &&
            mode != LightMode::MUSIC_REACTIVE_BLUE) {
            return;  // 只在音乐随动模式下分析
        }

        // 计算RMS音量和频段能量
        float sum_squares = 0.0f;
        float sum_abs = 0.0f;  // 绝对值和，用于更敏感的检测
        float bass_sum = 0.0f;
        float mid_sum = 0.0f;
        float treble_sum = 0.0f;
        
        // 简单的低通/高通滤波器状态（用于频段分离）
        static float bass_lpf = 0.0f;
        static float treble_hpf = 0.0f;
        
        for (size_t i = 0; i < num_samples; i += channels) {
            // 取左右声道的平均值
            int16_t left = samples[i];
            int16_t right = (channels > 1) ? samples[i + 1] : left;
            int16_t sample = (left + right) / 2;
            
            float normalized = (float)sample / 32768.0f;
            float abs_sample = fabsf(normalized);
            
            sum_squares += normalized * normalized;
            sum_abs += abs_sample;
            
            // 简单的频段分离：使用一阶滤波器
            // 低通滤波器（用于低频）
            bass_lpf = bass_lpf * 0.95f + abs_sample * 0.05f;
            // 高通滤波器（用于高频，需要存储前一个值）
            float diff = abs_sample - treble_hpf;
            treble_hpf = abs_sample;
            treble_sum += fabsf(diff) * 0.3f;
            
            // 低频能量（低通滤波后的值）
            bass_sum += bass_lpf * 0.6f;
            // 中频能量（原始信号减去低高频）
            mid_sum += (abs_sample - bass_lpf * 0.5f - fabsf(diff) * 0.2f) * 0.4f;
            if (mid_sum < 0) mid_sum = 0;
        }
        
        // 计算RMS和平均绝对值
        float rms = sqrtf(sum_squares / num_samples);
        float avg_abs = sum_abs / num_samples;
        
        // 使用RMS和平均绝对值的组合，提高灵敏度
        float volume_estimate = (rms * 0.7f + avg_abs * 0.3f) * 1.5f;  // 放大信号
        volume_estimate = fminf(volume_estimate, 1.0f);
        
        // 减少平滑程度，提高响应速度（让跳动更明显）
        float current_volume = audio_volume_.load();
        float new_volume = current_volume * 0.5f + volume_estimate * 0.5f;  // 进一步提高响应速度
        audio_volume_ = new_volume;
        
        // 计算频段能量，使用更灵敏的缩放
        float samples_scale = 1.0f / num_samples;
        float bass_energy = bass_sum * samples_scale * 2.5f;  // 进一步放大低频响应
        float mid_energy = mid_sum * samples_scale * 2.0f;    // 提高中频响应
        float treble_energy = treble_sum * samples_scale * 2.0f;  // 提高高频响应
        
        // 减少平滑频段能量更新（让跳动更直接）
        audio_bass_ = audio_bass_.load() * 0.6f + bass_energy * 0.4f;
        audio_mid_ = audio_mid_.load() * 0.6f + mid_energy * 0.4f;
        audio_treble_ = audio_treble_.load() * 0.6f + treble_energy * 0.4f;
        
        // 限制在合理范围
        if (audio_bass_.load() > 1.0f) audio_bass_ = 1.0f;
        if (audio_mid_.load() > 1.0f) audio_mid_ = 1.0f;
        if (audio_treble_.load() > 1.0f) audio_treble_ = 1.0f;
    }

    bool readTouchStatus(uint8_t touch_num) {
        if (touch_num < 1 || touch_num > 4) return false;
        uint8_t reg_addr;
        switch (touch_num) {
            case 1: reg_addr = REG_TOUCH1_STATUS; break;
            case 2: reg_addr = REG_TOUCH2_STATUS; break;
            case 3: reg_addr = REG_TOUCH3_STATUS; break;
            case 4: reg_addr = REG_TOUCH4_STATUS; break;
            default: return false;
        }
        uint8_t buffer[1];
        esp_err_t ret = i2c_master_transmit_receive(i2c_device_, &reg_addr, 1, buffer, 1, 300);
        if (ret != ESP_OK) return false;
        return (buffer[0] & 0x01) != 0;
    }

    void startEffectTask() {
        if (effect_task_handle_ == nullptr) {
            xTaskCreate(RgbEffectTask, "rgb_effect", 8192, this, 3, &effect_task_handle_);
            ESP_LOGI(TAG, "RGB effect task started");
        }
    }

    void stopEffectTask() {
        if (effect_task_handle_ != nullptr) {
            effect_running_ = false;
            vTaskDelay(pdMS_TO_TICKS(100));
            if (effect_task_handle_ != nullptr) {
                vTaskDelete(effect_task_handle_);
                effect_task_handle_ = nullptr;
            }
            ESP_LOGI(TAG, "RGB effect task stopped");
        }
    }

private:
    TaskHandle_t effect_task_handle_ = nullptr;
    std::atomic<bool> effect_running_{true};
    std::atomic<LightMode> effect_mode_{LightMode::MUSIC_REACTIVE}; // 默认为音乐随动模式
    uint32_t effect_color_ = 0x000000FF;
    int effect_speed_ = 10;
    
    // 音频分析相关
    std::atomic<float> audio_volume_{0.0f};  // 当前音量 (0.0 - 1.0)
    std::atomic<float> audio_bass_{0.0f};    // 低频能量 (0.0 - 1.0)
    std::atomic<float> audio_mid_{0.0f};     // 中频能量 (0.0 - 1.0)
    std::atomic<float> audio_treble_{0.0f};  // 高频能量 (0.0 - 1.0)
    int music_reactive_hue_ = 0;             // 音乐随动的色相

    void setRgbColorInternal(uint8_t channel, uint8_t index, uint32_t color) {
        uint8_t hardware_index = index;
        if (channel == 0 || channel == 1) {
            hardware_index = NUM_LEDS_PER_GROUP - 1 - index;
        }

        uint8_t reg_addr;
        switch (channel) {
            case 0: reg_addr = REG_RGB_CH1_I1_COLOR + hardware_index * 4; break;
            case 1: reg_addr = REG_RGB_CH2_I1_COLOR + hardware_index * 4; break;
            case 2: reg_addr = REG_RGB_CH4_I1_COLOR + hardware_index * 4; break;
            case 3: reg_addr = REG_RGB_CH3_I1_COLOR + hardware_index * 4; break;
            default: return;
        }
        WriteRegs(reg_addr, (uint8_t*)&color, sizeof(color));
    }

    void RenderMusicReactive(float energy, float hue, float saturation) {
        // 为所有RGB通道设置相同的灯效（使用4个通道）
        for (int c = 0; c < NUM_RGB_CHANNELS; c++) {
            // 计算应该点亮多少个LED（从顶部向下）
            // energy已经在计算时经过非线性映射，直接使用即可
            float active_led_count = energy * NUM_LEDS_PER_GROUP;
            
            // 为每个LED设置颜色和亮度（从顶部向底部）
            for (int i = 0; i < NUM_LEDS_PER_GROUP; i++) {
                float led_brightness = 0.0f;
                
                // 计算从顶部开始的LED索引（反方向）
                // i=0 对应最底部的LED，i=NUM_LEDS_PER_GROUP-1 对应最顶部的LED
                int led_from_top = NUM_LEDS_PER_GROUP - 1 - i;  // 0是最顶部，NUM_LEDS_PER_GROUP-1是最底部
                
                if (led_from_top < active_led_count) {
                    // 这个LED应该被点亮
                    // 计算LED在点亮区域中的位置（0.0到1.0，0.0是顶部）
                    float led_pos_in_active = (active_led_count > 0.1f) ? 
                        ((float)led_from_top / active_led_count) : 0.0f;
                    led_pos_in_active = fminf(led_pos_in_active, 1.0f);
                    
                    // 亮度渐变：从顶部LED到底部LED，亮度逐渐增加
                    // 顶部的LED较暗，底部的LED较亮 - 增加对比度让跳动更明显
                    float brightness_factor = 0.5f + (led_pos_in_active * 0.5f);  // 提高基础亮度，减少渐变范围
                    
                    // 如果LED刚好在边界上（active_led_count的小数部分），降低亮度实现平滑过渡
                    if (led_from_top + 1 > active_led_count) {
                        float fractional_part = active_led_count - led_from_top;
                        brightness_factor *= fractional_part;
                    }
                    
                    led_brightness = brightness_factor;
                } else {
                    // 超出能量范围，LED关闭
                    led_brightness = 0.0f;
                }
                
                // 限制亮度范围
                led_brightness = fminf(led_brightness, 1.0f);
                led_brightness = fmaxf(led_brightness, 0.0f);
                
                // 生成颜色并设置LED
                uint32_t color = HsvToRgb(hue, saturation, led_brightness);
                setRgbColorInternal(c, i, color);
            }
        }
    }

    static void RgbEffectTask(void* arg) {
        Stm32Impl* stm32 = static_cast<Stm32Impl*>(arg);
        stm32->EffectTaskLoop();
        vTaskDelete(NULL);
    }

    void EffectTaskLoop() {
        setRgbStripBrightness(0, 50);
        setRgbStripBrightness(1, 50);
        int position = 0;
        int breathe_step = 0;
        float hue = 0.0f;

        while (effect_running_) {
            switch (effect_mode_.load()) {
                case LightMode::OFF:
                    clearAllRgb();
                    vTaskDelay(pdMS_TO_TICKS(100));
                    break;

                case LightMode::STATIC:
                    setAllRgbColor(effect_color_);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    break;

                case LightMode::BREATHE: {
                    float brightness = (sin(breathe_step * 0.1f) + 1.0f) / 2.0f;
                    uint8_t r = ((effect_color_ >> 16) & 0xFF) * brightness;
                    uint8_t g = ((effect_color_ >> 8) & 0xFF) * brightness;
                    uint8_t b = (effect_color_ & 0xFF) * brightness;
                    uint32_t color = (r << 16) | (g << 8) | b;
                    setAllRgbColor(color);
                    breathe_step++;
                    if (breathe_step > 1000) breathe_step = 0;
                    vTaskDelay(pdMS_TO_TICKS(60));
                    break;
                }

                case LightMode::RAINBOW: {
                    for (int c = 0; c < NUM_RGB_CHANNELS; c++) {
                        for (int i = 0; i < NUM_LEDS_PER_GROUP; i++) {
                            int led_pos = c * NUM_LEDS_PER_GROUP + i;
                            float led_hue = (hue + led_pos * 15.0f) / 360.0f;
                            uint32_t color = HsvToRgb(led_hue, 1.0f, 1.0f);
                            setRgbColorInternal(c, i, color);
                        }
                    }
                    hue += 2.0f;
                    if (hue >= 360.0f) hue = 0.0f;
                    vTaskDelay(pdMS_TO_TICKS(effect_speed_));
                    break;
                }

                case LightMode::CHASE: {
                    clearAllRgb();
                    int total_leds = NUM_RGB_CHANNELS * NUM_LEDS_PER_GROUP;
                    int led_pos = position % total_leds;
                    int channel = led_pos / NUM_LEDS_PER_GROUP;
                    int led_index = led_pos % NUM_LEDS_PER_GROUP;
                    setRgbColorInternal(channel, led_index, effect_color_);
                    position = (position + 1) % total_leds;
                    vTaskDelay(pdMS_TO_TICKS(50));
                    break;
                }

                case LightMode::MUSIC_REACTIVE: {
                    float volume = audio_volume_.load();
                    float bass = audio_bass_.load();
                    float mid = audio_mid_.load();
                    float treble = audio_treble_.load();
                    
                    // 统一计算总能量（不分频段）
                    float total_energy = (bass + mid + treble) / 3.0f;
                    // 同时使用音量作为补充，提高响应灵敏度
                    float volume_energy = fminf(volume * 7.0f, 1.0f);  // 进一步提高音量放大倍数
                    // 综合能量，提高放大倍数让跳动更明显
                    float combined_energy = (total_energy * 0.65f + volume_energy * 0.35f);  // 增加音量权重
                    float energy = fminf(combined_energy * 8.0f, 1.0f);  // 提高放大倍数到8.0，让跳动更明显
                    
                    // 渐变颜色：蓝色(低) → 绿色(中) → 红色(高)
                    // HSV色相：蓝色=240°, 绿色=120°, 红色=0°
                    float hue = 0.0f;
                    if (energy < 0.5f) {
                        // 低能量到中能量：蓝色到绿色 (240° → 120°)
                        float t = energy / 0.5f;  // 0.0 到 1.0
                        hue = (240.0f - t * 120.0f) / 360.0f;  // 240° 到 120°
                    } else {
                        // 中能量到高能量：绿色到红色 (120° → 0°)
                        float t = (energy - 0.5f) / 0.5f;  // 0.0 到 1.0
                        hue = (120.0f - t * 120.0f) / 360.0f;  // 120° 到 0°
                    }
                    float saturation = 1.0f;  // 全饱和度，颜色更鲜艳
                    
                    // 渲染音乐随动效果（复用代码）
                    RenderMusicReactive(energy, hue, saturation);
                    
                    // 更新速度：根据能量调整，能量高时更新更快（响应更灵敏）
                    // 减少延迟让跳动更明显
                    int delay_ms = 25 - (int)(energy * 12.0f);
                    if (delay_ms < 15) delay_ms = 15;  // 最快15ms更新一次，让跳动更明显
                    vTaskDelay(pdMS_TO_TICKS(delay_ms));
                    break;
                }

                case LightMode::MUSIC_REACTIVE_RED: {
                    float volume = audio_volume_.load();
                    float bass = audio_bass_.load();
                    float mid = audio_mid_.load();
                    float treble = audio_treble_.load();
                    
                    // 统一计算总能量（与MUSIC_REACTIVE相同的优化）
                    float total_energy = (bass + mid + treble) / 3.0f;
                    float volume_energy = fminf(volume * 7.0f, 1.0f);
                    float combined_energy = (total_energy * 0.65f + volume_energy * 0.35f);
                    float energy = fminf(combined_energy * 8.0f, 1.0f);
                    
                    // 固定红色
                    float hue = 0.0f / 360.0f;  // 红色
                    float saturation = 1.0f;
                    
                    RenderMusicReactive(energy, hue, saturation);
                    
                    int delay_ms = 30 - (int)(energy * 15.0f);
                    if (delay_ms < 20) delay_ms = 20;
                    vTaskDelay(pdMS_TO_TICKS(delay_ms));
                    break;
                }

                case LightMode::MUSIC_REACTIVE_GREEN: {
                    float volume = audio_volume_.load();
                    float bass = audio_bass_.load();
                    float mid = audio_mid_.load();
                    float treble = audio_treble_.load();
                    
                    // 统一计算总能量（与MUSIC_REACTIVE相同的优化）
                    float total_energy = (bass + mid + treble) / 3.0f;
                    float volume_energy = fminf(volume * 7.0f, 1.0f);
                    float combined_energy = (total_energy * 0.65f + volume_energy * 0.35f);
                    float energy = fminf(combined_energy * 8.0f, 1.0f);
                    
                    // 固定绿色
                    float hue = 120.0f / 360.0f;  // 绿色
                    float saturation = 1.0f;
                    
                    RenderMusicReactive(energy, hue, saturation);
                    
                    int delay_ms = 30 - (int)(energy * 15.0f);
                    if (delay_ms < 20) delay_ms = 20;
                    vTaskDelay(pdMS_TO_TICKS(delay_ms));
                    break;
                }

                case LightMode::MUSIC_REACTIVE_BLUE: {
                    float volume = audio_volume_.load();
                    float bass = audio_bass_.load();
                    float mid = audio_mid_.load();
                    float treble = audio_treble_.load();
                    
                    // 统一计算总能量（与MUSIC_REACTIVE相同的优化）
                    float total_energy = (bass + mid + treble) / 3.0f;
                    float volume_energy = fminf(volume * 7.0f, 1.0f);
                    float combined_energy = (total_energy * 0.65f + volume_energy * 0.35f);
                    float energy = fminf(combined_energy * 8.0f, 1.0f);
                    
                    // 固定蓝色
                    float hue = 240.0f / 360.0f;  // 蓝色
                    float saturation = 1.0f;
                    
                    RenderMusicReactive(energy, hue, saturation);
                    
                    int delay_ms = 30 - (int)(energy * 15.0f);
                    if (delay_ms < 20) delay_ms = 20;
                    vTaskDelay(pdMS_TO_TICKS(delay_ms));
                    break;
                }

                default:
                    vTaskDelay(pdMS_TO_TICKS(100));
                    break;
            }
        }
    }

    uint32_t HsvToRgb(float h, float s, float v) {
        int i = (int)(h * 6.0f);
        float f = h * 6.0f - i;
        float p = v * (1.0f - s);
        float q = v * (1.0f - f * s);
        float t = v * (1.0f - (1.0f - f) * s);

        float r, g, b;
        switch (i % 6) {
            case 0: r = v; g = t; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = t; break;
            case 3: r = p; g = q; b = v; break;
            case 4: r = t; g = p; b = v; break;
            case 5: r = v; g = p; b = q; break;
            default: r = g = b = 0; break;
        }

        return ((uint8_t)(r * 255) << 16) | ((uint8_t)(g * 255) << 8) | (uint8_t)(b * 255);
    }
};

// EchoPyramid 实现

EchoPyramid::EchoPyramid(i2c_master_bus_handle_t i2c_bus, uint8_t stm32_addr) {
    stm32_ = new Stm32Impl(i2c_bus, stm32_addr);
}

EchoPyramid::~EchoPyramid() {
    stopTouchDetection();
    if (stm32_) {
        delete stm32_;
        stm32_ = nullptr;
    }
}

/**
 * Si5351 Clock Generator
 * I2C Address: 0x60
 * Reference: https://www.skyworksinc.com/-/media/Skyworks/SL/documents/public/application-notes/AN619.pdf
 */
Si5351::Si5351(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
    uint8_t w_buffer[10] = {0};

    // Disable all outputs
    WriteReg(3, 0xff);
    ESP_LOGI(TAG, "Si5351 Register 3 (OUTPUT_ENABLE_CONTROL): %02X", ReadReg(3));

    // Power down output drivers
    w_buffer[0] = 0x80;  // 10000000
    w_buffer[1] = 0x80;
    w_buffer[2] = 0x80;
    WriteRegs(16, w_buffer, 3);
    ESP_LOGI(TAG, "Si5351 Registers 16-23 configured");

    // Crystal Internal Load Capacitance
    WriteReg(183, 0xC0);  // 11000000 // Internal CL = 10 pF (default)
    ESP_LOGI(TAG, "Si5351 Register 183 (CRYSTAL_LOAD): %02X", ReadReg(183));

    // Multisynth NA Parameters
    w_buffer[0] = 0xFF;
    w_buffer[1] = 0xFD;
    w_buffer[2] = 0x00;
    w_buffer[3] = 0x09;
    w_buffer[4] = 0x26;
    w_buffer[5] = 0xF7;
    w_buffer[6] = 0x4F;
    w_buffer[7] = 0x72;
    WriteRegs(26, w_buffer, 8);
    ESP_LOGI(TAG, "Si5351 Registers 26-33 (Multisynth NA) configured");

    // Multisynth1 Parameters
    w_buffer[0] = 0x00;
    w_buffer[1] = 0x01;
    w_buffer[2] = 0x00;
    w_buffer[3] = 0x2F;
    w_buffer[4] = 0x00;
    w_buffer[5] = 0x00;
    w_buffer[6] = 0x00;
    w_buffer[7] = 0x00;
    WriteRegs(50, w_buffer, 8);
    ESP_LOGI(TAG, "Si5351 Registers 50-57 (Multisynth1) configured");

    // CLK1 Control
    // Bit 6: MS1 operates in integer mode
    // Bits 5-4: Select MultiSynth 1 as the source for CLK1
    WriteReg(17, ((3 << 2) | (1 << 6)));  // 01001100 = 0x4C
    ESP_LOGI(TAG, "Si5351 Register 17 (CLK1_CONTROL): %02X", ReadReg(17));

    // PLL Reset
    WriteReg(177, 0xA0);  // 10100000
    ESP_LOGI(TAG, "Si5351 Register 177 (PLL_RESET): %02X", ReadReg(177));

    // Enable all outputs
    WriteReg(3, 0x00);
    ESP_LOGI(TAG, "Si5351 Register 3 (OUTPUT_ENABLE_CONTROL): %02X - outputs enabled", ReadReg(3));
}

/**
 * AW87559 Audio Amplifier
 * I2C Address: 0x5B
 */
Aw87559::Aw87559(i2c_master_bus_handle_t i2c_bus, uint8_t addr) : I2cDevice(i2c_bus, addr) {
    ESP_LOGI(TAG, "AW87559 ID: %02X", ReadReg(0x00)); // ID: 0x5A
    WriteReg(0x01, 0x78); // default enable PA
}

void Aw87559::SetSpeaker(bool enable) {
    WriteReg(0x01, enable ? 0x78 : 0x30); // BIT3: PA Enable
}

void EchoPyramid::addTouchEventCallback(TouchEventCallback callback) {
    touch_callbacks_.push_back(callback);
}

void EchoPyramid::clearTouchEventCallbacks() {
    touch_callbacks_.clear();
}

void EchoPyramid::setLightMode(LightMode mode) {
    if (stm32_) {
        stm32_->setEffectMode(mode);
    }
}

LightMode EchoPyramid::getLightMode() const {
    if (stm32_) {
        return stm32_->getEffectMode();
    }
    return LightMode::OFF;
}

void EchoPyramid::setLightColor(uint32_t color) {
    if (stm32_) {
        stm32_->setEffectColor(color);
    }
}

void EchoPyramid::setLightBrightness(uint8_t strip, uint8_t brightness) {
    if (stm32_) {
        stm32_->setRgbStripBrightness(strip, brightness);
    }
}

void EchoPyramid::processAudioData(const int16_t* samples, size_t num_samples, int channels) {
    if (stm32_) {
        stm32_->processAudioData(samples, num_samples, channels);
    }
}

void EchoPyramid::startTouchDetection() {
    if (touch_task_handle_ == nullptr) {
        xTaskCreate(TouchTask, "touch_task", 8192, this, 3, &touch_task_handle_);
        ESP_LOGI(TAG, "Touch task started");
    }
}

void EchoPyramid::stopTouchDetection() {
    if (touch_task_handle_ != nullptr) {
        vTaskDelete(touch_task_handle_);
        touch_task_handle_ = nullptr;
        ESP_LOGI(TAG, "Touch task stopped");
    }
}

void EchoPyramid::pauseTouchCallbacks() {
    touch_callbacks_paused_ = true;
}

void EchoPyramid::resumeTouchCallbacks() {
    touch_callbacks_paused_ = false;
}

bool EchoPyramid::isTouchCallbacksPaused() const {
    return touch_callbacks_paused_.load();
}

void EchoPyramid::TouchTask(void* arg) {
    EchoPyramid* pyramid = static_cast<EchoPyramid*>(arg);
    
    while (true) {
        if (pyramid->stm32_ == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

        // 读取所有触摸状态
        bool touch_states[4] = {
            pyramid->stm32_->readTouchStatus(1),
            pyramid->stm32_->readTouchStatus(2),
            pyramid->stm32_->readTouchStatus(3),
            pyramid->stm32_->readTouchStatus(4)
        };

        // 在检测到触摸状态变化时打印触摸状态
        if (touch_states[0] != pyramid->touch_last_state_[0] ||
            touch_states[1] != pyramid->touch_last_state_[1] ||
            touch_states[2] != pyramid->touch_last_state_[2] ||
            touch_states[3] != pyramid->touch_last_state_[3]) {
            ESP_LOGI(TAG, "Touch states: %d, %d, %d, %d", 
                     touch_states[0], touch_states[1], touch_states[2], touch_states[3]);
        }

        // 处理Touch1/2的滑动检测
        pyramid->ProcessSwipe(touch_states[0], touch_states[1],
                             pyramid->touch_last_state_[0], pyramid->touch_last_state_[1],
                             0, 1, 2, current_time);

        // 处理Touch3/4的滑动检测
        pyramid->ProcessSwipe(touch_states[2], touch_states[3],
                             pyramid->touch_last_state_[2], pyramid->touch_last_state_[3],
                             1, 3, 4, current_time);

        // 更新状态
        for (int i = 0; i < 4; i++) {
            pyramid->touch_last_state_[i] = touch_states[i];
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    vTaskDelete(NULL);
}

void EchoPyramid::ProcessSwipe(bool touch1, bool touch2, uint8_t& last_state1, uint8_t& last_state2,
                               uint8_t swipe_index, uint8_t touch_num1, uint8_t touch_num2, uint32_t current_time) {
    // 检测按下事件
    if ((touch1 && !last_state1) || (touch2 && !last_state2)) {
        if (touch_swipe_first_[swipe_index] == 0) {
            // 记录第一个按下的按键
            if (touch1 && !last_state1) {
                touch_swipe_first_[swipe_index] = touch_num1;
                touch_swipe_time_[swipe_index] = current_time;
            } else if (touch2 && !last_state2) {
                touch_swipe_first_[swipe_index] = touch_num2;
                touch_swipe_time_[swipe_index] = current_time;
            }
        } else {
            // 检测第二个按键是否在超时内按下
            if (current_time - touch_swipe_time_[swipe_index] <= TOUCH_SWIPE_TIMEOUT_MS) {
                if (touch_swipe_first_[swipe_index] == touch_num1 && touch2 && !last_state2) {
                    // touch_num1 → touch_num2 滑动
                    TouchEvent event = (swipe_index == 0) ? TouchEvent::LEFT_SLIDE_UP : TouchEvent::RIGHT_SLIDE_UP;
                    NotifyTouchEvent(event);
                    touch_swipe_first_[swipe_index] = 0;
                } else if (touch_swipe_first_[swipe_index] == touch_num2 && touch1 && !last_state1) {
                    // touch_num2 → touch_num1 滑动
                    TouchEvent event = (swipe_index == 0) ? TouchEvent::LEFT_SLIDE_DOWN : TouchEvent::RIGHT_SLIDE_DOWN;
                    NotifyTouchEvent(event);
                    touch_swipe_first_[swipe_index] = 0;
                }
            } else {
                touch_swipe_first_[swipe_index] = 0;
            }
        }
    }

    // 如果两个按键都释放，重置滑动检测
    if (!touch1 && !touch2 && touch_swipe_first_[swipe_index] != 0) {
        touch_swipe_first_[swipe_index] = 0;
    }

    // 滑动检测超时处理
    if (touch_swipe_first_[swipe_index] != 0 && (current_time - touch_swipe_time_[swipe_index] > TOUCH_SWIPE_TIMEOUT_MS)) {
        touch_swipe_first_[swipe_index] = 0;
    }
}

void EchoPyramid::NotifyTouchEvent(TouchEvent event) {
    if (touch_callbacks_paused_) {
        return;
    }
    
    const char* event_names[] = {"LEFT_SLIDE_UP", "LEFT_SLIDE_DOWN", "RIGHT_SLIDE_UP", "RIGHT_SLIDE_DOWN"};
    ESP_LOGI(TAG, "Touch event: %s", event_names[static_cast<int>(event)]);

    for (auto& callback : touch_callbacks_) {
        if (callback) {
            callback(event);
        }
    }
}

