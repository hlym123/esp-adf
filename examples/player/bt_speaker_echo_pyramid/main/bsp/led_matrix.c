#include "led_matrix.h"
#include "esp_log.h"
#include "led_strip.h"
#include "led_strip_rmt.h"
#include <string.h>
#include <stdlib.h>
#include <stdatomic.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "LED_MATRIX";

/* ================= 配置区 ================= */

// 0 = 普通行扫描
// 1 = 蛇形排列（常见 5x5 WS2812）
#define MATRIX_SNAKE_LAYOUT   0

#define FRAME_DELAY_MS       120

/* ========================================== */

typedef struct {
    led_strip_handle_t strip;
    TaskHandle_t task;
    QueueHandle_t queue;
    _Atomic(bool) running;
} led_matrix_t;

/* ================== 工具函数 ================== */

static inline int matrix_xy_to_index(int x, int y)
{
#if MATRIX_SNAKE_LAYOUT
    if (y & 1) {
        return y * MATRIX_SIZE + (MATRIX_SIZE - 1 - x);
    } else {
        return y * MATRIX_SIZE + x;
    }
#else
    return y * MATRIX_SIZE + x;
#endif
}

static inline bool effect_interrupted(led_matrix_t *m)
{
    return !atomic_load(&m->running)
        || uxQueueMessagesWaiting(m->queue) > 0;
}

/* ================== 基础操作 ================== */

void led_matrix_set_pixel(void* handle, int x, int y,
                          uint8_t r, uint8_t g, uint8_t b)
{
    if (!handle) return;
    if (x < 0 || x >= MATRIX_SIZE || y < 0 || y >= MATRIX_SIZE) return;

    led_matrix_t *m = handle;
    int idx = matrix_xy_to_index(x, y);
    led_strip_set_pixel(m->strip, idx, r, g, b);
}

void led_matrix_clear(void* handle)
{
    if (!handle) return;
    led_matrix_t *m = handle;
    led_strip_clear(m->strip);
}

void led_matrix_refresh(void* handle)
{
    if (!handle) return;
    led_matrix_t *m = handle;
    led_strip_refresh(m->strip);
}

/* ================== Ripple 效果 ================== */

static void effect_ripple(led_matrix_t *m)
{
    const int cx = 2, cy = 2;
    const int max_r = 4;

    for (int r = 0; r <= max_r; r++) {
        if (effect_interrupted(m)) return;

        led_matrix_clear(m);

        int r2 = r * r;
        int max2 = max_r * max_r;

        for (int y = 0; y < MATRIX_SIZE; y++) {
            for (int x = 0; x < MATRIX_SIZE; x++) {
                int dx = x - cx;
                int dy = y - cy;
                int d2 = dx * dx + dy * dy;

                if (d2 <= r2) {
                    float br = 1.0f - (float)d2 / max2;
                    if (br < 0.15f) br = 0.15f;
                    led_matrix_set_pixel(
                        m, x, y,
                        0,
                        0,
                        (uint8_t)(60 * br)
                    );
                }
            }
        }

        led_matrix_refresh(m);
        vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));
    }

    led_matrix_clear(m);
    led_matrix_refresh(m);
}

/* ================== 箭头绘制 ================== */

typedef enum {
    ARROW_LEFT,
    ARROW_RIGHT,
    ARROW_UP,
    ARROW_DOWN,
} arrow_dir_t;

static void draw_arrow(led_matrix_t *m, arrow_dir_t dir, int pos)
{
    uint8_t r = 0, g = 0, b = 150;  // 蓝色，亮度60%
    const int c = 2;   // center = 2 (5x5)

    switch (dir) {

    /* ========== ← 左箭头（从左向右移动）========== */
    case ARROW_LEFT:
        // pos表示箭头主体（3像素竖条）所在的x坐标
        // 箭头形状：尾部(2) - 主体(3:上中下) - 尖角(1)
        
        // 尾部（2个像素）
        if (pos - 2 >= 0 && pos - 2 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, pos - 2, c, r, g, b);
        }
        if (pos - 1 >= 0 && pos - 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, pos - 1, c, r, g, b);
        }
        
        // 主体（3个像素：上、中、下）
        if (pos >= 0 && pos < MATRIX_SIZE) {
            if (c - 1 >= 0) {
                led_matrix_set_pixel(m, pos, c - 1, r, g, b);
            }
            led_matrix_set_pixel(m, pos, c, r, g, b);
            if (c + 1 < MATRIX_SIZE) {
                led_matrix_set_pixel(m, pos, c + 1, r, g, b);
            }
        }
        
        // 尖角（1个像素，向右）
        if (pos + 1 >= 0 && pos + 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, pos + 1, c, r, g, b);
        }
        break;

    /* ========== → 右箭头（从右向左移动）========== */
    case ARROW_RIGHT:
        // pos表示箭头主体（3像素竖条）所在的x坐标，从右向左移动
        // 箭头形状：尖角(1) - 主体(3:上中下) - 尾部(2) - 与左箭头对称
        
        // 尖角（1个像素，向左）
        if (pos - 1 >= 0 && pos - 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, pos - 1, c, r, g, b);
        }
        
        // 主体（3个像素：上、中、下）
        if (pos >= 0 && pos < MATRIX_SIZE) {
            if (c - 1 >= 0) {
                led_matrix_set_pixel(m, pos, c - 1, r, g, b);
            }
            led_matrix_set_pixel(m, pos, c, r, g, b);
            if (c + 1 < MATRIX_SIZE) {
                led_matrix_set_pixel(m, pos, c + 1, r, g, b);
            }
        }
        
        // 尾部（2个像素）
        if (pos + 1 >= 0 && pos + 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, pos + 1, c, r, g, b);
        }
        if (pos + 2 >= 0 && pos + 2 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, pos + 2, c, r, g, b);
        }
        break;

    /* ========== ↑ 上箭头（从上向下移动）========== */
    case ARROW_UP:
        // pos表示箭头主体（3像素横条）所在的y坐标
        // 箭头形状：尾部(2) - 主体(3:左中右) - 尖角(1)
        
        // 尾部（2个像素）
        if (pos - 2 >= 0 && pos - 2 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, c, pos - 2, r, g, b);
        }
        if (pos - 1 >= 0 && pos - 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, c, pos - 1, r, g, b);
        }
        
        // 主体（3个像素：左、中、右）
        if (pos >= 0 && pos < MATRIX_SIZE) {
            if (c - 1 >= 0) {
                led_matrix_set_pixel(m, c - 1, pos, r, g, b);
            }
            led_matrix_set_pixel(m, c, pos, r, g, b);
            if (c + 1 < MATRIX_SIZE) {
                led_matrix_set_pixel(m, c + 1, pos, r, g, b);
            }
        }
        
        // 尖角（1个像素，向下）
        if (pos + 1 >= 0 && pos + 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, c, pos + 1, r, g, b);
        }
        break;

    /* ========== ↓ 下箭头（从下向上移动）========== */
    case ARROW_DOWN:
        // pos表示箭头主体（3像素横条）所在的y坐标，从下向上移动
        // 箭头形状：尖角(1) - 主体(3:左中右) - 尾部(2) - 与上箭头对称
        
        // 尖角（1个像素，向上）
        if (pos - 1 >= 0 && pos - 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, c, pos - 1, r, g, b);
        }
        
        // 主体（3个像素：左、中、右）
        if (pos >= 0 && pos < MATRIX_SIZE) {
            if (c - 1 >= 0) {
                led_matrix_set_pixel(m, c - 1, pos, r, g, b);
            }
            led_matrix_set_pixel(m, c, pos, r, g, b);
            if (c + 1 < MATRIX_SIZE) {
                led_matrix_set_pixel(m, c + 1, pos, r, g, b);
            }
        }
        
        // 尾部（2个像素）
        if (pos + 1 >= 0 && pos + 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, c, pos + 1, r, g, b);
        }
        if (pos + 2 >= 0 && pos + 2 < MATRIX_SIZE) {
            led_matrix_set_pixel(m, c, pos + 2, r, g, b);
        }
        break;
    }
}

static void effect_arrow(led_matrix_t *m, arrow_dir_t dir)
{
    int start_pos, end_pos, step;
    
    // 根据箭头方向决定移动方向
    if (dir == ARROW_LEFT || dir == ARROW_UP) {
        // 左箭头和上箭头：从左到右/从上到下移动
        start_pos = -2;
        end_pos = MATRIX_SIZE + 2;
        step = 1;
    } else {
        // 右箭头和下箭头：从右到左/从下到上移动
        start_pos = MATRIX_SIZE + 2;
        end_pos = -2;
        step = -1;
    }
    
    for (int pos = start_pos; step > 0 ? pos <= end_pos : pos >= end_pos; pos += step) {
        if (effect_interrupted(m)) return;

        led_matrix_clear(m);
        draw_arrow(m, dir, pos);
        led_matrix_refresh(m);

        vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));
    }

    led_matrix_clear(m);
    led_matrix_refresh(m);
}
 

static void led_matrix_task(void *arg)
{
    led_matrix_t *m = arg;
    led_matrix_effect_t eff;

    while (atomic_load(&m->running)) {
        if (xQueueReceive(m->queue, &eff, portMAX_DELAY) != pdTRUE)
            continue;

        led_matrix_clear(m);

        switch (eff) {
        case LED_MATRIX_EFFECT_OFF:
            led_matrix_refresh(m);
            break;

        case LED_MATRIX_EFFECT_RIPPLE:
            effect_ripple(m);
            break;

        case LED_MATRIX_EFFECT_LEFT_ARROW:
            effect_arrow(m, ARROW_LEFT);
            break;

        case LED_MATRIX_EFFECT_RIGHT_ARROW:
            effect_arrow(m, ARROW_RIGHT);
            break;

        case LED_MATRIX_EFFECT_UP_ARROW:
            effect_arrow(m, ARROW_UP);
            break;

        case LED_MATRIX_EFFECT_DOWN_ARROW:
            effect_arrow(m, ARROW_DOWN);
            break;

        default:
            ESP_LOGW(TAG, "Unknown effect %d", eff);
            break;
        }
    }

    vTaskDelete(NULL);
}

 
void* led_matrix_init(int gpio_num)
{
    led_matrix_t *m = calloc(1, sizeof(led_matrix_t));
    if (!m) return NULL;

    atomic_init(&m->running, true);

    m->queue = xQueueCreate(5, sizeof(led_matrix_effect_t));
    if (!m->queue) goto fail;

    led_strip_config_t strip_cfg = {
        .strip_gpio_num = gpio_num,
        .max_leds = MATRIX_TOTAL_PIXELS,
        .led_model = LED_MODEL_WS2812,
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
    };

    led_strip_rmt_config_t rmt_cfg = {
        .resolution_hz = 10 * 1000 * 1000,
    };

    if (led_strip_new_rmt_device(&strip_cfg, &rmt_cfg, &m->strip) != ESP_OK)
        goto fail;

    xTaskCreate(led_matrix_task, "led_matrix",
                8192, m, 3, &m->task);

    led_strip_clear(m->strip);
    led_strip_refresh(m->strip);

    return m;

fail:
    if (m->queue) vQueueDelete(m->queue);
    free(m);
    return NULL;
}

void led_matrix_deinit(void* handle)
{
    if (!handle) return;
    led_matrix_t *m = handle;

    atomic_store(&m->running, false);

    led_matrix_effect_t off = LED_MATRIX_EFFECT_OFF;
    xQueueSend(m->queue, &off, 0);

    vTaskDelay(pdMS_TO_TICKS(100));

    led_strip_clear(m->strip);
    led_strip_refresh(m->strip);

    led_strip_del(m->strip);
    vQueueDelete(m->queue);
    free(m);
}

void led_matrix_trigger_effect(void* handle, led_matrix_effect_t effect)
{
    if (!handle) return;
    led_matrix_t *m = handle;

    if (xQueueSend(m->queue, &effect, 0) != pdTRUE) {
        ESP_LOGW(TAG, "effect queue full");
    }
}
