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


#define FRAME_DELAY_MS  150

typedef struct {
    led_strip_handle_t strip;
    TaskHandle_t task;
    QueueHandle_t queue;
    _Atomic(bool) running;
    TickType_t last_refresh_tick;
} led_matrix_t;

static led_matrix_t *g_led_matrix = NULL;

static inline int matrix_xy_to_index(int x, int y)
{
    return y * MATRIX_SIZE + x;
}

static inline bool effect_interrupted(led_matrix_t *m)
{
    return !atomic_load(&m->running) || uxQueueMessagesWaiting(m->queue) > 0;
}

void led_matrix_set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b)
{
    if (!g_led_matrix) return;
    if (x < 0 || x >= MATRIX_SIZE || y < 0 || y >= MATRIX_SIZE) return;

    int idx = matrix_xy_to_index(x, y);
    led_strip_set_pixel(g_led_matrix->strip, idx, r, g, b);
}

void led_matrix_clear(void)
{
    if (!g_led_matrix) return;
    led_strip_clear(g_led_matrix->strip);
}

void led_matrix_refresh(void)
{
    if (!g_led_matrix) return;
    
    const TickType_t min_interval_ticks = pdMS_TO_TICKS(50);
    TickType_t current_tick = xTaskGetTickCount();
    TickType_t elapsed = current_tick - g_led_matrix->last_refresh_tick;
    
    if (elapsed < min_interval_ticks) {
        vTaskDelay(min_interval_ticks - elapsed);
        current_tick = xTaskGetTickCount();
    }
    
    led_strip_refresh(g_led_matrix->strip);
    g_led_matrix->last_refresh_tick = current_tick;
}

static void effect_ripple(void)
{
    const int cx = 2, cy = 2;
    const int max_r = 4;

    for (int r = 0; r <= max_r; r++) {
        if (!g_led_matrix || effect_interrupted(g_led_matrix)) break;

        led_matrix_clear();

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
                    led_matrix_set_pixel(x, y, 0, 0, (uint8_t)(60 * br));
                }
            }
        }

        led_matrix_refresh();
        vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));
    }

    led_matrix_clear();
    led_matrix_refresh();
}

typedef enum {
    ARROW_LEFT,
    ARROW_RIGHT,
    ARROW_UP,
    ARROW_DOWN,
} arrow_dir_t;

static void draw_arrow(arrow_dir_t dir, int pos)
{
    uint8_t r = 0, g = 0, b = 50;
    const int c = 2;

    switch (dir) {

    /* ========== ← 左箭头（从左向右移动）========== */
    case ARROW_LEFT:
        // pos表示箭头主体（3像素竖条）所在的x坐标
        // 箭头形状：尾部(2) - 主体(3:上中下) - 尖角(1)
        
        // 尾部（2个像素）
        if (pos - 2 >= 0 && pos - 2 < MATRIX_SIZE) {
            led_matrix_set_pixel(pos - 2, c, r, g, b);
        }
        if (pos - 1 >= 0 && pos - 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(pos - 1, c, r, g, b);
        }
        
        // 主体（3个像素：上、中、下）
        if (pos >= 0 && pos < MATRIX_SIZE) {
            if (c - 1 >= 0) {
                led_matrix_set_pixel(pos, c - 1, r, g, b);
            }
            led_matrix_set_pixel(pos, c, r, g, b);
            if (c + 1 < MATRIX_SIZE) {
                led_matrix_set_pixel(pos, c + 1, r, g, b);
            }
        }
        
        // 尖角（1个像素，向右）
        if (pos + 1 >= 0 && pos + 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(pos + 1, c, r, g, b);
        }
        break;

    /* ========== → 右箭头（从右向左移动）========== */
    case ARROW_RIGHT:
        // pos表示箭头主体（3像素竖条）所在的x坐标，从右向左移动
        // 箭头形状：尖角(1) - 主体(3:上中下) - 尾部(2) - 与左箭头对称
        
        // 尖角（1个像素，向左）
        if (pos - 1 >= 0 && pos - 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(pos - 1, c, r, g, b);
        }
        
        // 主体（3个像素：上、中、下）
        if (pos >= 0 && pos < MATRIX_SIZE) {
            if (c - 1 >= 0) {
                led_matrix_set_pixel(pos, c - 1, r, g, b);
            }
            led_matrix_set_pixel(pos, c, r, g, b);
            if (c + 1 < MATRIX_SIZE) {
                led_matrix_set_pixel(pos, c + 1, r, g, b);
            }
        }
        
        // 尾部（2个像素）
        if (pos + 1 >= 0 && pos + 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(pos + 1, c, r, g, b);
        }
        if (pos + 2 >= 0 && pos + 2 < MATRIX_SIZE) {
            led_matrix_set_pixel(pos + 2, c, r, g, b);
        }
        break;

    /* ========== ↑ 上箭头（从上向下移动）========== */
    case ARROW_UP:
        // pos表示箭头主体（3像素横条）所在的y坐标
        // 箭头形状：尾部(2) - 主体(3:左中右) - 尖角(1)
        
        // 尾部（2个像素）
        if (pos - 2 >= 0 && pos - 2 < MATRIX_SIZE) {
            led_matrix_set_pixel(c, pos - 2, r, g, b);
        }
        if (pos - 1 >= 0 && pos - 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(c, pos - 1, r, g, b);
        }
        
        // 主体（3个像素：左、中、右）
        if (pos >= 0 && pos < MATRIX_SIZE) {
            if (c - 1 >= 0) {
                led_matrix_set_pixel(c - 1, pos, r, g, b);
            }
            led_matrix_set_pixel(c, pos, r, g, b);
            if (c + 1 < MATRIX_SIZE) {
                led_matrix_set_pixel(c + 1, pos, r, g, b);
            }
        }
        
        // 尖角（1个像素，向下）
        if (pos + 1 >= 0 && pos + 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(c, pos + 1, r, g, b);
        }
        break;

    /* ========== ↓ 下箭头（从下向上移动）========== */
    case ARROW_DOWN:
        // pos表示箭头主体（3像素横条）所在的y坐标，从下向上移动
        // 箭头形状：尖角(1) - 主体(3:左中右) - 尾部(2) - 与上箭头对称
        
        // 尖角（1个像素，向上）
        if (pos - 1 >= 0 && pos - 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(c, pos - 1, r, g, b);
        }
        
        // 主体（3个像素：左、中、右）
        if (pos >= 0 && pos < MATRIX_SIZE) {
            if (c - 1 >= 0) {
                led_matrix_set_pixel(c - 1, pos, r, g, b);
            }
            led_matrix_set_pixel(c, pos, r, g, b);
            if (c + 1 < MATRIX_SIZE) {
                led_matrix_set_pixel(c + 1, pos, r, g, b);
            }
        }
        
        // 尾部（2个像素）
        if (pos + 1 >= 0 && pos + 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(c, pos + 1, r, g, b);
        }
        if (pos + 2 >= 0 && pos + 2 < MATRIX_SIZE) {
            led_matrix_set_pixel(c, pos + 2, r, g, b);
        }
        break;
    }
}

static void effect_arrow(arrow_dir_t dir)
{
    if (!g_led_matrix) return;
    
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
        if (effect_interrupted(g_led_matrix)) break;

        led_matrix_clear();
        draw_arrow(dir, pos);
        led_matrix_refresh();

        vTaskDelay(pdMS_TO_TICKS(FRAME_DELAY_MS));
    }

    led_matrix_clear();
    led_matrix_refresh();
}

// 红色X效果（音量已到最小值）
static void effect_red_x(void)
{
    uint8_t r = 50, g = 0, b = 0;  // 红色
    
    led_matrix_clear();
    // 绘制X形状：两条对角线
    // 主对角线：从(0,0)到(4,4)
    for (int i = 0; i < MATRIX_SIZE; i++) {
        led_matrix_set_pixel(i, i, r, g, b);
    }
    // 副对角线：从(0,4)到(4,0)
    for (int i = 0; i < MATRIX_SIZE; i++) {
        led_matrix_set_pixel(i, MATRIX_SIZE - 1 - i, r, g, b);
    }
    led_matrix_refresh();

    vTaskDelay(pdMS_TO_TICKS(500)); 
    led_matrix_clear();
    led_matrix_refresh();
}

// 上箭头抖动效果（音量已到最大值）
static void effect_up_arrow_shake(void)
{
    if (!g_led_matrix) return;
    
    const int c = 2;  // center
    uint8_t r = 0, g = 0, b = 50;  // 蓝色
    
    // 抖动3次，在中心位置上下小幅移动
    for (int shake = 0; shake < 3; shake++) {
        if (effect_interrupted(g_led_matrix)) break;
        
        led_matrix_clear();
        
        // 使用ARROW_UP的绘制逻辑，pos表示箭头主体所在的y坐标
        // 抖动时pos在中心位置(c=2)附近变化，向上抖动：在y=2和y=3之间交替（向下移动一个像素）
        int pos = c + ((shake % 2 == 0) ? 0 : 1);  // 在y=2和y=3之间交替（向下移动一个像素）
        
        // 尾部（2个像素，在pos下方）
        if (pos - 2 >= 0 && pos - 2 < MATRIX_SIZE) {
            led_matrix_set_pixel(c, pos - 2, r, g, b);
        }
        if (pos - 1 >= 0 && pos - 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(c, pos - 1, r, g, b);
        }
        
        // 主体（3个像素：左、中、右）
        if (pos >= 0 && pos < MATRIX_SIZE) {
            if (c - 1 >= 0) {
                led_matrix_set_pixel(c - 1, pos, r, g, b);
            }
            led_matrix_set_pixel(c, pos, r, g, b);
            if (c + 1 < MATRIX_SIZE) {
                led_matrix_set_pixel(c + 1, pos, r, g, b);
            }
        }
        
        // 尖角（1个像素，向上）
        if (pos + 1 >= 0 && pos + 1 < MATRIX_SIZE) {
            led_matrix_set_pixel(c, pos + 1, r, g, b);
        }
        
        led_matrix_refresh();
        vTaskDelay(pdMS_TO_TICKS(200));  // 抖动间隔150ms
    }
    
    led_matrix_clear();
    led_matrix_refresh();
}

static void led_matrix_task(void *arg)
{
    led_matrix_t *m = arg;
    led_matrix_effect_t eff;

    while (atomic_load(&m->running)) {
        if (xQueueReceive(m->queue, &eff, portMAX_DELAY) != pdTRUE)
            continue;

        switch (eff) {
        case LED_MATRIX_EFFECT_OFF:
            led_matrix_clear();
            led_matrix_refresh();
            break;

        case LED_MATRIX_EFFECT_RIPPLE:
            effect_ripple();
            break;

        case LED_MATRIX_EFFECT_LEFT_ARROW:
            effect_arrow(ARROW_LEFT);
            break;

        case LED_MATRIX_EFFECT_RIGHT_ARROW:
            effect_arrow(ARROW_RIGHT);
            break;

        case LED_MATRIX_EFFECT_UP_ARROW:
            effect_arrow(ARROW_UP);
            break;

        case LED_MATRIX_EFFECT_DOWN_ARROW:
            effect_arrow(ARROW_DOWN);
            break;

        case LED_MATRIX_EFFECT_RED_X:
            effect_red_x();
            break;

        case LED_MATRIX_EFFECT_UP_ARROW_SHAKE:
            effect_up_arrow_shake();
            break;

        default:
            ESP_LOGW(TAG, "Unknown effect %d", eff);
            break;
        }
    }

    vTaskDelete(NULL);
}
 
void led_matrix_init(int gpio_num)
{
    if (g_led_matrix) {
        ESP_LOGW(TAG, "led_matrix already initialized");
        return;
    }

    led_matrix_t *m = calloc(1, sizeof(led_matrix_t));
    if (!m) return;

    atomic_init(&m->running, true);
    m->last_refresh_tick = 0;

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

    g_led_matrix = m;

    led_strip_clear(m->strip);
    for (int i = 0; i < MATRIX_TOTAL_PIXELS; i++) {
        led_strip_set_pixel(m->strip, i, 0, 0, 0);
    }
    led_strip_refresh(m->strip);

    xTaskCreate(led_matrix_task, "led_matrix", 8192, m, 4, &m->task);

    return;

fail:
    if (m->queue) vQueueDelete(m->queue);
    free(m);
}

void led_matrix_deinit(void)
{
    if (!g_led_matrix) return;
    led_matrix_t *m = g_led_matrix;

    atomic_store(&m->running, false);

    led_matrix_effect_t off = LED_MATRIX_EFFECT_OFF;
    xQueueSend(m->queue, &off, 0);

    vTaskDelay(pdMS_TO_TICKS(200));

    led_strip_clear(m->strip);
    led_strip_refresh(m->strip);
    led_strip_del(m->strip);
    vQueueDelete(m->queue);
    free(m);
    g_led_matrix = NULL;
}

void led_matrix_trigger_effect(led_matrix_effect_t effect)
{
    if (!g_led_matrix) return;

    if (xQueueSend(g_led_matrix->queue, &effect, 0) != pdTRUE) {
        ESP_LOGW(TAG, "effect queue full");
    }
}

void led_matrix_show_ripple_effect(void)
{
    led_matrix_trigger_effect(LED_MATRIX_EFFECT_RIPPLE);
}
