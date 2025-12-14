#ifndef LED_MATRIX_H
#define LED_MATRIX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 5x5 LED矩阵
#define MATRIX_SIZE 5
#define MATRIX_TOTAL_PIXELS 25

// LED矩阵灯效类型
typedef enum {
    LED_MATRIX_EFFECT_OFF = 0,      // 关闭
    LED_MATRIX_EFFECT_RIPPLE = 1,   // 水滴/涟漪效果
    LED_MATRIX_EFFECT_LEFT_ARROW = 2,   // 左箭头（向左移动）
    LED_MATRIX_EFFECT_RIGHT_ARROW = 3,  // 右箭头（向右移动）
    LED_MATRIX_EFFECT_UP_ARROW = 4,     // 上箭头（向上移动）
    LED_MATRIX_EFFECT_DOWN_ARROW = 5,   // 下箭头（向下移动）
    LED_MATRIX_EFFECT_RED_X = 6,        // 红色X（音量已到最小值）
    LED_MATRIX_EFFECT_UP_ARROW_SHAKE = 7, // 上箭头抖动（音量已到最大值）
} led_matrix_effect_t;

/**
 * @brief 初始化5x5 LED矩阵
 * 
 * @param gpio_num GPIO引脚号
 */
void led_matrix_init(int gpio_num);

/**
 * @brief 设置5x5矩阵中的某个像素颜色
 * 
 * @param x X坐标 (0-4)
 * @param y Y坐标 (0-4)
 * @param r 红色 (0-255)
 * @param g 绿色 (0-255)
 * @param b 蓝色 (0-255)
 */
void led_matrix_set_pixel(int x, int y, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 清除所有LED
 * 
 */
void led_matrix_clear(void);

/**
 * @brief 刷新显示（发送数据到LED）
 * 
 */
void led_matrix_refresh(void);

/**
 * @brief 触发LED矩阵灯效
 * 
 * @param effect 要显示的灯效类型
 */
void led_matrix_trigger_effect(led_matrix_effect_t effect);

/**
 * @brief 显示水滴/涟漪效果（从中心向四周发散）
 * 
 */
void led_matrix_show_ripple_effect(void);

/**
 * @brief 释放LED矩阵资源
 * 
 */
void led_matrix_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // LED_MATRIX_H
