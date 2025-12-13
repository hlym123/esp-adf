#ifndef AUDIO_ANALYZER_H
#define AUDIO_ANALYZER_H

#include "audio_element.h"
#include "bsp/echo_pyramid.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    EchoPyramid* echo_pyramid;  // EchoPyramid 实例，用于传递音频数据
} audio_analyzer_cfg_t;

/**
 * @brief 创建音频分析 element
 * 
 * 这个 element 会从音频流中提取音频数据并传递给 EchoPyramid 进行音乐随动灯效分析
 * 
 * @param config 配置参数，包含 EchoPyramid 实例指针
 * @return audio_element_handle_t 返回创建的 element 句柄
 */
audio_element_handle_t audio_analyzer_init(audio_analyzer_cfg_t *config);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_ANALYZER_H
