/*
@file: howling.c
@brief: 啸叫检测
@author:kang jin
@date:2025/05/05 
*/

#include <math.h>
#include <time.h>
#include "server/audio_server.h"
#include "server/server_core.h"
#include "system/app_core.h"
#include "generic/circular_buf.h"
#include "os/os_api.h"
#include "app_config.h"
#include "syscfg/syscfg_id.h"
#include "event/key_event.h"
#include "storage_device.h"
#include "fs/fs.h"
#include "system/timer.h"
#include "media/spectrum/SpectrumShow_api.h"
#include "device/gpio.h"
#include "asm/gpio.h"
#include "jl_math/kiss_fft.h"
#include "howling.h"

#ifdef FEEDBACK_SUPPRESSION_ENABLE

#if 1
#define log_info(x, ...)    printf("\n[Howling]>" x " \n", ## __VA_ARGS__)
#else
#define log_info(...)
#endif

struct recorder_hdl recorder_handler;
struct feedback_suppressor fb_suppressor;

#define __this (&recorder_handler)

// 陷波滤波器初始化 - 修正实现
static void init_notch_filter(float freq, float sample_rate, float q)
{
    const float omega = 2 * M_PI * freq / sample_rate;
    const float alpha = sinf(omega) / (2 * q);
    const float a0 = 1 + alpha;
    
    // 计算滤波器系数 (标准二阶直接形式)
    fb_suppressor.b0 = 1.0f / a0;
    fb_suppressor.b1 = -2.0f * cosf(omega) / a0;
    fb_suppressor.b2 = 1.0f / a0;
    fb_suppressor.a1 = -2.0f * cosf(omega) / a0;
    fb_suppressor.a2 = (1 - alpha) / a0;
    
    // 重置状态变量
    fb_suppressor.x1 = 0;
    fb_suppressor.x2 = 0;
    fb_suppressor.y1 = 0;
    fb_suppressor.y2 = 0;
    
    log_info("Notch filter: f=%.1fHz, Q=%.1f, SR=%.1fHz", 
             freq, q, sample_rate);
}

void fft_execute(const float *input, float *output, int size) {
    if(!input || !output || size <= 0) return;
    
    kiss_fft_cfg cfg = kiss_fft_alloc(size, 0, NULL, NULL);
    if (!cfg) {
        log_info("FFT allocation failed");
        return;
    }
    
    kiss_fft_cpx *in = (kiss_fft_cpx*)malloc(size * sizeof(kiss_fft_cpx));
    kiss_fft_cpx *out = (kiss_fft_cpx*)malloc(size * sizeof(kiss_fft_cpx));
    
    if (!in || !out) {
        log_info("FFT memory allocation failed");
        free(cfg);
        if (in) free(in);
        if (out) free(out);
        return;
    }
    
    // 准备输入数据
    for (int i = 0; i < size; i++) {
        in[i].r = input[i];
        in[i].i = 0;
    }
    
    // 执行 FFT
    kiss_fft(cfg, in, out);
    
    // 计算幅度谱
    for (int i = 0; i < size/2; i++) {
        output[i] = sqrtf(out[i].r*out[i].r + out[i].i*out[i].i);
    }

    free(in);
    free(out);
    free(cfg);
}

// 实时处理函数 - 修正实现
float feedback_cancellation(s16 sample)
{
    float input = (float)sample;
    
    // 计算滤波输出 (标准二阶直接形式)
    float output = fb_suppressor.b0 * input + 
                   fb_suppressor.b1 * fb_suppressor.x1 + 
                   fb_suppressor.b2 * fb_suppressor.x2 -
                   fb_suppressor.a1 * fb_suppressor.y1 - 
                   fb_suppressor.a2 * fb_suppressor.y2;
    
    // 更新状态变量
    fb_suppressor.x2 = fb_suppressor.x1;
    fb_suppressor.x1 = input;
    fb_suppressor.y2 = fb_suppressor.y1;
    fb_suppressor.y1 = output;
    
    // 限幅处理
    return CLAMP(output, -32768.0f, 32767.0f);
}

// 初始化自适应参数
void init_adaptive_params()
{
    log_info("Initializing feedback suppressor with sample rate: %d", __this->sample_rate);
    __this->suppress_freq = 3000;
    __this->q_factor = DEFAULT_Q;
    __this->threshold = powf(10.0f, THRESHOLD_DB/20.0f);
    __this->adapt_interval = ADAPT_INTERVAL;
    __this->sample_counter = 0;
    memset(__this->spectrum, 0, sizeof(__this->spectrum));

    // 初始化滤波器
    init_notch_filter(__this->suppress_freq, __this->sample_rate, __this->q_factor);
}

// 执行FFT分析
void analyze_spectrum(const s16 *samples, int num_samples)
{
    if(num_samples < FFT_SIZE) {
        return;
    }
    
    static int init_done = 0;
    static float hann_window[FFT_SIZE];

    if(!init_done) {
        for(int i=0; i<FFT_SIZE; i++) {
            hann_window[i] = 0.5f * (1 - cosf(2*M_PI*i/(FFT_SIZE-1)));
        }
        init_done = 1;
    }
    
    // 应用窗函数
    float windowed[FFT_SIZE];
    for(int i=0; i<FFT_SIZE; i++) {
        windowed[i] = (float)samples[i] * hann_window[i];
    }
    
    // 执行FFT
    fft_execute(windowed, __this->spectrum, FFT_SIZE);
}

// 检测啸叫频率
static int detect_feedback_freq()
{
    int peak_bin = -1;
    float peak_val = 0.0f;
    float threshold = __this->threshold;
    
    // 分析可能的啸叫频率范围
    int start_bin = (int)(MIN_SUPPRESS_FREQ * FFT_SIZE / __this->sample_rate);
    int end_bin = (int)(MAX_SUPPRESS_FREQ * FFT_SIZE / __this->sample_rate);
    
    // 寻找最高峰值
    for(int i = start_bin; i <= end_bin; i++) {
        if(__this->spectrum[i] > peak_val) {
            peak_val = __this->spectrum[i];
            peak_bin = i;
        }
    }
    
    if(peak_bin == -1 || peak_val <= 0) {
        return -1;
    }
    
    // 转换为频率
    int detected_freq = peak_bin * __this->sample_rate / FFT_SIZE;
    
    // 检查是否超过阈值
    float peak_db = 20 * log10f(peak_val);
    if(peak_db < THRESHOLD_DB) {
        return -1;
    }
    
    // 检查是否为窄带峰值
    if(peak_bin > 0 && peak_bin < FFT_SIZE/2-1) {
        float left_val = __this->spectrum[peak_bin-1];
        float right_val = __this->spectrum[peak_bin+1];
        
        // 窄带峰值判断
        if(left_val < peak_val * 0.6f && right_val < peak_val * 0.6f) {
            log_info("Detected howling at %dHz, peak: %.2fdB", detected_freq, peak_db);
            return detected_freq;
        }
    }
    
    return -1;
}

// 自适应调整滤波器
int adapt_filter()
{
    int new_freq = detect_feedback_freq();
    
    if (new_freq <= 0) {
        return -1;
    }
    
    // 频率变化超过50Hz才调整
    if(abs(new_freq - __this->suppress_freq) > 50) {
        __this->suppress_freq = new_freq;
        
        // 根据啸叫强度动态调整Q值
        int peak_bin = new_freq * FFT_SIZE / __this->sample_rate;
        float peak_val = __this->spectrum[peak_bin];
        float peak_db = 20 * log10f(peak_val);
        
        // Q值随啸叫强度增加而增加
        __this->q_factor = DEFAULT_Q * (1.0f + (peak_db - THRESHOLD_DB) / 20.0f);
        __this->q_factor = CLAMP(__this->q_factor, 5.0f, 30.0f);
        
        init_notch_filter(__this->suppress_freq, __this->sample_rate, __this->q_factor);
        
        log_info("Current suppress freq: freq=%dHz, Q=%.1f, peak=%.2fdB", 
                __this->suppress_freq, __this->q_factor, peak_db);
        
        return 1; 
    }
    
    return 0;
}

#endif