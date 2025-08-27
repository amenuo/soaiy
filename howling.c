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
#include "jl_math/kiss_fft.h"  // 或其他FFT库
#include "howling.h"



// 初始化陷波滤波器
#ifdef FEEDBACK_SUPPRESSION_ENABLE

#if 1
#define log_info(x, ...)    printf("\n[Howling]>" x " \n", ## __VA_ARGS__)
#else
#define log_info(...)
#endif


struct recorder_hdl recorder_handler;
struct feedback_suppressor fb_suppressor;

#define __this (&recorder_handler)



// 陷波滤波器初始化
static void init_notch_filter(float freq, float sample_rate, float q)
{
    const float omega = 2 * M_PI * freq / sample_rate;
    const float alpha = sinf(omega) / (2 * q);

    fb_suppressor.coeff[0] = 1 + alpha;
    fb_suppressor.coeff[1] = -2 * cosf(omega);
    fb_suppressor.coeff[2] = 1 - alpha;
    
    // 归一化系数
    for(int i=0; i<=NOTCH_FILTER_ORDER; i++) {
        fb_suppressor.coeff[i] /= (1 + alpha);
    }
}

void fft_execute(const float *input, float *output, int size) {
    if(!input || !output || size <= 0) return;
    kiss_fft_cfg cfg = kiss_fft_alloc(FFT_SIZE, 0, NULL, NULL);
    if (!cfg) {
        log_info("FFT------fail\n");
        return;
    }
   kiss_fft_cpx *in = NULL;
   kiss_fft_cpx *out = NULL;

        
   in = (kiss_fft_cpx*)malloc(size * sizeof(kiss_fft_cpx));
   out = (kiss_fft_cpx*)malloc(size * sizeof(kiss_fft_cpx));
    
    // 准备输入数据（实部为采样值，虚部为0）
    for (int i = 0; i < size; i++) {
        in[i].r = input[i];
        in[i].i = 0;
    }
    
    // 执行 FFT
    kiss_fft(cfg, in, out);
    
    // 计算幅度谱（只需要前 size/2 个点）
    for (int i = 0; i < size/2; i++) {
        output[i] = sqrtf(out[i].r*out[i].r + out[i].i*out[i].i);
    }

    

    free(in);
    free(out);
    free(cfg);
}

// 实时处理函数
float feedback_cancellation(s16 sample)
{
    // 更新输入缓冲区
    fb_suppressor.x_buf[fb_suppressor.buf_pos] = sample;
    
    // 计算滤波输出
    float output = 0;
    for(int i=0; i<=NOTCH_FILTER_ORDER; i++){
        int idx = (fb_suppressor.buf_pos - i + NOTCH_FILTER_ORDER+1) % (NOTCH_FILTER_ORDER+1);
        output += fb_suppressor.coeff[i] * fb_suppressor.x_buf[idx];
    }
    
        // 更新输出缓冲区
    fb_suppressor.y_buf[fb_suppressor.buf_pos] = output;

    // 更新缓冲区位置
    fb_suppressor.buf_pos = (fb_suppressor.buf_pos + 1) % (NOTCH_FILTER_ORDER+1);
    //return output;
    // 限幅处理
    return CLAMP(output,  -32768.0f, 32767.0);
}

// // 初始化自适应参数
void init_adaptive_params()
{
    log_info("Initializing feedback suppressor with sample rate: %d\n", __this->sample_rate);
    __this->suppress_freq = 3000;
    __this->q_factor = DEFAULT_Q;
    __this->threshold = powf(10.0f, THRESHOLD_DB/20.0f); // 转换为线性值
    __this->adapt_interval = ADAPT_INTERVAL;
    __this->sample_counter = 0;
    memset(__this->spectrum, 0, sizeof(__this->spectrum));

    fb_suppressor.buf_pos = 0;
    
    init_notch_filter(__this->suppress_freq, __this->sample_rate, __this->q_factor);
}

// 执行FFT分析
void analyze_spectrum(const s16 *samples, int num_samples)
{
        // 检查是否有足够样本
    if(num_samples < FFT_SIZE) {
        // printf("analyze_spectrum------------\n");
            return;
    }
    static int init_done = 0;
    static float hann_window[FFT_SIZE];

    if(!init_done) {
    // 在初始化时计算
        for(int i=0; i<FFT_SIZE; i++) {
            hann_window[i] = 0.5f * (1 - cosf(2*M_PI*i/(FFT_SIZE-1)));
        }
        init_done = 1;
    }
        // 应用窗函数
        float windowed[FFT_SIZE];
        for(int i=0; i<FFT_SIZE; i++) {
            windowed[i] = samples[i] * hann_window[i];
        }
    
    // 执行FFT (需要调用FFT库)
    fft_execute(windowed, __this->spectrum, FFT_SIZE);
}

// 
static int detect_narrow_peak(float *spectrum, int start_bin, int end_bin, float threshold,int *out_idx, float *out_val)
{
    int peak_idx = -1;
    float max_val = 0;

    for (int i = start_bin; i <= end_bin; i++) {
        if (spectrum[i] > threshold && spectrum[i] > max_val) {
            max_val = spectrum[i];
            peak_idx = i;
        }
    }

    if (peak_idx == -1) return -1;

    // 判断是否为尖峰：左右 bin 的能量显著低于峰值
    if (spectrum[peak_idx - 1] < max_val * 0.6f &&
        spectrum[peak_idx + 1] < max_val * 0.6f) {
        log_info("detect_narrow_peak>>>>>>>peak_idx - 1:%d peak_idx + 1:%d max_val:%.2f",spectrum[peak_idx - 1],spectrum[peak_idx + 1],max_val * 0.6f);
        //return peak_idx;
        *out_idx = peak_idx;
        *out_val = max_val;
        return true;
        
    }

    return -1; // 不是窄带尖峰
}


// // 检测啸叫频率
// static int detect_feedback_freq()
// {
//     int peak_bin = -1;
//     float peak_val = 0.0f;
//     float threshold = __this->threshold; // 使用模块内阈值（线性值）
    
//     // 只分析可能的啸叫频率范围(1kHz-5kHz)
//     int start_bin = (int)(MIN_SUPPRESS_FREQ * FFT_SIZE / __this->sample_rate);
//     int end_bin = (int)(MAX_SUPPRESS_FREQ * FFT_SIZE / __this->sample_rate);
//         // 检测窄带尖峰
//     detect_narrow_peak(__this->spectrum, start_bin, end_bin, threshold,  &peak_bin, &peak_val);
        
//         // for(int i=start_bin; i<=end_bin; i++) {
//         //     if(__this->spectrum[i] > peak_val) {
//         //         peak_val = __this->spectrum[i];
//         //         peak_bin = i;
//         //     }
//         // }
        
//     if (peak_val <=0) {
        
//         log_info("detect_narrow_peak>>>>>>>>>>>>>>>>undetected ");
//         return -1; // 未检测到有效啸叫       
        
//     }
//     // 转换为频率
//     int detected_freq = peak_bin *( __this->sample_rate / FFT_SIZE);
//     // // 如需返回整数频率
//     // int detected_freq = (int)(detected_freq_f + 0.5f);  // 四舍五入
    
//     // 检查是否超过阈值        
//         float peak_db = 20 * log10f(peak_val);
//         if(peak_db < THRESHOLD_DB) {
//              log_info("detect_narrow_peak>>>>>>>>>>>>>>>>is_too_low ");
//             return -1; // 未检测到啸叫
//             log_info("=== @@@@@@@@@@@@ ===");  // 用于验证           
//         }

//     log_info("detect_feedback_freq***peak_db>>>>>>>>>>>>>>>> %.2f\n",peak_db);
//         // 可选：再次确认频率在允许范围内
//     if (detected_freq < MIN_SUPPRESS_FREQ || detected_freq > MAX_SUPPRESS_FREQ) {
//         return -1;
//     }
//     log_info("detect_feedback_freq>>>>>>>>>>>>>>>> %d\n",detected_freq);
//     return detected_freq;
// }

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
        return -1; // 未检测到有效峰值
    }
    
    // 转换为频率
    int detected_freq = peak_bin * (__this->sample_rate / FFT_SIZE);
    
    // 检查是否超过阈值
    float peak_db = 20 * log10f(peak_val);
    if(peak_db < THRESHOLD_DB) {
        return -1; // 能量不足，不是啸叫
    }
    
    // 检查是否为窄带峰值（啸叫特征）
    if(peak_bin > 0 && peak_bin < FFT_SIZE/2-1) {
        float left_val = __this->spectrum[peak_bin-1];
        float right_val = __this->spectrum[peak_bin+1];
        
        // 窄带峰值判断：相邻频点能量显著低于峰值
        if(left_val < peak_val * 0.7f && right_val < peak_val * 0.7f) {
            log_info("Detected howling at %dHz, peak: %.2fdB\n", detected_freq, peak_db);
            return detected_freq;
        }
    }
    
    return -1; // 不是窄带峰值
}


// 添加此函数来早期检测内存问题
#if 0
static void check_memory() {
    extern unsigned int _free_heap_size(void);
    printf("剩余堆内存: %u 字节\n", _free_heap_size());
    
    if(_free_heap_size() < 4096) { // 设定一个阈值
        printf("警告: 堆内存不足!\n");
    }
}
#endif

// 自适应调整滤波器
 int  adapt_filter()
{
    int new_freq = detect_feedback_freq();
    
        // 首先检查是否检测到有效频率
    if (new_freq <= 0) {
        log_info("No valid feedback frequency detected");
        return -1;
    }
    
    if(abs(new_freq - __this->suppress_freq) > 50) {
        // 频率变化超过100Hz才调整，避免频繁抖动
        __this->suppress_freq = new_freq;
        
        // 根据啸叫强度动态调整Q值
        float peak_val = __this->spectrum[new_freq * FFT_SIZE / __this->sample_rate];
        float peak_db = 20 * log10f(peak_val);
        __this->q_factor = DEFAULT_Q * (1 + (peak_db - THRESHOLD_DB)/20.0f);
        __this->q_factor = CLAMP(__this->q_factor, 1.0f, 5.0f);
        
        init_notch_filter(__this->suppress_freq, __this->sample_rate, __this->q_factor);
        
       log_info("Current suppress freq: freq=%dHz, Q=%.1f, peak=%.2fdB\n", 
              __this->suppress_freq, __this->q_factor, peak_db);

       return 1; 
    }

    return 0;
    //check_memory();
}


#endif
