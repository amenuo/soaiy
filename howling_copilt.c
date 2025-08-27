/*
@file: howling.c
@brief: 多频点啸叫检测与抑制（优化版）
@author: kang jin,
@date: 2025/08/23
*/

#include <math.h>
#include <string.h>
#include "jl_math/kiss_fft.h"
#include "howling.h"

// #define MAX_SUPPRESSORS 3           // 最多同时抑制3个频点
// #define NOTCH_FILTER_ORDER 2        // 陷波器阶数
// #define FFT_SIZE 512                // FFT长度
// #define DEFAULT_Q 3.0f              // 默认Q值
// #define THRESHOLD_DB -35.0f         // 啸叫判定阈值
// #define MIN_SUPPRESS_FREQ 1000      // 最低检测频率
// #define MAX_SUPPRESS_FREQ 5000      // 最高检测频率

#define CLAMP(val, min, max) ( (val)<(min)?(min):((val)>(max)?(max):(val)) )
#define true 1
#define false 0

// struct feedback_suppressor {
//     float coeff[NOTCH_FILTER_ORDER+1];
//     float x_buf[NOTCH_FILTER_ORDER+1];
//     float y_buf[NOTCH_FILTER_ORDER+1];
//     int   buf_pos;
//     float freq_target;      // 当前目标频率
//     float freq_current;     // 当前滤波频率
//     float q_target;         // 当前目标Q
//     float q_current;        // 当前滤波Q
// };

// struct recorder_hdl {
//     int sample_rate;
//     float spectrum[FFT_SIZE];
//    // int sample_counter;
//     //int adapt_interval;
//     //float threshold;
//     //int feedback_suppress_en;
// };

// 全局变量
struct recorder_hdl recorder_handler;
struct feedback_suppressor fb_suppressors[MAX_SUPPRESSORS];
#define __this (&recorder_handler)

// ----------- FFT 优化 ----------
static kiss_fft_cfg fft_cfg = NULL;
static float hann_window[FFT_SIZE];
static int fft_init_done = 0;

// 初始化FFT和窗函数
static void fft_global_init() {
    if (!fft_init_done) {
        fft_cfg = kiss_fft_alloc(FFT_SIZE, 0, NULL, NULL);
        for (int i = 0; i < FFT_SIZE; i++) {
            hann_window[i] = 0.5f * (1 - cosf(2*M_PI*i/(FFT_SIZE-1)));
        }
        fft_init_done = 1;
    }
}

// FFT 执行
void fft_execute(const float *input, float *output, int size) {
    if (!input || !output || size != FFT_SIZE) return;
    fft_global_init();
    kiss_fft_cpx in[FFT_SIZE], out[FFT_SIZE];
    for (int i = 0; i < size; i++) {
        in[i].r = input[i];
        in[i].i = 0;
    }
    kiss_fft(fft_cfg, in, out);
    for (int i = 0; i < size/2; i++) {
        output[i] = sqrtf(out[i].r*out[i].r + out[i].i*out[i].i);
    }
}

// ---------- 多频点峰值检测 ----------
static int multi_peak_detect(float *spectrum, int sample_rate, int *freqs, float *peaks) {
    int start_bin = (int)(MIN_SUPPRESS_FREQ * FFT_SIZE / sample_rate);
    int end_bin = (int)(MAX_SUPPRESS_FREQ * FFT_SIZE / sample_rate);
    float threshold = __this->threshold;
    int found = 0;

    for (int i = start_bin+1; i < end_bin-1 && found < MAX_SUPPRESSORS; i++) {
        float val = spectrum[i];
        if (val > threshold) {
            float left = spectrum[i-1], right = spectrum[i+1];
            if (val > left*1.4f && val > right*1.4f && val > spectrum[i-2]*1.2f && val > spectrum[i+2]*1.2f) {
                int freq = i * (sample_rate / FFT_SIZE);
                float peak_db = 20 * log10f(val);
                if (freq >= MIN_SUPPRESS_FREQ && freq <= MAX_SUPPRESS_FREQ && peak_db > THRESHOLD_DB) {
                    freqs[found] = freq;
                    peaks[found] = peak_db;
                    found++;
                    i += 2; // 跳过相邻峰，避免重复
                }
            }
        }
    }
    return found;
}

// ----------- 陷波滤波器 ----------
static void notch_filter_param(struct feedback_suppressor *sup, float freq, float sample_rate, float q) {
    // Bilinear变换陷波器系数
    const float omega = 2 * M_PI * freq / sample_rate;
    const float alpha = sinf(omega) / (2 * q);

    float b0 = 1;
    float b1 = -2 * cosf(omega);
    float b2 = 1;
    float a0 = 1 + alpha;
    float a1 = -2 * cosf(omega);
    float a2 = 1 - alpha;
    sup->coeff[0] = b0 / a0;
    sup->coeff[1] = b1 / a0;
    sup->coeff[2] = b2 / a0;
    sup->freq_current = freq;
    sup->q_current = q;
}

// 平滑参数过渡
static void update_notch_target(struct feedback_suppressor *sup, float freq, float q) {
    sup->freq_target = freq;
    sup->q_target = q;
}

static void notch_filter_smooth(struct feedback_suppressor *sup, float sample_rate) {
    // freq/q平滑过渡
    float step_f = 10.0f, step_q = 0.1f;
    if (fabs(sup->freq_current - sup->freq_target) > step_f)
        sup->freq_current += (sup->freq_target > sup->freq_current ? step_f : -step_f);
    else
        sup->freq_current = sup->freq_target;

    if (fabs(sup->q_current - sup->q_target) > step_q)
        sup->q_current += (sup->q_target > sup->q_current ? step_q : -step_q);
    else
        sup->q_current = sup->q_target;

    notch_filter_param(sup, sup->freq_current, sample_rate, sup->q_current);
}

// 对单通道样本做多陷波处理
float feedback_cancellation(s16 sample) {
    float out = sample;
    for (int k = 0; k < MAX_SUPPRESSORS; k++) {
        struct feedback_suppressor *sup = &fb_suppressors[k];
        sup->x_buf[sup->buf_pos] = out;
        float y = 0;
        for (int i = 0; i <= NOTCH_FILTER_ORDER; i++) {
            int idx = (sup->buf_pos - i + NOTCH_FILTER_ORDER+1) % (NOTCH_FILTER_ORDER+1);
            y += sup->coeff[i] * sup->x_buf[idx];
        }
        sup->y_buf[sup->buf_pos] = y;
        sup->buf_pos = (sup->buf_pos + 1) % (NOTCH_FILTER_ORDER+1);
        // 平滑参数
        notch_filter_smooth(sup, __this->sample_rate);
        out = y;
    }
    return CLAMP(out, -32768.0f, 32767.0f);
}

// ---------- 初始化与分析 ----------
void init_adaptive_params() {
    __this->sample_rate = 16000; // 可由外部配置
    __this->threshold = powf(10.0f, THRESHOLD_DB/20.0f);
    __this->adapt_interval = FFT_SIZE * 2;
    __this->sample_counter = 0;
    memset(__this->spectrum, 0, sizeof(__this->spectrum));

    for (int i = 0; i < MAX_SUPPRESSORS; i++) {
        memset(&fb_suppressors[i], 0, sizeof(struct feedback_suppressor));
        fb_suppressors[i].freq_current = 0;
        fb_suppressors[i].freq_target = 0;
        fb_suppressors[i].q_current = DEFAULT_Q;
        fb_suppressors[i].q_target = DEFAULT_Q;
        notch_filter_param(&fb_suppressors[i], 0, __this->sample_rate, DEFAULT_Q);
    }
}

// FFT分析入口
void analyze_spectrum(const s16 *samples, int num_samples) {
    if (num_samples < FFT_SIZE) return;
    fft_global_init();
    float windowed[FFT_SIZE];
    for (int i = 0; i < FFT_SIZE; i++)
        windowed[i] = samples[i] * hann_window[i];
    fft_execute(windowed, __this->spectrum, FFT_SIZE);
}

// ---------- 多频点自适应调整 ----------
int adapt_filter() {
    int freqs[MAX_SUPPRESSORS] = {0};
    float peaks[MAX_SUPPRESSORS] = {0};
    int found = multi_peak_detect(__this->spectrum, __this->sample_rate, freqs, peaks);

    for (int i = 0; i < MAX_SUPPRESSORS; i++) {
        if (i < found && freqs[i] > 0) {
            float q = DEFAULT_Q * (1 + (peaks[i] - THRESHOLD_DB)/20.0f);
            q = CLAMP(q, 1.0f, 5.0f);
            update_notch_target(&fb_suppressors[i], freqs[i], q);
        } else {
            // 未检测到则关闭陷波器
            update_notch_target(&fb_suppressors[i], 0, DEFAULT_Q);
        }
    }
    return found;
}
