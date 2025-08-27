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


#define FEEDBACK_SUPPRESSION_ENABLE    // 算法总开关


#ifdef FEEDBACK_SUPPRESSION_ENABLE

// 结构体声明
struct feedback_suppressor;
struct recorder_hdl;

// 函数声明
void analyze_spectrum(const int16_t *samples, int num_samples);
float feedback_cancellation(int16_t sample);
void init_adaptive_params();
int adapt_filter();

#define MAX_SUPPRESSORS 3           // 最多同时抑制3个频点
#define NOTCH_FILTER_ORDER 4             // 陷波滤波器阶数
#define FFT_SIZE 128   // 160
#define MAX_SUPPRESS_FREQ 5500  // 最大抑制频率
#define MIN_SUPPRESS_FREQ 1500  // 最小抑制频率
#define DEFAULT_Q 2.2f         // 默认Q值
#define THRESHOLD_DB -35.0f     // 啸叫检测阈值(dB)
#define ADAPT_INTERVAL 16000    // 每16000样本调整一次(约1秒@16kHz)
//#define M_PI 3.141592653589793238462643383279502884197169399375105820974944
#define CLAMP(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#endif

#ifdef FEEDBACK_SUPPRESSION_ENABLE
struct feedback_suppressor {
    float coeff[NOTCH_FILTER_ORDER+1];    // 滤波器系数
    float x_buf[NOTCH_FILTER_ORDER+1];    // 输入缓冲区
    float y_buf[NOTCH_FILTER_ORDER+1];    // 输出缓冲区
    int buf_pos;                          // 缓冲区位置
    float target_freq;                    // 目标抑制频率
    //int suppress_freq;                    // 当前抑制频率
    //float q_factor;                       // Q值
    float freq_current;     // 当前滤波频率
    float freq_target;
    float q_target;         // 当前目标Q
    float q_current;        // 当前滤波Q
    float b0,b1,b2,a1,a2;
    float x1,x2;
    float y1,y2;
 
    
};
#endif

struct recorder_hdl {
    FILE *fp;
    FILE *rec_fp;//录音专用文件句柄
    struct server *enc_server;
    struct server *enc_server_rec; //录音专用服务
    struct server *dec_server;    
    void *cache_buf;
    cbuffer_t save_cbuf;
    OS_SEM w_sem;
    OS_SEM r_sem;
    volatile u8 run_flag;
    u8 volume;
    u8 gain;
    u8 channel;
    u8 direct;
    u8 recorder_flag;      //录音标记
    //u8 rec_key_times;        //按键次数
    const char *sample_source;
    int sample_rate;
#ifdef FEEDBACK_SUPPRESSION_ENABLE
    u8 feedback_suppress_en;  // 啸叫抑制使能标志
    int suppress_freq;        // 当前抑制频率
    float q_factor;          // Q值
    float threshold;         // 啸叫检测阈值
    int adapt_interval;      // 自适应调整间隔(样本数)
    int sample_counter;      // 样本计数器
    float spectrum[FFT_SIZE/2]; // 频谱分析缓冲区
#endif
#ifdef CONFIG_SPECTRUM_FFT_EFFECT_ENABLE
    void *work_buf;
    s16 *out_buf;
    u32 out_buf_size;
    u32 offset;
    u16 show_timer_id;
#endif
};
