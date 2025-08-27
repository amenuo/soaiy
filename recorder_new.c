/*
 这个文件是增加了杰理提供的文件基础上，增加了啸叫抑制算法。啸叫抑制算法起到作用。
 @author: kangjin
 @date: 2025/07/15 
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
#include "event/device_event.h"
#include "event/net_event.h"
#include "app_power_manage.h"
#include "storage_device.h"
#include "fs/fs.h"
#include "system/timer.h"
#include "media/spectrum/SpectrumShow_api.h"
#include "device/gpio.h"
#include "asm/gpio.h"
#include "jl_math/kiss_fft.h"  // 或其他FFT库
#include "howling.h"
#include "app_music.h"
#include "action.h"

#ifdef CONFIG_RECORDER_MODE_ENABLE
#if 1
#define log_info(x, ...)    printf("\n[Recorder_new]>" x " \n", ## __VA_ARGS__)
#else
#define log_info(...)
#endif


#define POST_TASK_NAME  "ui_demo"

#define WS300_EN_IO IO_PORTH_06
#define REC_EN_IO   IO_PORTA_10

#define CONFIG_STORE_VOLUME
#define VOLUME_STEP         5
#define GAIN_STEP           5
#define MIN_VOLUME_VALUE	5
#define MAX_VOLUME_VALUE	100
#define INIT_VOLUME_VALUE   20


extern void analyze_spectrum(const s16 *samples, int num_samples);
extern float feedback_cancellation(s16 sample);
extern void init_adaptive_params();
extern int adapt_filter();
extern struct recorder_hdl recorder_handler;
extern struct feedback_suppressor fb_suppressor;
extern void open_recorder();
extern void ui_play_wifi_show(u8 flag);

//static struct recorder_hdl recorder_handler;
#define __this (&recorder_handler)

static void ws300_en(){
    // gpio_direction_output(WS300_EN_IO,1);
    // gpio_set_pull_up(WS300_EN_IO,1);
    // gpio_set_pull_down(WS300_EN_IO,0);

}
static void ws300_en_close(){

   // gpio_direction_output(WS300_EN_IO,0);
}

static void rec_en(){
    gpio_direction_output(REC_EN_IO,1);
    gpio_set_pull_up(REC_EN_IO,1);
    gpio_set_pull_down(REC_EN_IO,0);
}

static void rec_en_close(){
    gpio_direction_output(REC_EN_IO,0);
}




//AUDIO ADC支持的采样率
static const u16 sample_rate_table[] = {
    8000,
    11025,
    12000,
    16000,
    22050,
    24000,
    32000,
    44100,
    48000,
};



#ifdef CONFIG_SPECTRUM_FFT_EFFECT_ENABLE
static void recorder_spectrum_fft_show(void *p)
{
    if (__this->work_buf) {
        short *db_data = getSpectrumValue(__this->work_buf);
        int num = getSpectrumNum(__this->work_buf);
        if (db_data && num > 0) {
            for (int i = 0; i < num; i++) {
                //输出db_num个 db值
                printf("db_data db[%d] %d\n", i, db_data[i]);
            }
        }
    }
}
#endif
#if 0
void adapt_filter_task_init(void)
{ 
    task_create(adapt_filter, 0, "adapt_filter_task");
}
#endif
//编码器输出PCM数据
static int recorder_vfs_fwrite(void *file, void *data, u32 len)
{
//    put_buf(data,len);

    cbuffer_t *cbuf = (cbuffer_t *)file;
    if (0 == cbuf_write(cbuf, data, len)) {
        //上层buf写不进去时清空一下，避免出现声音滞后的情况
        cbuf_clear(cbuf);
    }
    os_sem_set(&__this->r_sem, 0);
    os_sem_post(&__this->r_sem);


#if FEEDBACK_SUPPRESSION_ENABLE_1
    if(__this->feedback_suppress_en){
        log_info("Processing %d bytes with feedback suppression\n", len);

        s16 *pcm = (s16 *)data;
        int samples = len / 2; // 16-bit样本
        
        // 自适应处理
        __this->sample_counter += samples;
        if(__this->sample_counter >= __this->adapt_interval) {
            analyze_spectrum(pcm, samples);
            adapt_filter();           
            __this->sample_counter = 0;
        }
        
        // 应用啸叫抑制
        for(int i=0; i<samples; i++){
            pcm[i] = (s16)feedback_cancellation(pcm[i]);
        }
    }
#endif

#ifdef CONFIG_SPECTRUM_FFT_EFFECT_ENABLE
    u32 in_remain = len, tlen = 0;

    while (in_remain) {
        if (__this->offset < __this->out_buf_size) {
            tlen = __this->out_buf_size - __this->offset;
            if (tlen > in_remain) {
                tlen = in_remain;
            }
            memcpy((u8 *)__this->out_buf + __this->offset, (u8 *)data + (len - in_remain), tlen);
            __this->offset += tlen;
            in_remain -= tlen;
            if (in_remain && (__this->offset != __this->out_buf_size)) {
                continue;
            }
        }
        if (__this->offset == __this->out_buf_size) {
            __this->offset = 0;
            SpectrumShowRun(__this->work_buf, __this->out_buf, 512);
        }
    }
#endif

    //此回调返回0录音就会自动停止
    return len;
}

//解码器读取PCM数据
static int recorder_vfs_fread(void *file, void *data, u32 len)
{
    cbuffer_t *cbuf = (cbuffer_t *)file;
            
    u32 rlen;

    do {
        rlen = cbuf_get_data_size(cbuf);
        rlen = rlen > len ? len : rlen;
        if (cbuf_read(cbuf, data, rlen) > 0) {
            len = rlen;
            break;
        }
        //此处等待信号量是为了防止解码器因为读不到数而一直空转
        os_sem_pend(&__this->r_sem, 0);
        if (!__this->run_flag) {
            return 0;
        }
    } while (__this->run_flag);

    //返回成功读取的字节数
    return len;
}

static int recorder_vfs_fclose(void *file)
{
    return 0;
}

static int recorder_vfs_flen(void *file)
{
    return 0;
}

static const struct audio_vfs_ops recorder_vfs_ops = {
    .fwrite = recorder_vfs_fwrite,
    .fread  = recorder_vfs_fread,
    .fclose = recorder_vfs_fclose,
    .flen   = recorder_vfs_flen,
};

static int enc_server_vfs_fwrite(void *file, void *data, u32 len)
{
    //不要做长时间堵塞操作
    return fwrite(data, len, 1, __this->rec_fp);
}

static int enc_server_vfs_fclose(void *file)
{
    return 0;
}

static const struct audio_vfs_ops enc_server_vfs_ops = {
    .fwrite = enc_server_vfs_fwrite,
    .fclose = enc_server_vfs_fclose,
};


// 编码虚拟源输入函数，用于recorder_play_to_dac模式
static u32 recorder_read_input(u8 *buf, u32 len)
{
    u32 rlen = 0;
    
    // 检查运行标志和缓冲区状态
    // if (!__this->run_flag || !__this->cache_buf) {
    //     return 0;
    // }

    // 获取可读数据长度
    rlen = cbuf_get_data_size(&__this->save_cbuf);
    log_info(">>>>>>>>>>>>>>>>>>>recorder_read_input: rlen %d\n", rlen);
    if (rlen == 0) {
        // 无数据可读
        return 0;
    }

    // 不超过请求长度
    rlen = MIN(rlen, len);
    
    // 从环形缓冲区读取数据
    rlen = cbuf_read(&__this->save_cbuf, buf, rlen);
    
    // 唤醒写入端（如果有等待的写入者）
    os_sem_set(&__this->w_sem, 0);
    os_sem_post(&__this->w_sem);

    return rlen;
}

int recorder_file_close(void){
    
      union audio_req req = {0};

      //if (!__this->rec_fp) return 0; // 防止重复关闭
      if (!__this->fp) return 0; // 防止重复关闭


    // 关闭录音专用服务器
    if (__this->enc_server_rec) {
        union audio_req req = {0};
        req.enc.cmd = AUDIO_ENC_CLOSE;
        server_request(__this->enc_server_rec, AUDIO_REQ_ENC, &req);
        //server_close(__this->enc_server_rec);
        //__this->enc_server_rec = NULL;
    }
        
    // if (__this->rec_fp) {
    //     int wlen;
    //      wlen = flen(__this->rec_fp);
    //     fclose(__this->rec_fp);
    //     __this->rec_fp = NULL;
    //    log_info("recorder_file_close**********write file len: %d \n", wlen);
    // }

        if (__this->fp) {
        int wlen;
         wlen = flen(__this->fp);
        fclose(__this->fp);
        __this->fp = NULL;
       log_info("recorder_file_close**********write file len: %d \n", wlen);
    }

} 

static int colse_enc(void){
    
    union audio_req req = {0};
        if (__this->enc_server) {
        req.enc.cmd = AUDIO_ENC_CLOSE;
        server_request(__this->enc_server, AUDIO_REQ_ENC, &req);
    }

}

static int colse_dec(void){
    
    union audio_req req = {0};
    if (__this->dec_server) {
        req.dec.cmd = AUDIO_DEC_STOP;
        server_request(__this->dec_server, AUDIO_REQ_DEC, &req);
    }

}


int recorder_close(void)
{
    union audio_req req = {0};

    if (!__this->run_flag) {
        return 0;
    }

    log_d("----------recorder close----------\n");

    __this->run_flag = 0;

    os_sem_post(&__this->w_sem);
    os_sem_post(&__this->r_sem);

    if (__this->enc_server) {
        req.enc.cmd = AUDIO_ENC_CLOSE;
        server_request(__this->enc_server, AUDIO_REQ_ENC, &req);
    }

    if (__this->dec_server) {
        req.dec.cmd = AUDIO_DEC_STOP;
        server_request(__this->dec_server, AUDIO_REQ_DEC, &req);
    }

    if (__this->cache_buf) {
        free(__this->cache_buf);
        __this->cache_buf = NULL;
    }

    if (__this->fp) {
        int wlen;
         wlen = flen(__this->fp);
        fclose(__this->fp);
        __this->fp = NULL;
       log_info("write file len: %d \n", wlen);            
           

    }


#ifdef CONFIG_SPECTRUM_FFT_EFFECT_ENABLE
    if (__this->work_buf) {
        free(__this->work_buf);
        __this->work_buf = NULL;
    }
    if (__this->out_buf) {
        free(__this->out_buf);
        __this->out_buf = NULL;
    }
    if (__this->show_timer_id) {
        sys_timeout_del(__this->show_timer_id);
        __this->show_timer_id = 0;
    }
#endif  
    return 0;
}

static void enc_server_event_handler(void *priv, int argc, int *argv)
{
    switch (argv[0]) {
    case AUDIO_SERVER_EVENT_ERR:
    case AUDIO_SERVER_EVENT_END:
        recorder_close();
        break;
    case AUDIO_SERVER_EVENT_SPEAK_START:
        log_i("speak start ! \n");
        break;
    case AUDIO_SERVER_EVENT_SPEAK_STOP:
        log_i("speak stop ! \n");
        break;
    default:
        break;
    }
}
// 录音文件名称
static char* get_file_name(void){

    static char file_name[100] = {0};
    char time_str[64] = {0};   
    char month_dir[32] = {0}; //创建月目录
    char full_dir_path[100] = {0}; //创建路径
    u8 dir_len = 0;
    
    //获取当前时间
    struct tm timeinfo = {0};
    time_t timestamp = time(NULL) + 28800;
    localtime_r(&timestamp, &timeinfo);

    // 1. 创建年月目录(如: /root/2025-06/)
    strftime(month_dir, sizeof(month_dir), "%Y-%m", &timeinfo); 
    snprintf(full_dir_path, sizeof(full_dir_path), "%s%s", CONFIG_ROOT_PATH, month_dir);
    
  
    strcpy(time_str, full_dir_path );   
    strcat(time_str, "/\\U"); // 增加/扛保持目录结构    
    dir_len = strlen(time_str);
    //strftime(time_str + dir_len, sizeof(time_str) - dir_len, "%Y-%m-%dT%H-%M-%S.", &timeinfo);
    strftime(time_str + dir_len, sizeof(time_str) - dir_len, "%d-%H-%M.", &timeinfo);
    strcat(time_str, "mp3");    
    log_info("recorder file name : %s\n", time_str);
    memcpy(file_name, time_str, dir_len);

    for (u8 i = 0; i < strlen(time_str) - dir_len; ++i) {
        file_name[dir_len + i * 2] = time_str[dir_len + i];
    }
     //log_info("get_file_recorder file name : %s\n", file_name);
     return file_name;

}
//将MIC的数字信号采集后推到DAC播放
//注意：如果需要播放两路MIC，DAC分别对应的是DACL和DACR，要留意芯片封装是否有DACR引脚出来，
//      而且要使能DAC的双通道输出，DAC如果采用差分输出方式也只会听到第一路MIC的声音
static int recorder_play_to_dac(int sample_rate, u8 channel)
{
    int err;
    union audio_req req = {0};
    //通过切换后关闭指示灯   

    log_d("----------recorder_play_to_dac----------\n");

    if (channel > 2) {
        channel = 2;
    }
    __this->cache_buf = malloc(sample_rate * channel); //上层缓冲buf缓冲0.5秒的数据，缓冲太大听感上会有延迟
    if (__this->cache_buf == NULL) {
        return -1;
    }
    cbuf_init(&__this->save_cbuf, __this->cache_buf, sample_rate * channel);

    os_sem_create(&__this->w_sem, 0);
    os_sem_create(&__this->r_sem, 0);

    __this->run_flag = 1;

    /****************打开解码DAC器*******************/
    req.dec.cmd             = AUDIO_DEC_OPEN;
    req.dec.volume          = __this->volume;
    req.dec.output_buf_len  = 4 * 1024;
    req.dec.channel         = channel;
    req.dec.sample_rate     = sample_rate;
    req.dec.vfs_ops         = &recorder_vfs_ops;
    req.dec.dec_type 		= "pcm";
    req.dec.sample_source   = CONFIG_AUDIO_DEC_PLAY_SOURCE;
    req.dec.file            = (FILE *)&__this->save_cbuf;
    /* req.dec.attr            = AUDIO_ATTR_LR_ADD; */          //左右声道数据合在一起,封装只有DACL但需要测试两个MIC时可以打开此功能

    err = server_request(__this->dec_server, AUDIO_REQ_DEC, &req);
    if (err) {
        goto __err;
    }

    req.dec.cmd = AUDIO_DEC_START;
    req.dec.attr = AUDIO_ATTR_NO_WAIT_READY;
    server_request(__this->dec_server, AUDIO_REQ_DEC, &req);

#ifdef CONFIG_SPECTRUM_FFT_EFFECT_ENABLE
    __this->work_buf = zalloc(getSpectrumShowBuf());
    if (!__this->work_buf) {
        goto __err1;
    }
    __this->offset = 0;
    __this->out_buf_size = 512 * 2 * channel;
    __this->out_buf = zalloc(__this->out_buf_size);
    if (!__this->out_buf) {
        free(__this->work_buf);
        __this->work_buf = NULL;
        goto __err1;
    }

    SpectrumShowInit(__this->work_buf, 0.9, 0.9,
                     sample_rate, channel, channel > 1 ? 2 : 0, JL_FFT_BASE);

    __this->show_timer_id = sys_timer_add(NULL, recorder_spectrum_fft_show, 1000);
#endif

    /****************打开编码器*******************/
    memset(&req, 0, sizeof(union audio_req));
   
    //获取录音文件名
    //char* file_name = get_file_name();

    //BIT(x)用来区分上层需要获取哪个通道的数据
    if (channel == 2) {
        req.enc.channel_bit_map = BIT(CONFIG_AUDIO_ADC_CHANNEL_L) | BIT(CONFIG_AUDIO_ADC_CHANNEL_R);
    } else {
        req.enc.channel_bit_map = BIT(CONFIG_AUDIO_ADC_CHANNEL_L);
    }
    req.enc.frame_size = sample_rate / 100 * 4 * channel;	//收集够多少字节PCM数据就回调一次fwrite
    req.enc.output_buf_len = req.enc.frame_size * 3; //底层缓冲buf至少设成3倍frame_size
    req.enc.cmd = AUDIO_ENC_OPEN;
    req.enc.channel = channel;
    req.enc.volume = __this->gain;
    req.enc.sample_rate = sample_rate;
    req.enc.format = "pcm";
    req.enc.sample_source = __this->sample_source;
    req.enc.vfs_ops = &recorder_vfs_ops;    
    req.enc.file = (FILE *)&__this->save_cbuf;
    if (channel == 1 && !strcmp(__this->sample_source, "mic") && (sample_rate == 8000 || sample_rate == 16000)) {
        //  req.enc.use_vad = 1; //打开VAD断句功能
        //  req.enc.dns_enable = 1; //打开降噪功能
        //  req.enc.vad_auto_refresh = 1; //VAD自动刷新
    }

    err = server_request(__this->enc_server, AUDIO_REQ_ENC, &req);
    if (err) {
        goto __err1;
    }

    return 0;

__err1:
    req.dec.cmd = AUDIO_DEC_STOP;
    server_request(__this->dec_server, AUDIO_REQ_DEC, &req);

__err:
    if (__this->cache_buf) {
        free(__this->cache_buf);
        __this->cache_buf = NULL;
    }

    __this->run_flag = 0;

    return -1;
}

//MIC或者LINEIN模拟直通到DAC，不需要软件参与
static int audio_adc_analog_direct_to_dac(int sample_rate, u8 channel)
{
    union audio_req req = {0};

    log_info("----------audio_adc_analog_direct_to_dac----------\n");

    __this->run_flag = 1;
    req.enc.cmd = AUDIO_ENC_OPEN;
    req.enc.channel = channel;
    req.enc.volume = __this->gain;
    req.enc.format = "pcm";
    req.enc.sample_source = __this->sample_source;
    req.enc.sample_rate = sample_rate;
    req.enc.direct2dac = 1;
    req.enc.high_gain = 1;
    if (channel == 4) {
        req.enc.channel_bit_map = 0x0f;
    } else if (channel == 2) {
        req.enc.channel_bit_map = BIT(CONFIG_AUDIO_ADC_CHANNEL_L) | BIT(CONFIG_AUDIO_ADC_CHANNEL_R);
    } else {
        req.enc.channel_bit_map = BIT(CONFIG_AUDIO_ADC_CHANNEL_L);
    }

    return server_request(__this->enc_server, AUDIO_REQ_ENC, &req);
}


//录音文件到SD卡
 int recorder_to_file(int sample_rate, u8 channel, const char *format)
{
    
    __this->enc_server_rec = server_open("audio_server", "enc");
    server_register_event_handler_to_task(__this->enc_server_rec, NULL, enc_server_event_handler, "app_core");
    union audio_req req = {0};

    __this->run_flag = 1;
    __this->direct = 0;

    // 获取录音文件名
    char* file_name = get_file_name();

    req.enc.cmd = AUDIO_ENC_OPEN;
    req.enc.channel = channel;
    req.enc.volume = 100;
    req.enc.frame_size = 8192;
    req.enc.output_buf_len = req.enc.frame_size * 10;
    req.enc.sample_rate = sample_rate;
    req.enc.format = format;   
    req.enc.sample_source = "mic";  
    req.enc.msec = 0 ;//CONFIG_AUDIO_RECORDER_DURATION;
    req.enc.file = __this->fp = fopen(file_name, "w+");
//    if (!strcmp(req.enc.format, "aac")) {
        req.enc.bitrate = 16000;  sample_rate * 4;
        req.enc.no_header = 1;
//    }
    /* req.enc.sample_depth = 24; //IIS支持采集24bit深度 */
    if (channel == 4) {
        req.enc.channel_bit_map = 0x0f;
    } else if (channel == 2) {
        req.enc.channel_bit_map = BIT(CONFIG_AUDIO_ADC_CHANNEL_L) | BIT(CONFIG_AUDIO_ADC_CHANNEL_R);
    } else {
        req.enc.channel_bit_map = BIT(CONFIG_AUDIO_ADC_CHANNEL_L);
    }

    //return server_request(__this->enc_server, AUDIO_REQ_ENC, &req);
    return server_request(__this->enc_server_rec, AUDIO_REQ_ENC, &req);
}
static int rec_to_file (void)
{
    union audio_req req = {0};
    char* file_name = get_file_name();

    req.enc.format = "mp3";   
    req.enc.sample_source = "mic";
    req.enc.msec = 0;
    req.enc.file = __this->fp = fopen(file_name, "w+");
    return server_request(__this->enc_server, AUDIO_REQ_ENC, &req);
}


static void recorder_play_pause(void)
{
    union audio_req req = {0};

    req.dec.cmd = AUDIO_DEC_PP;
    server_request(__this->dec_server, AUDIO_REQ_DEC, &req);

    req.enc.cmd = AUDIO_ENC_PP;
    server_request(__this->enc_server, AUDIO_REQ_ENC, &req);

    if (__this->cache_buf) {
        cbuf_clear(&__this->save_cbuf);
    }
}

//调整ADC的模拟增益
static int recorder_enc_gain_change(int step)
{
    union audio_req req = {0};

    int gain = __this->gain + step;
    if (gain < 0) {
        gain = 0;
    } else if (gain > 100) {
        gain = 100;
    }
    if (gain == __this->gain) {
        return -1;
    }
    __this->gain = gain;

    if (!__this->enc_server) {
        return -1;
    }

    log_info("set_enc_gain: %d\n", gain);

    req.enc.cmd     = AUDIO_ENC_SET_VOLUME;
    req.enc.volume  = gain;
    return server_request(__this->enc_server, AUDIO_REQ_ENC, &req);
}

//调整DAC的数字音量和模拟音量
static int recorder_dec_volume_change(int step)
{
    union audio_req req = {0};

    int volume = __this->volume + step;
    if (volume < MIN_VOLUME_VALUE) {
        volume = MIN_VOLUME_VALUE;
    } else if (volume > MAX_VOLUME_VALUE) {
        volume = MAX_VOLUME_VALUE;
    }
    if (volume == __this->volume) {
        return -1;
    }
    __this->volume = volume;

    if (!__this->dec_server) {
        return -1;
    }

    log_info("set_dec_volume: %d\n", volume);
    //os_taskq_post(POST_TASK_NAME, 2, UI_MSG_SET_VOLUME, volume);
    // os_taskq_post(POST_TASK_NAME, 2, UI_MSG_POWER_LOG, volume);

#ifdef CONFIG_STORE_VOLUME
   #if SYSCFG_WRITE
    syscfg_write(CFG_MUSIC_VOL, &__this->volume, sizeof(__this->volume));
   #endif
#endif

    req.dec.cmd     = AUDIO_DEC_SET_VOLUME;
    req.dec.volume  = volume;
    return server_request(__this->dec_server, AUDIO_REQ_DEC, &req);
}

/* 初始化录音模块*/
static int recorder_mode_init(void)
{
    log_info("recorder_play_main\n");
    memset(__this, 0, sizeof(struct recorder_hdl));

#ifdef CONFIG_STORE_VOLUME
    if (syscfg_read(CFG_MUSIC_VOL, &__this->volume, sizeof(__this->volume)) < 0 ||
        __this->volume < MIN_VOLUME_VALUE || __this->volume > MAX_VOLUME_VALUE) {
        __this->volume = INIT_VOLUME_VALUE;
    }
#else
    __this->volume = INIT_VOLUME_VALUE;
#endif
    if (__this->volume < 0 || __this->volume > MAX_VOLUME_VALUE) {
        __this->volume = INIT_VOLUME_VALUE;
    }

#if CONFIG_AUDIO_ENC_SAMPLE_SOURCE == AUDIO_ENC_SAMPLE_SOURCE_PLNK0
    __this->sample_source = "plnk0";
#elif CONFIG_AUDIO_ENC_SAMPLE_SOURCE == AUDIO_ENC_SAMPLE_SOURCE_PLNK1
    __this->sample_source = "plnk1";
#elif CONFIG_AUDIO_ENC_SAMPLE_SOURCE == AUDIO_ENC_SAMPLE_SOURCE_IIS0
    __this->sample_source = "iis0";
#elif CONFIG_AUDIO_ENC_SAMPLE_SOURCE == AUDIO_ENC_SAMPLE_SOURCE_IIS1
    __this->sample_source = "iis1";
#elif CONFIG_AUDIO_ENC_SAMPLE_SOURCE == AUDIO_ENC_SAMPLE_SOURCE_LINEIN
    __this->sample_source = "linein";
#else
    __this->sample_source = "mic";
#endif

    __this->channel = CONFIG_AUDIO_RECORDER_CHANNEL;
    __this->gain = CONFIG_AUDIO_ADC_GAIN;
    __this->sample_rate = CONFIG_AUDIO_RECORDER_SAMPLERATE;




#ifdef FEEDBACK_SUPPRESSION_ENABLE    
    // 初始化自适应啸叫抑制参数
    __this->feedback_suppress_en = 1;
    init_adaptive_params();
    //启动滤波算法
    //adapt_filter_task_init();
#endif
     // 初始化录音参数
     __this->recorder_flag =0 ;    
     __this->direct = 1;

     __this->enc_server = server_open("audio_server", "enc");
    server_register_event_handler_to_task(__this->enc_server, NULL, enc_server_event_handler, "app_core");

    __this->dec_server = server_open("audio_server", "dec");

    return recorder_play_to_dac(__this->sample_rate, __this->channel);

    
}

 void recorder_mode_exit(void)
{
    //recorder_close();
    server_close(__this->dec_server);
    __this->dec_server = NULL;
    server_close(__this->enc_server);
    __this->enc_server = NULL;

}

static void feedback_suppress_en(void){
     // 切换啸叫抑制开关            
    #ifdef FEEDBACK_SUPPRESSION_ENABLE
        __this->feedback_suppress_en = !__this->feedback_suppress_en; 
        if(__this->feedback_suppress_en){
            //rec_en();

        }else{

            //rec_en_close();
        }       
 
    #endif
    
}

void recorder_mode(u8 key)
{ 
    //  __this->recorder_flag =  !__this->recorder_flag;
    //  log_info("recorder_flag: %d\n", __this->recorder_flag);
      if(key){
            if (storage_device_ready()) {
            log_info("\n >>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);             
            recorder_to_file(__this->sample_rate, __this->channel, CONFIG_AUDIO_RECORDER_SAVE_FORMAT);
            open_recorder();            

            }
        }else{
            //  //recorder_close();
            //  __this->recorder_flag=0;
             recorder_file_close();            
                       
        }

}

static int recorder_key_click(struct key_event *key)
{
    int ret = true;

    switch (key->value) {
    case KEY_OK:
#if CONFIG_AUDIO_ENC_SAMPLE_SOURCE == AUDIO_ENC_SAMPLE_SOURCE_LINEIN || CONFIG_AUDIO_ENC_SAMPLE_SOURCE == AUDIO_ENC_SAMPLE_SOURCE_MIC
        if (__this->direct) {
            if (__this->run_flag) {
                log_info("run_flag: %d\n", __this->run_flag);
                recorder_close();
            } else {
                audio_adc_analog_direct_to_dac(__this->sample_rate, __this->channel);
            }
        } else {
            recorder_play_pause();
        }
#else
        recorder_play_pause();
#endif
       __this->direct = !__this->direct;
        break;
    case KEY_VOLUME_DEC:
        recorder_dec_volume_change(-VOLUME_STEP);               
        break;
    case KEY_VOLUME_INC:
        recorder_dec_volume_change(VOLUME_STEP);
        break;
    case KEY_UP:        
        break;
    case KEY_DOWN:       
        break;
    case KEY_MODE:
        // 应用层函数
        extern  ui_rec_mode(u8 mode);
        ui_rec_mode(0);

        break;
     
    default:
        ret = false;
        break;
    }

    return ret;
}

static int recorder_key_long(struct key_event *key)
{
    switch (key->value) {
#if CONFIG_AUDIO_ENC_SAMPLE_SOURCE == AUDIO_ENC_SAMPLE_SOURCE_LINEIN || CONFIG_AUDIO_ENC_SAMPLE_SOURCE == AUDIO_ENC_SAMPLE_SOURCE_MIC
    case KEY_OK:
//        recorder_close();
//        if (__this->direct) {
//            recorder_play_to_dac(__this->sample_rate, __this->channel);
//        } else {
//            audio_adc_analog_direct_to_dac(__this->sample_rate, __this->channel);
//        }
//        __this->direct = !__this->direct;
        break;
    case KEY_VOLUME_DEC:
        recorder_enc_gain_change(-GAIN_STEP);
        break;
    case KEY_VOLUME_INC:
        recorder_enc_gain_change(GAIN_STEP);
        break;
#endif
    case KEY_MODE:
	    feedback_suppress_en();
        break;
    default:
        break;
    }

    return true;
}

static int recorder_key_event_handler(struct key_event *key)
{
    switch (key->action) {
    case KEY_EVENT_CLICK:
        return recorder_key_click(key);
    case KEY_EVENT_LONG:
        return recorder_key_long(key);
    default:
        break;
    }

    return true;
}


int battery_per;
static int main_dev_event_handler(struct sys_event *sys_eve)
{
   struct device_event *device_eve = (struct device_event *)sys_eve->payload;
   
   if (sys_eve->from == DEVICE_EVENT_FROM_POWER){

        switch (device_eve->event){

            case POWER_EVENT_POWER_CHANGE:
            battery_per = sys_power_get_battery_persent(); 
            os_taskq_post(POST_TASK_NAME, 2, UI_MSG_POWER_LOG, battery_per);
            break;
            case POWER_EVENT_POWER_WARNING:
            app_play_tone_file ("mnt/sdfile/EXT_RESERVED/aupackres/tone/BtSucceed.mp3");
            break;
            case POWER_EVENT_POWER_LOW:
            app_play_tone_file ("mnt/sdfile/EXT_RESERVED/aupackres/tone/LowBatOff.mp3");
            break;
            case POWER_EVENT_POWER_AUTOOFF:
            app_play_tone_file ("mnt/sdfile/EXT_RESERVED/aupackres/tone/PowerOff.mp3");
            break;
            case POWER_EVENT_POWER_CHARGE:
            break;
            default:                 
            break;

        }

   }

}

static int net_event_handler(struct net_event *event)
{
    switch (event->event) {
    case NET_CONNECT_ASSOCIAT_FAIL:
         log_info("***************************NET_CONNECT_ASSOCIAT_FAIL\n");
          ui_play_wifi_show(0);
        break;
    case NET_EVENT_CONNECTED:
         log_info("***************************NET_EVENT_CONNECTED\n");
         ui_play_wifi_show(1);
        break;
    case NET_CONNECT_TIMEOUT_NOT_FOUND_SSID:
         log_info("***************************ET_CONNECT_TIMEOUT_NOT_FOUND_SSID\n");
         ui_play_wifi_show(0);
        break;
    default:
        break;
    }

    return false;
}

static int recorder_event_handler(struct application *app, struct sys_event *event)
{
    switch (event->type) {
    case SYS_KEY_EVENT:
        return recorder_key_event_handler((struct key_event *)event->payload);
        break;
    case SYS_DEVICE_EVENT:
        return main_dev_event_handler(event);
        break;
    case SYS_NET_EVENT:
        return net_event_handler((struct net_event *)event->payload);
        break;
    case SYS_BT_EVENT:
        break;
    default:
        return false;
    }
}

static int recorder_state_machine(struct application *app, enum app_state state, struct intent *it)
{
    switch (state) {
    case APP_STA_CREATE:
        break;
    case APP_STA_START:
        //  switch (it->action) {
        //     case ACTION_MUSIC_PLAY_MAIN:
        //         recorder_mode_init();
        //         break;
        //     case ACTION_MUSIC_PLAY_VOICE_PROMPT:
        //         // if ((!__this->bt_music_enable || is_in_config_network_state()) && !strcmp((const char *)it->data, "BtDisc.mp3")) {

        //         //     break;	//蓝牙关闭后不需要蓝牙断开提示音
        //         // }
        //         // if (!strcmp((const char *)it->data, "BtSucceed.mp3") && __this->dec_end_handler != (void *)app_music_shutdown) {
        //         //     if (__this->play_voice_prompt) {
        //         //         local_music_dec_ops.dec_stop(0);
        //         //         __this->play_voice_prompt = 0;
        //         //     }
        //         //     __this->bt_connecting = 1;
        //         // }
        //         // app_music_play_voice_prompt("BtClose.mp3", __this->dec_ops->dec_breakpoint);    //关闭蓝牙

        //         break;
        //  }
         recorder_mode_init();
        break;
    case APP_STA_PAUSE:
        break;
    case APP_STA_RESUME:
        break;
    case APP_STA_STOP:
//        recorder_mode_exit();
        break;
    case APP_STA_DESTROY:
        break;
    }

    return 0;
}

static const struct application_operation recorder_ops = {
    .state_machine  = recorder_state_machine,
    .event_handler 	= recorder_event_handler,
};

REGISTER_APPLICATION(recorder) = {
    .name 	= "recorder",
    .ops 	= &recorder_ops,
    .state  = APP_STA_DESTROY,
};

#include "server\audio_dev.h"
#include "video\video_ioctl.h"

static u8 __data[1 * 1024 * 1024];
static int offset;
cbuffer_t save_cbuf;
static u8 cache_buf[16 * 1024];
s16 buf[POINT_ADC / 2] sec(.sram);
static void audio_dev_enc_irq_handler(void *priv, u8 *data, int len)
{
 // log_info("Processing %d audio_dev_enc_irq_handler\n", len);
//   log_info("Feedback suppression %d\n", 
//               __this->feedback_suppress_en );
//    if((offset + len) < sizeof(__data)){
//    putchar('A');
//    printf("\n >>>>>>>>>>>>>>>>ADC len = %d\n",len);
//        memcpy(__data+offset,data,len);
//        offset += len;

//    }

//    put_buf(data,64);

#ifdef FEEDBACK_SUPPRESSION_ENABLE
    if(__this->feedback_suppress_en){
        int n = 0;
        //log_info("feedback_suppress_en_Processing %d bytes with feedback suppression\n", len);
        s16 *pcm = (s16 *)data;
        int samples = len / 2; // 16-bit样本
        //int samples = sizeof(buf) / 2; // 16-bit样本
        
        // 自适应处理
        __this->sample_counter += samples;
        if(__this->sample_counter >= __this->adapt_interval) {
            analyze_spectrum(pcm, samples);
             n = adapt_filter();           
            __this->sample_counter = 0;
            
        }
        
        // 应用啸叫抑制
        if( n > 0){
            //log_info(">>>>>>>>>>>nnnnnnnnnnnnnnn: %d\n", n);
            log_info(">>>>>>>>>>>start feedback_cancellation");   
            for(int i=0; i<sizeof(buf); i++){
                //float processed = feedback_cancellation(pcm[i*4+1]);
                // buf[i] = (s16)processed;  // 确保数据被写回
                buf[i] = feedback_cancellation(pcm[i*4+1]);                      
            
            }
        }else{
            for(int i = 0; i < sizeof(buf); i++){
                buf[i] = pcm[i*4+1];
            }
        }
    }
#else
    s16 *__data = (s16 *)data;

    for(int i = 0;i < sizeof(buf);i++){
        buf[i] = __data[i*4 + 1];//mic 0 mic1 mic2 mic3 0 1 2 3 4 5 6
    }
//    cbuf_write(&save_cbuf,buf,320);
#endif // 0


// #if 0
//     u8 buf[320];
//     for(int i = 0;i < 320;i++){
//         buf[i] = data[i*4 + 1];//mic 0 mic1 mic2 mic3 0 1 2 3 4 5 6
//         buf[i+1] = data[i*4 + 2];//mic 0 mic1 mic2 mic3 0 1 2 3 4 5 6
//     }
//     cbuf_write(&save_cbuf,buf,320);
// #else
//     s16 *__data = (s16 *)data;

//     for(int i = 0;i < sizeof(buf);i++){
//         buf[i] = __data[i*4 + 1];//mic 0 mic1 mic2 mic3 0 1 2 3 4 5 6
//     }
//     //cbuf_write(&save_cbuf,buf,320);
// #endif // 0
}


void init_adc(u8 init_flag)
{
     int err;
     static void *dev = NULL;
     static int bindex ;
     static u32 parm[2];
    if(init_flag == 0){
        void *arg = (void *)AUDIO_TYPE_ENC_MIC; 
        dev = dev_open("audio", arg);  
        if(!dev){
            return;
        }
        printf("\n >>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);

        struct video_reqbufs breq = {0};
        breq.buf  = malloc(4096);
        breq.size = 4096;
        breq.dev.fd = NULL;
        err = dev_ioctl(dev, AUDIOC_REQBUFS, (unsigned int)&breq);
        if (err) {
            return;
        }
        printf("\n >>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);


        struct audio_format f = {0};;

        f.format        = "pcm";
        f.channel       = 1;
        f.sample_rate   = 16000;
        f.volume        = 100;   
        f.sample_source = "mic";
        f.frame_len     = 8192;


            f.channel_bit_map  = BIT(CONFIG_AUDIO_ADC_CHANNEL_L);
            printf("\n >>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);
            err = dev_ioctl(dev, AUDIOC_SET_FMT, (unsigned int)&f);
            if (err) {
                printf("audio_set_fmt: err");

            }
        printf("\n >>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);
        
        
        parm[0] = 0;
        parm[1] = (u32)audio_dev_enc_irq_handler;

     
        dev_ioctl(dev, IOCTL_REGISTER_IRQ_HANDLER, (u32)parm);
        printf("\n >>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);
        
        err = dev_ioctl(dev, AUDIOC_STREAM_ON, (u32)&bindex);
        printf("\n >>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);
     }else{
        if(dev){
        err = dev_ioctl(dev, AUDIOC_STREAM_OFF, (u32)&bindex);
        printf("\n >>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);

        dev_ioctl(dev, IOCTL_UNREGISTER_IRQ_HANDLER, (u32)parm);
        printf("\n >>>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);
        dev_close(dev);
        dev = NULL;
        log_info(">>>>>>>>>>>>>>>dac_complete close");
        }
        log_info(">>>>>>>>>>>>>>>dac_complete");

     }

}

static void rt_talk_dac_irq_handler(void *priv, void *data, int len)
{
//    putchar('B');
//
//    printf("\n >>>>>>>>>>>>>>>>>>>>>dac len = %d\n",len);
//    memset(data,0,len);
//   int ret =  cbuf_read(&save_cbuf,(u8 *)data,len);
//   printf("\n ret = %d\n",cbuf_get_data_size(&save_cbuf));
    memcpy(data,buf,len);
}
int init_dac(u8 init_flag)
{
    int err;
    static u32 arg[2];
    static int bindex;
    struct audio_format f;
    static void *dev = NULL;
    if(init_flag == 0){
        dev =  dev_open("audio", (void *)AUDIO_TYPE_DEC);
        if (!dev) {
            return 0;
        }

        f.volume = -1;
        f.channel = 1;
        f.sample_rate = 16000;
        f.priority = 9;

        err = dev_ioctl(dev, AUDIOC_SET_FMT, (u32)&f);
        if (err) {
            puts("format_err\n");
            return 0;
        }
        arg[0] = 0;
        arg[1] = (u32)rt_talk_dac_irq_handler;
    
    dev_ioctl(dev, IOCTL_REGISTER_IRQ_HANDLER, (u32)arg);    
    dev_ioctl(dev, AUDIOC_STREAM_ON, (u32)&bindex);
    }else{
        if(dev){
        dev_ioctl(dev,AUDIOC_STREAM_OFF, (u32)&bindex);
        dev_ioctl(dev, IOCTL_UNREGISTER_IRQ_HANDLER, (u32)arg);
        dev_close(dev);
        log_info(">>>>>>>>>>>>>>>dac_complete close");
        }
        log_info(">>>>>>>>>>>>>>>dac_complete....");

    }
 return 0;
}


void close_adc_dac (void)
{
    log_info(">>>>>>>>>>>>>>>close_adc_dac...."); 
    init_dac(1);
    init_adc(1);
}

void open_recorder(void)
{ 
    recorder_mode_init();
    log_info(">>>>>>>>>>>>>>>open_recorder...."); 
    msleep(5 * 100);
    init_dac(0);
    init_adc(0);
    
}

void recorder_test_task(void *priv)
{
//
//     printf("\n >>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);
//
//                     printf("\n >>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);
//            recorder_to_file(16000, 1, CONFIG_AUDIO_RECORDER_SAVE_FORMAT);
//                 printf("\n >>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);
//            msleep(20000);
//                 printf("\n >>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);
//          recorder_close();
//               printf("\n >>>>>>>>>>>>>>>>>%s %d\n",__func__,__LINE__);



    msleep(10 * 1000);
//    cbuf_init(&save_cbuf, cache_buf,  16 * 1024);

    init_dac(0);
    init_adc(0);

    while(1){




        msleep(100);
    //    if(offset +1280 >= sizeof(__data)){

    //        void *fd = fopen(CONFIG_ROOT_PATH"1.pcm","w+");
    //        fwrite(__data,offset,1,fd);
    //        fclose(fd);
    //    }
    }
}
#include "init.h"
static int recorder_test_init(void)   //主要是create wifi 线程的
{
    puts("recorder_test_init \n\n");
    return thread_fork("recorder_test_task", 10, 4096, 0, 0, recorder_test_task, NULL);
}

late_initcall(recorder_test_init);
#endif
