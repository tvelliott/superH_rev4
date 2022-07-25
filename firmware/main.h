
//MIT License
//
//Copyright (c) 2019 tvelliott
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.



#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32h7xx_hal.h"

#define ARM_MATH_CM7 1
#include "arm_common_tables.h"
#include "arm_math.h"

#define IN_BUFFER_N 16
#define N_SAMPLES 256

extern volatile int do_sample_process;
extern volatile int rssi_offset;
extern volatile int bb_gain;
extern volatile int fft_buffer_ready;
extern volatile int adc_rate;
extern volatile int stats_buffer_ready;
extern volatile int channel_settle;
extern volatile int squelch_timer;
extern volatile double d_alpha_i;
extern volatile double d_alpha_q;
extern volatile double d_beta_i;
extern volatile double d_beta_q;
void reset_channel_timeout( void );
void set_channel_timeout_zero( void );
void set_channel_timeout_min( void );
void scanner_next_channel( void );
double get_current_freq( void );
void process_incoming_rf( void );
int upsample_audio( uint8_t *buffer, int len );
void _Error_Handler( char *file, int line );
void calc_stddev( int16_t *data, int n );
int main( void );
float atan2f( float y, float x );
float get_real_amp( void );
extern volatile int udp_out_len;
extern float eqlms_dhat[1];
extern float eqlms_d[1];
extern float eqlms_mu;
extern float eqlms_beta;
extern unsigned int eqlms_m;
extern unsigned int eqlms_k;
extern struct eqlms_rf_s *eqlms;
extern struct freqmod_s *freqmod;
extern struct nco_cf_s *nco;
extern volatile int upsample_ready;
extern volatile short upsampled[160 * 6];
extern struct resamp_rf_s *audio_resamp_q;
extern unsigned int audio_resamp_npfb;
extern float audio_resamp_bw;
extern float audio_resamp_As;
extern unsigned int audio_resamp_m;
extern float audio_resamp_r;
extern unsigned int ch_ny;
extern struct resamp2_s *halfband_q;
extern float half_band_as;
extern unsigned int half_band_m;
extern struct resamp_cf_s *ch_resamp_q;
extern unsigned int ch_resamp_npfb;
extern float ch_resamp_bw;
extern float ch_resamp_As;
extern unsigned int ch_resamp_m;
extern float ch_resamp_r;
extern struct symsync_s *q;
extern struct symsync_s symsync;
extern float symsync_buf_out[1];
extern float symsync_buf_in[1];
extern unsigned int symsync_npfb;
extern float symsync_beta;
extern unsigned int symsync_m;
extern unsigned int symsync_k;
extern int symsync_ftype;
extern unsigned int did_sym;
void tcp_iperf_init( void );
static void MX_SPI3_Init( void );
static void SystemClock_Config( void );
extern volatile int fm_cnt;
extern int eqlms_trained;
extern int do_silent_frame;
extern int audio_frames_done;
extern volatile struct udp_pcb *udp_data_pcb;
extern int64_t __errno;
extern volatile int scanner_mod;
extern volatile int in_buffer_len;
extern volatile char in_buffer_tmp[1500 * IN_BUFFER_N];
extern volatile char in_buffer[1500 * IN_BUFFER_N];
extern int found_signal;
extern volatile int sample_cnt;
extern volatile uint32_t uptime_sec;
extern volatile uint32_t second_tick;
void main_tick( void );
void main_tick();
void led_tick( void );
void led_tick();
int _mem_free( void );
void udp_data_rx( void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip4_addr *addr, uint16_t port );
void p25_udp_init( void );
void p25_udp_init();
void udp_send_data( uint8_t *buffer, int len );
void udp_send_data( uint8_t *buffer, int len );
void process_incoming_samples( int16_t val );
void process_incoming_samples( int16_t val );
void p25_net_tick( void );
extern SPI_HandleTypeDef hspi3;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc1;
extern volatile int did_print_channel;
extern volatile double scanner_freq_p25[];
extern volatile int scanner_channel_mask[];
extern volatile int scanner_freqs;
extern volatile int8_t std_dev;
extern volatile uint32_t scanner_idle_time;
extern volatile int scanner_channel;
extern volatile int channel_timeout;
extern int stddev_n;
extern float32_t stddev_out;
extern float32_t stddev_in[N_SAMPLES];
extern volatile int do_low_if_mix;
extern volatile int do_48khz_iq_only;
extern volatile int do_freq_offset;
extern volatile struct ip4_addr udp_saddr;

#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_4 //actually 3
#define LED2_GPIO_Port GPIOE
#define data_4_Pin GPIO_PIN_2
#define data_4_GPIO_Port GPIOB
#define data_0_Pin GPIO_PIN_15
#define data_0_GPIO_Port GPIOF
#define data_1_Pin GPIO_PIN_0
#define data_1_GPIO_Port GPIOG
#define data_2_Pin GPIO_PIN_1
#define data_2_GPIO_Port GPIOG
#define postmix_gain2_Pin GPIO_PIN_7
#define postmix_gain2_GPIO_Port GPIOE
#define postmix_gain1_Pin GPIO_PIN_8
#define postmix_gain1_GPIO_Port GPIOE
#define postmix_gain0_Pin GPIO_PIN_9
#define postmix_gain0_GPIO_Port GPIOE
#define data_3_Pin GPIO_PIN_10
#define data_3_GPIO_Port GPIOE
#define data_5_Pin GPIO_PIN_11
#define data_5_GPIO_Port GPIOE
#define rf_sw_Pin GPIO_PIN_12
#define rf_sw_GPIO_Port GPIOE
#define mixer_cs_Pin GPIO_PIN_13
#define mixer_cs_GPIO_Port GPIOE
#define synth1_cs_Pin GPIO_PIN_14
#define synth1_cs_GPIO_Port GPIOE
#define synth2_cs_Pin GPIO_PIN_15
#define synth2_cs_GPIO_Port GPIOE
#define FLT_CS_Pin GPIO_PIN_10
#define FLT_CS_GPIO_Port GPIOB
#define atten_le_Pin GPIO_PIN_11
#define atten_le_GPIO_Port GPIOB
#define IR_INPUT_Pin GPIO_PIN_3
#define IR_INPUT_GPIO_Port GPIOE

#endif

