
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



#ifndef __GLOBALS_H_H__
#define __GLOBALS_H_H__

#define __USE_MISC 1  //include M_PI, etc in math.h
#include <math.h>

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "lwip.h"

#define ADC_SIZE   ((uint32_t)  256)
#define ADC3_SIZE   ((uint32_t)  256)

#define STATS_SIZE (ADC_SIZE/2)

#define UDP_PORT 8889

#define M_E   2.7182818284590452354 /* e */
#define M_LOG2E 1.4426950408889634074 /* log_2 e */
#define M_LOG10E  0.43429448190325182765  /* log_10 e */
#define M_LN2   0.69314718055994530942  /* log_e 2 */
#define M_LN10    2.30258509299404568402  /* log_e 10 */
#define M_PI    3.14159265358979323846  /* pi */
#define M_PI_2    1.57079632679489661923  /* pi/2 */
#define M_PI_4    0.78539816339744830962  /* pi/4 */
#define M_1_PI    0.31830988618379067154  /* 1/pi */
#define M_2_PI    0.63661977236758134308  /* 2/pi */
#define M_2_SQRTPI  1.12837916709551257390  /* 2/sqrt(pi) */
#define M_SQRT2 1.41421356237309504880  /* sqrt(2) */
#define M_SQRT1_2 0.70710678118654752440  /* 1/sqrt(2) */

extern uint32_t udp_send_ip_bak;
extern int current_talkgroup;
extern volatile int tone_timeout;
extern volatile int do_audio_tone;
extern volatile int audio_out_rate;
extern volatile char udp_out_buffer[];
void init_dc_correction( double val );
void init_globals( void );
int read_configuration_from_flash( void );
void write_configuration_to_flash( void );
float get_rssi( void );

extern int pocsag_debug_on;

extern int do_write_config;
extern int do_read_config;
extern int do_delete_config;
extern int do_system_reset;
extern int do_iq_mode;
extern int do_rf_tx_test;
extern int do_ls_regs;
extern int do_ls_presets;
extern int do_ls_talkgroups;
extern int ls_talkgroups_parm;
extern volatile int __preset;

typedef struct {
  volatile double frequency;
  volatile double if_frequency;
  volatile int audio_on;
  volatile float audio_volume_f;  //should be sys_config
  volatile int sa_mode;
  volatile int squelch;
  volatile int unused_22;
  volatile uint32_t mcu_unique_id;  //should be sys_config
  volatile int show_srate;
  volatile int show_rssi;
  volatile int rcut;
  volatile int bw;
  volatile int rfgain;
  volatile float i_off;
  volatile float q_off;
  volatile float i_gain;
  volatile float q_gain;
  volatile int logging;
  volatile int unused5;
  volatile int unused6;
  volatile int unused8;
  volatile int unused7;
  volatile int uart3_baudrate;
  volatile int dc_off;
  volatile int gain1;     //should be sys_config
  volatile int gain2;
  volatile int unused_25;
  volatile uint32_t mcu_ver;  //should be sys_config
  volatile float iq_dc_off_rate;
  volatile int adc_flt_bw;
  volatile int do_adsb;
  volatile int unused3;
  volatile int do_pagers;
  volatile int notused1;
  volatile int unused2;
  volatile int do_acars;
  volatile float am_gain;
  volatile int  am_offset;
  volatile int channel_bw;
  volatile int front_end_atten;
  volatile int bb_atten;
  volatile int bb_ampg;
  volatile int decimate;
  volatile int audio_filter;
  volatile int do_low_if;
  volatile float nco_val;
  volatile float nco_offset_freq;

  volatile int agc_auto;
  volatile int agc_target;
  volatile int agc_max_gain1;
  volatile int agc_attack;
  volatile int agc_decay;
  volatile int agc_mode;


  union {
    volatile char ip_addr[4]; //should be sys_config
    volatile uint32_t ip_addr32;  //should be sys_config
  };

  volatile float aud_dc_off_rate;
  volatile float p25_audio_gain;
  volatile char  preset_desc_str[65];
  volatile float freq_offset_mhz;
  volatile int rssi_offset;  //should be sys_config 
  volatile int unused_23;

  union {
    volatile char udp_ip_addr[4]; //should be sys_config
    volatile uint32_t udp_ip_addr32;  //should be sys_config
  };

  volatile int unused_20;
  volatile int adc3_frac; //should be sys_config

  volatile int unused_21;
  volatile float fm_deviation;
  volatile int16_t dmr_level_max;
  volatile int is_control;
  volatile float aud_agc;
  volatile int p25_sys_id;
  volatile int p25_follow;
  volatile int if_low_high;
  volatile int udp_mode;

  volatile int mixer_doi;
  volatile int mixer_doq;

  volatile int aud_agc_en;
  volatile int p25_grant;
  volatile int am_mode;

  uint8_t padding[103];

} config_t1;

typedef struct {
  union {
    config_t1 *config;
    config_t1 configt1;
  };
  volatile uint32_t crc32;
} config_t;

const enum {
  MODE_FM=0,
  MODE_IQ=1,
  MODE_P25=2,
  MODE_AM=3,
  MODE_4=4,
  MODE_ADSB=5,
  MODE_6=6,
  MODE_DMR=7,
  MODE_SYNCMON=8
};

const enum {
  UDP_STREAM_OFF,
  UDP_STREAM_ADC_RAW,
  UDP_STREAM_IQ16_DECFLT,
  UDP_STREAM_DEMOD,
  UDP_STREAM_4FSK_SYM_S16,
  UDP_STREAM_4FSK_DIBIT,
  UDP_STREAM_AUDIO_48K_S16,
  UDP_STREAM_AUDIO_48K_S16_CONT,
  UDP_STREAM_VOICE8K,
  UDP_STREAM_CONSOLE
};

extern config_t1 *config;
extern void main_tick( void );
extern void DelayClk3( unsigned long );
extern void delay_ms( int delay );
extern void delay_us( int delay );
extern SPI_HandleTypeDef hspi3;
extern volatile struct udp_pcb *udp_data_pcb;
extern volatile struct ip4_addr udp_saddr;
extern volatile int8_t do_handle_exti1;
extern volatile uint16_t iq_data[512];
extern void process_incoming_rf();
extern volatile uint32_t out_s;
extern volatile uint32_t out_e;
extern volatile int is_playing;
void audio_int_config( void );
void rf900_int_config( void );
double get_current_freq( void );
float get_real_amp( void );
void reset_channel_timeout( void );
void set_channel_timeout_min( void );
volatile uint32_t uptime_sec;
void udp_send_response_demod( uint8_t *buffer, int len );
volatile int do_low_if_mix;
volatile int do_freq_offset;
volatile int do_48khz_iq_only;

#define FLASH_BASE_ADDR      (uint32_t)(FLASH_BASE)
#define FLASH_END_ADDR       (uint32_t)(0x081FFFFF)

/* Base address of the Flash sectors Bank 1 */
#define ADDR_FLASH_SECTOR_0_BANK1     ((uint32_t)0x08000000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK1     ((uint32_t)0x08020000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK1     ((uint32_t)0x08040000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK1     ((uint32_t)0x08060000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK1     ((uint32_t)0x08080000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK1     ((uint32_t)0x080A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK1     ((uint32_t)0x080C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK1     ((uint32_t)0x080E0000) /* Base @ of Sector 7, 128 Kbytes */

/* Base address of the Flash sectors Bank 2 */
#define ADDR_FLASH_SECTOR_0_BANK2     ((uint32_t)0x08100000) /* Base @ of Sector 0, 128 Kbytes */
#define ADDR_FLASH_SECTOR_1_BANK2     ((uint32_t)0x08120000) /* Base @ of Sector 1, 128 Kbytes */
#define ADDR_FLASH_SECTOR_2_BANK2     ((uint32_t)0x08140000) /* Base @ of Sector 2, 128 Kbytes */
#define ADDR_FLASH_SECTOR_3_BANK2     ((uint32_t)0x08160000) /* Base @ of Sector 3, 128 Kbytes */
#define ADDR_FLASH_SECTOR_4_BANK2     ((uint32_t)0x08180000) /* Base @ of Sector 4, 128 Kbytes */
#define ADDR_FLASH_SECTOR_5_BANK2     ((uint32_t)0x081A0000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7_BANK2     ((uint32_t)0x081E0000) /* Base @ of Sector 7, 128 Kbytes */

#define CONFIG_ADDRESS_BACKUP ADDR_FLASH_SECTOR_6_BANK2
#define CONFIG_ADDRESS ADDR_FLASH_SECTOR_7_BANK2
#endif
