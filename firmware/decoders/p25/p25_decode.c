
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


/*
 * Copyright (C) 2010 DSD Author
 * GPG Key ID: 0x3F1D7FD0 (74EF 430D F7F2 0A48 FCE6  F630 FAA2 635D 3F1D 7FD0)
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS.  IN NO EVENT SHALL ISC BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */



#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <complex.h>
#include <limits.h>

#include "command.h"
#include "globals.h"
#include "p25p1_const.h"
#include "Golay.h"

#include "mbelib/mbelib.h"
#include "p25_stats.h"
#include "p25_decode.h"
#include "p25p1_const.h"

#include "resamp_rf_arm32.h"
#include "resamp_cf_arm32.h"

#include "mbelib/mbelib_test_main.h"
#include "dsd.h"
#include "agc.h"
#include "firfilt_rf_arm32.h"
#include "fec.h"
#include "std_io.h"
#include "rfdemod.h"


static double goto_freq;
static int do_goto_freq;
static int on_control_freq;
static int return_to_control_mod;
int current_talkgroup;
static int previous_talkgroup;
static int current_priority;
static char current_desc[32]; 
static char current_loc[16]; 
static int current_nac;
static tgrecord *tgrec;
static int nrecs;

static float current_p25_gain=1.0f;

#define P25_CONTROL_TIMEOUT 500
#define P25_IGNORE_ENCRYPTED_TG_TIMEOUT 30000
static volatile int p25_time_since_grant;

static base_freq_iden bfreq_iden[16];
static base_freq_iden *bfreq_ptr;

static struct firfilt_rf_s audio_fir_s4;
static struct firfilt_rf_s *audio_fir4=NULL;

static int soft_sync=0;
static int found_header;

static float audio_fir_bw4;
static int audio_fir_len4;
static float audio_fir_as;
static float voice_fir_f;
static float last_input;


extern dsd_state dsdstate;
extern dsd_opts dsdopts;
extern dsd_state *t_state;
extern dsd_opts *t_opts;

extern volatile int agc_locked;

static int encrypted_count;

static int symsync_reset_mod;

static int sync_count;
void MX_LWIP_Process( void );

void p25_reset_stats( void );
void udp_send_data( char *buffer, int len );
int upsample_audio( char *buffer, int len );
void p25_net_tick( void );

static uint8_t tmp_uint8_128[128];
static uint8_t get_tsdu;
static int tsdu_idx;
static uint8_t trellis_buffer[49];
static uint8_t out[12];
static unsigned int err;
static int tsdu_blocks;
static int tsdu_last_block;
static int tsbk_count;


extern int audio_frames_done;
extern int do_silent_frame;

static int debug = 0;

static int gerr;

static uint64_t dreg;

int p25_status_mod;

static int is_synced;

static uint32_t nac;
static uint8_t duid;

static int status_bit_counter;
static int bit_count;
static int voice_state;

static char imbe_d[88];
static char imbe_fr[8][23];

static char hdu_hex_data[9 * 8];

static char ldu_hex_parity[20 * 6];


static int errors;
static int errors2;

static int skip_bits;

static const float  max = DEFAULT_MAX;
static const float  min = DEFAULT_MIN;
static const float  center = DEFAULT_CENTER;

  //use max/min to define the center and slicer levels
static const float  umid = DEFAULT_CENTER + ( fabs( DEFAULT_MAX - DEFAULT_CENTER ) * 0.55f );
static const float  lmid = DEFAULT_CENTER - ( fabs( DEFAULT_CENTER - DEFAULT_MIN ) * 0.55f );

static uint8_t cval; 


static unsigned char algid = 0;
static int word_count;
static int hex_bit_count;

volatile short out_buffer[OUT_BUFFER_SIZE];
static uint32_t stime;

static int p25_reset_tick;
struct symsync_s *ss_q = NULL;

extern struct resamp_rf_s *audio_resamp_q;
extern struct resamp_cf_s *ch_resamp_q_narrow;

static volatile int16_t last_upsampled[6*160];
static volatile int16_t last_8khz[160];


enc_timeout_rec enc_timeouts[16];


const unsigned short crc_table [256] = {

  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5,
  0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b,
  0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
  0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c,
  0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
  0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b,
  0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
  0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
  0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5,
  0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969,
  0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
  0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
  0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03,
  0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
  0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6,
  0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
  0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
  0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1,
  0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c,
  0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2,
  0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
  0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447,
  0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
  0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2,
  0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
  0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827,
  0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
  0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0,
  0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d,
  0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
  0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba,
  0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
  0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};



//Symbol interleaving, derived from CAI specification table 7.4
static const int INTERLEAVING[] = {
    0, 13, 25, 37,
    1, 14, 26, 38,
    2, 15, 27, 39,
    3, 16, 28, 40,
    4, 17, 29, 41,
    5, 18, 30, 42,
    6, 19, 31, 43,
    7, 20, 32, 44,
    8, 21, 33, 45,
    9, 22, 34, 46,
   10, 23, 35, 47,
   11, 24, 36, 48,
   12
};

//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
unsigned short CRCCCITT(uint8_t *data, int16_t length, uint16_t seed, uint16_t final)
{

int16_t count;
unsigned int crc = seed;
unsigned int temp;

  for (count = 0; count < length; ++count) {
    temp = (*data++ ^ (crc >> 8)) & 0xff;
    crc = crc_table[temp] ^ (crc << 8);
  }

  return (unsigned short)(crc ^ final);
}



//////////////////////////////////////////////////////////////////////////////////////////////////
//typedef struct {
//  uint32_t enabled;
//  int32_t sys_id;
//  int32_t  priority;
//  int32_t talkgroup;
//  char    desc[32];
//  char    location[16];
//} tgrecord;
//#define ADDR_FLASH_SECTOR_6_BANK2     ((uint32_t)0x081C0000) /* Base @ of Sector 6, 128 Kbytes */
//////////////////////////////////////////////////////////////////////////////////////////////////
void list_talkgroups(int sys_id_filter) {

  int i;
  uint32_t n;
  tgrecord *ptr;

  //uint32_t prim = __get_PRIMASK();
  //__disable_irq();

  memcpy( (uint8_t *) &n, (uint8_t *) ADDR_FLASH_SECTOR_1_BANK2, 4);  //first 4 bytes in bank are the # of records
  ptr = (tgrecord *) (ADDR_FLASH_SECTOR_1_BANK2 + 4L);  //ptr to first record

  printf("\r\nP25 Talkgroups");

  for(i=0;i<n;i++) {

    if(sys_id_filter==0 || sys_id_filter==ptr->sys_id) {
      printf("\r\ntg: %d, sys_id: %d, desc: %s, loc: %s", 
        ptr->talkgroup, 
        ptr->sys_id, 
        &ptr->desc, 
        &ptr->location);

      if(!ptr->enabled) printf("  >DISABLED<");
    }

    ptr++; 

    if( i % 16 == 0 ) { //every 16 records, wait for console buffer to flush
      while( netbuf_s!=netbuf_e ) {
        main_tick();
      }
    }
  }

  //if( !prim ) {
  //  __enable_irq();
  //}
 
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void p25_list_base_freqs(void) {

  int i;
  base_freq_iden *ptr;

  printf("\r\nP25 Control Channel Frequency Plan");

  for(i=0;i<16;i++) {
    ptr = (base_freq_iden *) &bfreq_iden[i];

    if(ptr->base_freq!=0) {
      printf("\r\n%02d  iden %d, base_freq %d, bw %d, tx_off_vu %d, spacing %d",
        i,
        ptr->iden,
        ptr->base_freq,
        ptr->bw,
        ptr->tx_off_vu,
        ptr->spacing );
    }
    else {
      printf("\r\n%02d  Not Used.", i);
    }
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
void p25_reset_base_iden(void) {
  memset( (uint8_t *) &bfreq_iden, 0x00, sizeof(base_freq_iden)*16);
  do_goto_freq=0;
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
int p25_is_tg_enabled(int tg, int *pri) {

  int i;
  int enabled=-1;

  uint32_t prim = __get_PRIMASK();
  __disable_irq();

  memcpy( (uint8_t *) &nrecs, (uint8_t *) ADDR_FLASH_SECTOR_1_BANK2, 4);
  tgrec = (tgrecord *) (ADDR_FLASH_SECTOR_1_BANK2 + 4L);


  for(i=0;i<nrecs;i++) {
    if(tgrec->talkgroup == tg && config->p25_sys_id == tgrec->sys_id) {
      enabled = tgrec->enabled; 
      *pri = tgrec->priority;
      break;
    }
    tgrec++; 
  }


  if( !prim ) {
    __enable_irq();
  }

  return enabled;
}


//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
char * p25_get_desc(int tg) {

  uint32_t prim = __get_PRIMASK();
  __disable_irq();

  memcpy( (uint8_t *) &nrecs, (uint8_t *) ADDR_FLASH_SECTOR_1_BANK2, 4);
  tgrec = (tgrecord *) (ADDR_FLASH_SECTOR_1_BANK2 + 4L);

  char *str = NULL;

  int i;
  for(i=0;i<nrecs;i++) {
    if(tgrec->talkgroup == tg && config->p25_sys_id == tgrec->sys_id) {
      str = &tgrec->desc; 
      break;
    }
    tgrec++; 
  }

  if( !prim ) {
    __enable_irq();
  }

  return str;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
char * p25_get_loc(int tg) {

  uint32_t prim = __get_PRIMASK();
  __disable_irq();

  memcpy( (uint8_t *) &nrecs, (uint8_t *) ADDR_FLASH_SECTOR_1_BANK2, 4);
  tgrec = (tgrecord *) (ADDR_FLASH_SECTOR_1_BANK2 + 4L);

  char *str = NULL;

  int i;
  for(i=0;i<nrecs;i++) {
    if(tgrec->talkgroup == tg && config->p25_sys_id == tgrec->sys_id) {
      str = &tgrec->location; 
      break;
    }
    tgrec++; 
  }

  if( !prim ) {
    __enable_irq();
  }

  return str;
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
void process_mbe( char *t_imbe_d, int err1, int err2, int enc, int is_last, int is_imbe )
{

  int i;

  if(config->sa_mode==MODE_P25 && config->is_control && on_control_freq==1 ) return;   //ignore garbage coming in if we are on the control channel

  if(audio_fir4==NULL) {
    audio_fir_bw4 = ( 0.08); 
    audio_fir_len4 = 64;  //64 taps
    audio_fir_as = -50.0f; //50dB attenuation
    audio_fir4 = firfilt_rf_create_kaiser( &audio_fir_s4, audio_fir_len4, audio_fir_bw4, audio_fir_as, 0.0f );
  }

  t_state->errs = err1;
  t_state->errs2 = err2;


    //TODO: figure out why printf doesn't work right with single character strings
   if(err1>3 || err2>3) {
    putchar_stdio('E');
   }
   else {
    putchar_stdio('*');
   }


  if(is_imbe) {
    t_opts->uvquality = 2;  //about 7 max for stm32,  2 is ok
    //P25P1
    mbe_processImbe4400Dataf( dsd_fbuf, &t_state->errs, &t_state->errs2, t_state->err_str, t_imbe_d, t_state->cur_mp, t_state->prev_mp, t_state->prev_mp_enhanced, t_opts->uvquality );
  }
  else {
    t_opts->uvquality = 2;
    //DMR and P25 Phase 2 use AMBE 2450
    mbe_processAmbe2450Dataf( dsd_fbuf, &t_state->errs, &t_state->errs2, t_state->err_str, t_imbe_d, t_state->cur_mp, t_state->prev_mp, t_state->prev_mp_enhanced, t_opts->uvquality );

    //D-STAR uses AMBE 2400
    //mbe_processAmbe2400Dataf( dsd_fbuf, &t_state->errs, &t_state->errs2, t_state->err_str, t_imbe_d, t_state->cur_mp, t_state->prev_mp, t_state->prev_mp_enhanced, t_opts->uvquality );
  }


  if(config->p25_audio_gain<0.01f) config->p25_audio_gain=1.0f;

  if( enc == 0x00 && encrypted_count==0x00 && (found_header || config->sa_mode==MODE_DMR) ) { //non-encrypted voice


   if(is_imbe) {

      //P25P1 


     if(err1<=3 && err2<=3) {

        for( i = 0; i < 160; i++ ) {
          dsd_fbuf[i] *= config->p25_audio_gain * 0.8;
        }
     }
     else {
        for( i = 0; i < 160; i++ ) {
          dsd_fbuf[i] = 0; 
        }
     }

   }
   else {

     //DMR 

     if(err1<=3 && err2<=3 ) {


        for( i = 0; i < 160; i++ ) {
          dsd_fbuf[i] *= config->p25_audio_gain * 0.8;
        }
     }
     else {
        for( i = 0; i < 160; i++ ) {
          dsd_fbuf[i] =0.0f; 
        }
     }
   }

    if( config->udp_mode == UDP_STREAM_VOICE8K ) {
      for( i = 0; i < 160; i++ ) {
        dsd_fbuf[i] *= current_p25_gain * 7.5f;  //should result in ~ optimum audio level
      }
    }
    mbe_processAudio( dsd_fbuf, dsd_sbuf ); //convert float to shorts ... 160 of them.  Output is 8 kHz
    audio_frames_done++;

    char *ptr = ( char * ) dsd_sbuf;
    memcpy((char *) last_8khz, ptr, 320); 

    if( out_s == out_e ) {
      resamp_rf_reset( audio_resamp_q ); //why does this need to be done?
    }

    if( config->udp_mode == UDP_STREAM_VOICE8K ) {
      udp_send_data( ( char * ) dsd_sbuf, 320 );
    }
    

    //re-sample the audio from 8 KHz to 48 kHz to match desired audio hardware/clocks
    int n = upsample_audio( ( char * ) ptr, 320 );


    for( i = 0; i < n; i++ ) {

      //64-tap fir filter to low-pass the voice data from the synthesizers
      voice_fir_f = (float) upsampled[i] / 32768.0f;


      firfilt_rf_push( audio_fir4, voice_fir_f); 
      firfilt_rf_execute( audio_fir4, &voice_fir_f );
      upsampled[i] = (int16_t) (voice_fir_f * 3500.0f);

      //high freq boost
      //if( highpass_boost_filter_on ) {
        if(i==0) {
          last_input = upsampled[i];
        }
        else {
          float output = upsampled[i] - 0.95f * last_input;
          last_input = upsampled[i];
          upsampled[i] += output * 0.5 * 1.6;
        }
      //}



      last_upsampled[i] = upsampled[i];
    }

    uint32_t prim = __get_PRIMASK();
    __disable_irq();

      //this also gets called in am_demod inside int handler
      current_p25_gain = update_gain_s16(upsampled, n, 22500.0f, 29.0f);

      for( i = 0; i < n; i++ ) {
        //this must be inside disable/enable irq block
        out_buffer[out_e++] = upsampled[i];
        out_e &= (uint32_t) (OUT_BUFFER_SIZE - 1);
      }

      if( config->udp_mode==UDP_STREAM_AUDIO_48K_S16 || config->udp_mode==UDP_STREAM_AUDIO_48K_S16_CONT) {
        udp_send_data((uint8_t *) upsampled,n*2);
      }

    if( !prim ) {
      __enable_irq();
    }

  }

}


/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
void add_encrypted_talkgroup(int tg) {
  int i;

  enc_timeout_rec *enc_ptr;
  for(i=0;i<16;i++) {
    enc_ptr = &enc_timeouts[i]; 
    if(enc_ptr->timeout==0) {
      enc_ptr->talkgroup=tg;
      enc_ptr->timeout=30000; //30 seconds 
      return;
    }
  }

  //couldn't find an empty record, so just use the first one
  enc_ptr = &enc_timeouts[0]; 
  enc_ptr->talkgroup=tg;
  enc_ptr->timeout=30000; //30 seconds 

  return;
}



/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
int is_encrypted_talkgroup(int tg) {
  int i;

  enc_timeout_rec *enc_ptr;
  for(i=0;i<16;i++) {
    enc_ptr = &enc_timeouts[i]; 
    if(enc_ptr->talkgroup==tg && enc_ptr->timeout>0) return 1; 
  }

  return 0;
}


/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
void p25_net_tick()
{
  int i;
  uint32_t prim = __get_PRIMASK();
  __disable_irq();

  p25_time_since_grant++;
  if(config->logging > 5 && p25_time_since_grant%1000==0) printf("\r\np25 time since last grant %d ms", p25_time_since_grant);

  if( p25_reset_tick++ > 2000 ) {
    p25_reset_tick = 0;
    p25_reset_stats();
  }


  //encrypted talkgroups timeout counters
  enc_timeout_rec *enc_ptr;
  for(i=0;i<16;i++) {
    enc_ptr = &enc_timeouts[i]; 
    if(enc_ptr->timeout>0) enc_ptr->timeout--;
  }

  if( config->is_control) {
    if(do_goto_freq && is_synced) {
      p25_reset_tick=0;
      do_goto_freq=0;
      printf("\r\n  control is switching to monitor grant for talkgroup on freq %4.6f", goto_freq);
      printf("\r\n  ==============================================================================================");
      set_freq_mhz(config->if_frequency + goto_freq);
      on_control_freq=0;
      return_to_control_mod=1;
    }
    else if(!is_synced && !on_control_freq) {
      if(return_to_control_mod++%P25_CONTROL_TIMEOUT==0) {

        //attempt to flush audio
        //this must be inside disable/enable irq block
        for( i = 0; i < 320*6; i++ ) {
          out_buffer[out_e++] = 0; 
          out_e &= (uint32_t) (OUT_BUFFER_SIZE - 1);
        }

        printf("\r\n  >== Conversation timed-out.  Return To Control Channel Frequency %4.6f ==<\r\n", config->frequency);
        set_freq_mhz(config->frequency + config->if_frequency);
        on_control_freq=1;
        current_priority=INT_MIN;
        encrypted_count=0;
        do_goto_freq=0;
        is_synced=0;

      }
    }
  }

  if( !is_synced ) {
    led2_off();
  }

  if( !prim ) {
    __enable_irq();
  }

}

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
int p25_is_synced()
{
  return is_synced;
}

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
void p25_reset_stats()
{

  if( ss_q == NULL ) return;

  if( uptime_sec < 20 ) return;

  if( config->logging > 99 ) printf( "\r\np25_reset_stats" );

  p25_status_mod=0;

  if(is_synced) {
    p25_reset_tick=0;
  }
  is_synced = 0;
  status_bit_counter=0;
  nac = 0;
  duid = -1;
  bit_count = 0;
  voice_state = -1;

  algid = 0;

  symsync_reset( ss_q );
  resamp_cf_reset( ch_resamp_q_narrow );
}

/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
void p25_decode( float sample_f, struct symsync_s *symsync_q_4fsk )
{


  int i;
  unsigned int k;
  unsigned char t;


  ss_q = symsync_q_4fsk;


  ////////////////////////////////////////////////////////////////
  dreg <<= 2;

  if(!is_synced) {
    status_bit_counter=0;
  }

  //slice 4 levels
  if( sample_f > umid ) {
    dreg |= 0x01;
    cval = 0x01; 
  } else if( sample_f > center ) {
    dreg |= 0x00;
    cval = 0x00; 
  } else if( sample_f < lmid ) {
    dreg |= 0x03;
    cval = 0x03; 
  } else {
    dreg |= 0x02;
    cval = 0x02; 
  }

  ////////////////////////////////////////////////////////////////


  if(skip_bits>0) {
    skip_bits--;
    goto end_decode;
  }


  /////////////////////////////////////////////////////////////////////
  // these are the 'status bits' that annoyingly happen every 36 dibits
  /////////////////////////////////////////////////////////////////////
  if( ++status_bit_counter == 36 ) {
    //if(config->logging>90) printf("\r\nstatus_bits: %02x", (uint8_t) cval);
    if(config->logging>5) {
      if(cval==0x03) printf("\r\nCHANNEL IDLE");
      if(cval==0x01) printf("\r\nCHANNEL BUSY");
    }

    status_bit_counter = 0;
    goto end_decode;  //skip over everything

  } else {
    bit_count += 2;
  }


  if(config->is_control && get_tsdu) {

    tmp_uint8_128[tsdu_idx++] = cval; 
    if(tsdu_idx==98) {
      tsdu_idx=0;
      if(!tsdu_last_block && tsdu_blocks<3) {

        #if 0
          printf("\r\nraw dibits:  ");
          for(i=0;i<49;i++) {
            printf("%02x,", tmp_uint8_128[i]);
          }
          goto end_decode;
        #endif


        for (i = 0; i < 49; i++) {
          k = (int) INTERLEAVING[i];
          t = (uint8_t) ((tmp_uint8_128[2*k] << 2) | (tmp_uint8_128[2*k+1] ));
          trellis_buffer[i] = t;
        }
        #if 0
          printf("\r\ntrellis buffer:  ");
          for(i=0;i<49;i++) {
            printf("%02x,", trellis_buffer[i]);
          }
          goto end_decode;
        #endif
        memset(tmp_uint8_128,0x00,sizeof(tmp_uint8_128));
        err = p25_trellis_1_2_decode(trellis_buffer, 49, tmp_uint8_128); 

        for(i = 0; i < 12; ++i) {
          out[i] = ((tmp_uint8_128[4*i] << 6) | (tmp_uint8_128[4*i+1] << 4) | (tmp_uint8_128[4*i+2] << 2) | (tmp_uint8_128[4*i+3] << 0));
        }
        if(!err) {
          processTSBK(out);
          tsdu_blocks++;
          tsdu_last_block=out[0]>>7;
        }
        else {
          get_tsdu=0;
          bit_count = 0;
          voice_state = 0;
          duid = -1;
        }

        if(tsdu_last_block) {
          get_tsdu=0;
          bit_count = 0;
          voice_state = 0;
          duid = -1;
        }

        goto end_decode; 
      }
      else {
        get_tsdu=0;
        bit_count = 0;
        voice_state = 0;
        duid = -1;
      }

    }
    goto end_decode; 
  }

  #define SYNC_MAX_BAD_BITS 2

  //soft sync detection.  allow up to 3 errors in the sync word
  uint64_t sync_test = (dreg&0xffffffffffff) ^ P25P1_TUNING_SYNC;
  soft_sync=0;
  for(i=0;i<48;i++) {
    if(sync_test & 0x01) soft_sync++;
      sync_test>>=1;
  }

  if( ( dreg & 0xffffffffffff ) == P25P1_TUNING_SYNC  || soft_sync < (SYNC_MAX_BAD_BITS+1) ) {

    if(config->logging >3) printf("\r\nsync:  is_sync_count %d", is_synced);
    if(soft_sync > 0 && config->logging>3) printf("\r\nSYNC ERR: corrupted bit count: %d", soft_sync);

    if( config->logging > 4 ) printf( "\r\nfound sync %d, freq %3.4f", sync_count++, get_current_freq() );

    if(!is_synced) symsync_set_lf_bw( symsync_q_4fsk, 0.005f );  //tighten up the loop filter now that we are synced
    is_synced = ( 32 * 6 + 8 );
    nac = 0;
    duid = -1;
    status_bit_counter = 0;
    bit_count = 0;
    voice_state = -1;
    p25_reset_tick = 0;
  } 

  if( bit_count > 426 ) {
    bit_count = 0;
  }

  if( is_synced ) {
    is_synced--;

    if( is_synced == 0 ) {

      algid = 0;
      bit_count = 0;
      voice_state = 0;
      duid = -1;
      found_header=0;

      symsync_set_lf_bw( symsync_q_4fsk, 0.01f );
      symsync_reset( ss_q );                    //if the symsync code is not reset periodically, it will stop working 
                                                //very well
      resamp_cf_reset( ch_resamp_q_narrow );

    }
  }

  if( !is_synced && symsync_reset_mod++ % 48000 == 0 ) {
    symsync_set_lf_bw( symsync_q_4fsk, 0.01f );
    symsync_reset( ss_q );
    resamp_cf_reset( ch_resamp_q_narrow );
  } else if( is_synced ) {
    symsync_reset_mod = 1;
  }

  if( is_synced && voice_state == -1 ) {

    //NAC.  Note that the NAC is *part* of the sys_id.  It appears that the sys_id is 4 more lsb or'ed with the NAC
    if( bit_count >= 2 && bit_count <= 12 ) {
      nac <<= 2;
      nac |= ( uint32_t )( dreg & 0x03 );
      current_nac = nac;
      duid = 0;

      //TODO:
      //haven't seen issues yet, but the 'nac' here could be used to reject sync with a near-by incoming mobile unit
    }

    if( bit_count >= 14 && bit_count <= 16 ) {
      duid <<= 2;
      duid |= ( uint8_t )( dreg & 0x03 );
    }

    //TODO:  need to error-correct / validate the nac and duid



    if( bit_count == 18 ) {
      voice_state = 0;

      switch( ( int ) duid ) {

      case 0x07 :
          if(config->is_control) {
            if(config->logging>4) printf("\r\nfound TSBK");
            get_tsdu=1;
            tsdu_idx=0;
            status_bit_counter=21;
            is_synced = 1200;
            skip_bits=24;
            tsdu_blocks=0;
            tsdu_last_block=0;
            goto end_decode;
          }
      break;

      case  0x00  :
        if( !debug && config->logging > 1 ) printf( "\r\n\n  ->HDU");

        if(!config->is_control || on_control_freq==0) {
          word_count = 0;
          memset( ( char * )hdu_hex_data, 0x00, sizeof( hdu_hex_data ) );
          is_synced = 1200;
          algid = 0x00;
        }
        break;

      case  0x05  :
        if(!config->is_control || on_control_freq==0) {
          is_synced = 1200;
          led2_on();
          tdu_count = 0;
          return_to_control_mod=1;
           p25_status_mod=1;

           //printf( "\r\n  ->LLDU1 (VOICE), freq: %4.4f, rssi: %3.0f dBm, TGroup: %d, Desc: %s   ",  goto_freq, get_rssi(), current_talkgroup, current_desc );
           printf( "\r\n  ->(VOICE), freq: %4.4f, rssi: %3.0f dBm, TGroup: %d, Desc: %s   ",  goto_freq, get_rssi(), current_talkgroup, current_desc );
        }
        break;

      case  0x0a  :
        if(!config->is_control || on_control_freq==0) {
          is_synced = 1200;
          led2_on();
          tdu_count = 0;
          return_to_control_mod=1;
          p25_status_mod=1;

          //printf( "\r\n  ->LLDU2 (VOICE), freq: %4.4f, rssi: %3.0f dBm, TGroup: %d, Desc: %s   ",  goto_freq, get_rssi(), current_talkgroup, current_desc );
          printf( "\r\n  ->(VOICE), freq: %4.4f, rssi: %3.0f dBm, TGroup: %d, Desc: %s   ",  goto_freq, get_rssi(), current_talkgroup, current_desc );
        }
        break;

      case  0x03  :
        if( !debug && config->logging > 1 ) printf( "\r\n  ->TDU" );
        is_synced = 1200;
        return_to_control_mod=1;  
        //  duid=-1;
        break;

      case  0x0c  :
          if(config->logging>2) printf("\r\nPDU");
      break;

      case  0x0f  :

        if( !debug && config->logging > 1 ) printf( "\r\n  ->TDULC" );
        //is_synced=1;
        //return_to_control_mod=192;  //LLDU1/2 keep this in line with activity
        //is_synced = 1200;
        duid = -1;
        break;

      default :
        //is_synced=0;
        duid = -1;
        voice_state = -1;
        if(config->logging>2) printf("\r\nunknown DUID type 0x%02x", duid);
        break;
      }

    }
  }
  if( is_synced && duid == 0x00 && voice_state == 0 && bit_count > 66 + ( 9 * 12 * 2 ) ) {
    if( word_count < 8 * 9 ) {
      hdu_hex_data[word_count++] = dreg & 0x03;
      if( word_count == 8 * 9 ) {
        p25_processHDU( hdu_hex_data );
      }
    }
  }

  //voice unit
  if( is_synced && ( duid == 0x05 || duid == 0x0a ) ) {

    switch( voice_state ) {
    case  0 :
      if( bit_count == 66 ) { //NID bits
        bit_count = 0;
        voice_state = 1;
        status_bit_counter = 21;
      }
      break;

    case  1 : //VOICE1

      deinterleave_frame( bit_count - 2, ( uint8_t )( dreg & 0x03 ) );
      if( bit_count == 144 ) {

        memset( imbe_d, 0x00, 88 );

        errors = p25_mbe_eccImbe7200x4400C0( imbe_fr );
        p25_mbe_demodulateImbe7200x4400Data( imbe_fr );
        errors2 = p25_mbe_eccImbe7200x4400Data( imbe_fr, imbe_d );
        process_mbe( imbe_d, errors, errors2, algid, 0, 1 );

        bit_count = 0;
        voice_state = 2;
      }
      break;

    case  2 : //VOICE2
      deinterleave_frame( bit_count - 2, ( uint8_t )( dreg & 0x03 ) );
      if( bit_count == 144 ) {
        memset( imbe_d, 0x00, 88 );

        errors = p25_mbe_eccImbe7200x4400C0( imbe_fr );
        p25_mbe_demodulateImbe7200x4400Data( imbe_fr );
        errors2 = p25_mbe_eccImbe7200x4400Data( imbe_fr, imbe_d );
        process_mbe( imbe_d, errors, errors2, algid, 0, 1 );

        bit_count = 0;
        voice_state = 3;
        hex_bit_count = 0; //only reset here in the VOICE frames to collect total of 40 * 6 bits
      }
      break;

    case  3 : //4 blocks of 6-bits data + 4-bits parity for 40 bits total
      if( bit_count < 40 ) {
        ldu_hex_parity[hex_bit_count++] = ( dreg & 0x03 );
      } else {
        bit_count = 0;
        voice_state = 4;
      }
      break;

    case  4 : //VOICE3
      deinterleave_frame( bit_count - 2, ( uint8_t )( dreg & 0x03 ) );
      if( bit_count == 144 ) {
        memset( imbe_d, 0x00, 88 );

        errors = p25_mbe_eccImbe7200x4400C0( imbe_fr );
        p25_mbe_demodulateImbe7200x4400Data( imbe_fr );
        errors2 = p25_mbe_eccImbe7200x4400Data( imbe_fr, imbe_d );
        process_mbe( imbe_d, errors, errors2, algid, 0, 1 );

        bit_count = 0;
        voice_state = 5;
      }
      break;

    case  5 : //4 blocks of 6-bits data + 4-bits parity for 40 bits total
      if( bit_count < 40 ) {
        ldu_hex_parity[hex_bit_count++] = ( dreg & 0x03 );
      } else {
        bit_count = 0;
        voice_state = 6;
      }
      break;

    case  6 : //VOICE4
      deinterleave_frame( bit_count - 2, ( uint8_t )( dreg & 0x03 ) );
      if( bit_count == 144 ) {
        memset( imbe_d, 0x00, 88 );

        errors = p25_mbe_eccImbe7200x4400C0( imbe_fr );
        p25_mbe_demodulateImbe7200x4400Data( imbe_fr );
        errors2 = p25_mbe_eccImbe7200x4400Data( imbe_fr, imbe_d );
        process_mbe( imbe_d, errors, errors2, algid, 0, 1 );

        bit_count = 0;
        voice_state = 7;
      }
      break;

    case  7 : //4 blocks of 6-bits data + 4-bits parity for 40 bits total
      if( bit_count < 40 ) {
        ldu_hex_parity[hex_bit_count++] = ( dreg & 0x03 );
      } else {
        bit_count = 0;
        voice_state = 8;
      }
      break;

    case  8 : //VOICE5
      deinterleave_frame( bit_count - 2, ( uint8_t )( dreg & 0x03 ) );
      if( bit_count == 144 ) {
        memset( imbe_d, 0x00, 88 );

        errors = p25_mbe_eccImbe7200x4400C0( imbe_fr );
        p25_mbe_demodulateImbe7200x4400Data( imbe_fr );
        errors2 = p25_mbe_eccImbe7200x4400Data( imbe_fr, imbe_d );
        process_mbe( imbe_d, errors, errors2, algid, 0, 1 );

        bit_count = 0;
        voice_state = 9;
      }
      break;

    case  9 : //4 blocks of 6-bits data + 4-bits parity for 40 bits total
      if( bit_count < 40 ) {
        ldu_hex_parity[hex_bit_count++] = ( dreg & 0x03 );  //idx 0, 1, 2, 5  are the encryption algid in LLDU2
      } else {

        //algid only present on HDU and LLDU2.  
        if(duid==0x0a) { //LLDU2
          uint8_t algid_lldu2=0;
          algid_lldu2 <<=2;
          algid_lldu2 |= ldu_hex_parity[0]; 
          algid_lldu2 <<=2;
          algid_lldu2 |= ldu_hex_parity[1]; 
          algid_lldu2 <<=2;
          algid_lldu2 |= ldu_hex_parity[2]; 
          algid_lldu2 <<=2;
          algid_lldu2 |= ldu_hex_parity[5]; 

          if(config->logging > 5) printf("\r\nalgid_llud2: %02x", algid_lldu2);

          if(algid_lldu2!=0x00) encrypted_count++;

          if(encrypted_count>3) {  //set threshold since algid_lldu2 is not error corrected
            printf("\r\nENCRYPTED.  Disabling talkgroup %d for %d ms", current_talkgroup, P25_IGNORE_ENCRYPTED_TG_TIMEOUT);
            algid = 0x80;

            add_encrypted_talkgroup(current_talkgroup);

            is_synced=0;
            return_to_control_mod=P25_CONTROL_TIMEOUT-1;  //force return to control channel
          }
          else {
            if(algid_lldu2==0x00) encrypted_count=0x00;
            found_header=1; //even if we didn't find header, this will flag to allow audio output
           // algid = algid_lldu2;  //silent output if algid is non-zero
          }

        }


        bit_count = 0;
        voice_state = 10;
      }
      break;

    case  10 :  //VOICE6
      deinterleave_frame( bit_count - 2, ( uint8_t )( dreg & 0x03 ) );
      if( bit_count == 144 ) {
        memset( imbe_d, 0x00, 88 );

        errors = p25_mbe_eccImbe7200x4400C0( imbe_fr );
        p25_mbe_demodulateImbe7200x4400Data( imbe_fr );
        errors2 = p25_mbe_eccImbe7200x4400Data( imbe_fr, imbe_d );
        process_mbe( imbe_d, errors, errors2, algid, 0, 1 );

        bit_count = 0;
        voice_state = 11;
      }
      break;

    case  11 :  //4 blocks of 6-bits data + 4-bits parity for 40 bits total
      if( bit_count < 40 ) {
        ldu_hex_parity[hex_bit_count++] = ( dreg & 0x03 );
      } else {
        bit_count = 0;
        voice_state = 12;
      }
      break;

    case  12 :  //VOICE7
      deinterleave_frame( bit_count - 2, ( uint8_t )( dreg & 0x03 ) );
      if( bit_count == 144 ) {
        memset( imbe_d, 0x00, 88 );

        errors = p25_mbe_eccImbe7200x4400C0( imbe_fr );
        p25_mbe_demodulateImbe7200x4400Data( imbe_fr );
        errors2 = p25_mbe_eccImbe7200x4400Data( imbe_fr, imbe_d );
        process_mbe( imbe_d, errors, errors2, algid, 0, 1 );

        bit_count = 0;
        voice_state = 13;
      }
      break;

    case  13 :  //4 blocks of 6-bits data + 4-bits parity for 40 bits total
      if( bit_count < 40 ) {
        ldu_hex_parity[hex_bit_count++] = ( dreg & 0x03 );
      } else {
        bit_count = 0;
        voice_state = 14;
      }
      break;

    case  14 :  //VOICE8
      deinterleave_frame( bit_count - 2, ( uint8_t )( dreg & 0x03 ) );
      if( bit_count == 144 ) {
        memset( imbe_d, 0x00, 88 );

        errors = p25_mbe_eccImbe7200x4400C0( imbe_fr );
        p25_mbe_demodulateImbe7200x4400Data( imbe_fr );
        errors2 = p25_mbe_eccImbe7200x4400Data( imbe_fr, imbe_d );
        process_mbe( imbe_d, errors, errors2, algid, 0 , 1);

        bit_count = 0;
        voice_state = 15;
      }
      break;

    case  15 :  //LSD=32
      if( bit_count < 32 ) {
      } else {
        bit_count = 0;
        voice_state = 16;
      }
      break;

    case  16 :  //VOICE9
      deinterleave_frame( bit_count - 2, ( uint8_t )( dreg & 0x03 ) );
      if( bit_count == 144 ) {
        memset( imbe_d, 0x00, 88 );

        errors = p25_mbe_eccImbe7200x4400C0( imbe_fr );
        p25_mbe_demodulateImbe7200x4400Data( imbe_fr );
        errors2 = p25_mbe_eccImbe7200x4400Data( imbe_fr, imbe_d );
        process_mbe( imbe_d, errors, errors2, algid, 1, 1 );

        bit_count = 0;
        voice_state = 0;
        duid = -1;

      }
      break;

    }

  }

end_decode:
 return;
}



///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
void deinterleave_frame( int dibit_n, uint8_t dibit_val )
{


  if( dibit_n == 0 ) stime = HAL_GetTick();

  if( dibit_n > 142 ) dibit_n = 0;

  int i = dibit_n / 2;

  imbe_fr[iW[i]][iX[i]] = ( 1 & ( dibit_val >> 1 ) ); // bit 1
  imbe_fr[iY[i]][iZ[i]] = ( 1 & dibit_val );      // bit 0
}


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
int p25_mbe_eccImbe7200x4400C0( char imbe_fr[8][23] )
{

  int j, errs;
  char in[23], out[23];

  for( j = 0; j < 23; j++ ) {
    in[j] = imbe_fr[0][j];
  }
  errs = mbe_golay2312( in, out );
  for( j = 0; j < 23; j++ ) {
    imbe_fr[0][j] = out[j];
  }

  return ( errs );

}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
void p25_mbe_demodulateImbe7200x4400Data( char imbe[8][23] )
{
  int i, j = 0, k;
  unsigned short pr[115], foo = 0;

  // create pseudo-random modulator
  for( i = 11; i >= 0; i-- ) {
    foo <<= 1;
    foo |= imbe[0][11 + i];
  }

  pr[0] = ( 16 * foo );
  for( i = 1; i < 115; i++ ) {
    pr[i] = ( 173 * pr[i - 1] ) + 13849 - ( 65536 * ( ( ( 173 * pr[i - 1] ) + 13849 ) >> 16 ) );
  }
  for( i = 1; i < 115; i++ ) {
    pr[i] >>= 15;
  }

  // demodulate imbe with pr
  k = 1;
  for( i = 1; i < 4; i++ ) {
    for( j = 22; j >= 0; j-- )
      imbe[i][j] ^= pr[k++];
  }
  for( i = 4; i < 7; i++ ) {
    for( j = 14; j >= 0; j-- )
      imbe[i][j] ^= pr[k++];
  }
}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
int p25_mbe_eccImbe7200x4400Data( char imbe_fr[8][23], char *imbe_d )
{
  int i, j, errs;
  char *imbe, gin[23], gout[23], hin[15], hout[15];

  errs = 0;
  imbe = imbe_d;
  for( i = 0; i < 4; i++ ) {
    if( i > 0 ) {
      for( j = 0; j < 23; j++ ) {
        gin[j] = imbe_fr[i][j];
      }
      errs += mbe_golay2312( gin, gout );
      for( j = 22; j > 10; j-- ) {
        *imbe = gout[j];
        imbe++;
      }
    } else {
      for( j = 22; j > 10; j-- ) {
        *imbe = imbe_fr[i][j];
        imbe++;
      }
    }
  }
  for( i = 4; i < 7; i++ ) {
    for( j = 0; j < 15; j++ ) {
      hin[j] = imbe_fr[i][j];
    }
    errs += mbe_hamming1511( hin, hout );
    for( j = 14; j >= 4; j-- ) {
      *imbe = hout[j];
      imbe++;
    }
  }
  for( j = 6; j >= 0; j-- ) {
    *imbe = imbe_fr[7][j];
    imbe++;
  }

  return ( errs );
}


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
void pack_data( char *in, char *out, int errors )
{
  int i, j, k;
  unsigned char b = 0;

  out[0] = ( uint8_t ) errors;
  //out[0] = 0; //no errors

  k = 0;
  for( i = 0; i < 11; i++ ) {

    for( j = 0; j < 8; j++ ) {
      b <<= 1;
      if( in[k++] ) b |= 1;
    }

    out[i + 1] = b;
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
void p25_processHDU( char *hex_and_parity )
{

  unsigned char hex_data[8];    // Data in hex-words (6 bit words). A total of 8 hex words.
  int i;

  errors = 0;
  for( i = 0; i < 8; i++ ) {
    // read both the hex word and the Golay(23, 12) parity information
    //read_dibit(opts, state, hex_and_parity, 9, &status_count);
    // Use the Golay23 FEC to correct it.
    hex_data[i] = hdu_correct_hex_word( ( unsigned char * ) &hex_and_parity[i * 9], 9 );
    if( gerr > 3 ) errors++; //uncorrectable
  }



  if( errors == 0 ) {
    found_header=1;

    //algid  = (((hex_data[1] >> 2) << 4) | (hex_data[2] & 0x0f));
    algid = ( hex_data[2] << 3 ) & 0xff;

    if(algid) {
      printf("\r\nENCRYPTED.  Disabling talkgroup %d for %d ms", current_talkgroup, P25_IGNORE_ENCRYPTED_TG_TIMEOUT);
      add_encrypted_talkgroup(current_talkgroup);
      is_synced=0;
      return_to_control_mod=P25_CONTROL_TIMEOUT-1;  //force return to control channel
    }

  } else {
    if( config->logging > 5 ) printf( "\r\nHDU contains errors" );
  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int hdu_correct_hex_word( unsigned char *hex_and_parity, unsigned int codeword_bits )
{
  unsigned int i, golay_codeword = 0;


  // codeword now contains:
  // bits 0-10: golay (23,12) parity bits
  // bits 11-22: hex bits
  for( i = 0; i < codeword_bits; i++ ) {
    golay_codeword <<= 2;
    golay_codeword |= hex_and_parity[i];
  }
  golay_codeword >>= 1;

  gerr = Golay23_CorrectAndGetErrCount( &golay_codeword );

  //if(gerr>3) if(config->logging>1) printf("\r\nhdu error: %d", gerr);

  return golay_codeword;
}


///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
static unsigned int GolayGenerator[12] = {
  0x63a, 0x31d, 0x7b4, 0x3da, 0x1ed, 0x6cc, 0x366, 0x1b3, 0x6e3, 0x54b, 0x49f, 0x475
};

///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
void Golay23_Correct( unsigned int *block )
{
  unsigned int i, syndrome = 0;
  unsigned int mask, block_l = *block;

  mask = 0x400000l;
  for( i = 0; i < 12; i++ ) {
    if( ( block_l & mask ) != 0 ) {
      syndrome ^= GolayGenerator[i];
    }
    mask = mask >> 1;
  }

  syndrome ^= ( block_l & 0x07ff );
  *block = ( ( block_l >> 11 ) ^ GolayMatrix[syndrome] );
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void processTSBK(unsigned char out[12]) {

  int manf_id=0;
  int iden=0;
  int iden_to_use=0;
  int bw=0;
  uint32_t tx_off_sign=0;
  int32_t tx_off_vu=0;
  int spacing=0;
  uint32_t base_freq=0;
  uint32_t freq=0;
  double freq_f=0;
  int opcode=0;
  int priority=0;
  int enabled=0;
  uint8_t *descptr=NULL;
  uint8_t *locptr=NULL;
  int priority_1=0;
  int priority_2=0;
  int using_second=0;
  uint16_t tgroup=0;
  uint32_t channel2=0;
  uint32_t channel_iden=0;
  uint32_t channel_iden2=0;
  uint16_t tgroup1=0;
  uint16_t tgroup2=0;
  int enabled_1=0;
  int enabled_2=0;
  uint16_t sys_id=0;
  uint32_t src_id=0;
  uint32_t src_addr=0;
  int btf=0;
  uint16_t channel_tx=0;
  uint16_t channel_rx=0;
  uint16_t talkgrp=0;
  uint32_t channel=0;


    //TSBK uses this, feed it first 10, equals last 2
  uint16_t crc = CRCCCITT( out, 10, 0x0000, 0xffff);
  uint16_t crc_check = out[10]<<8 | out[11];
  if(crc==crc_check) {
    if(config->logging>5) printf("\r\ncrc: %04x,  %04x", crc, crc_check);
  }
  else {
    if(config->logging>5) printf("\r\ncrc NOT OK: %04x,  %04x", crc, crc_check);
    return;
  }


  //printf("\r\nTSBK count %d", tsbk_count++);
 
    opcode  = out[0]&0x3f;


    if(opcode==0x34) {  //IDEN_UP_VU

      manf_id = out[1];
      iden = out[2]>>4;
      bw = (out[2]&0x0f);

      if(bw == 0x04) bw = 6250;
      if(bw == 0x05) bw = 12500;

      tx_off_sign = out[3]&0x80;

      tx_off_vu = out[3]<<6; 
      tx_off_vu |= (out[4]>>2);

      if(tx_off_sign==0) tx_off_vu *= -1;

      spacing = (out[4]&0x03)<<8;
      spacing |= out[5];

      base_freq = out[6]<<24 | out[7]<<16 | out[8] <<8 | out[9];

      bfreq_ptr = (base_freq_iden *) &bfreq_iden[iden];
      bfreq_ptr->iden = iden;
      bfreq_ptr->base_freq = base_freq*5;
      bfreq_ptr->bw = bw;
      bfreq_ptr->tx_off_vu = tx_off_vu;
      bfreq_ptr->spacing = spacing*125; 

      if(config->logging>4) printf("\r\niden_vu: %d, manf_id %d, base_freqx5 %u,  tx_off_vu %d,  spacingx125 %d", iden, manf_id, base_freq*5, tx_off_vu, spacing*125);

    }
    else if(opcode==0x3d) { //IDEN_UP

      manf_id = out[1];
      iden = out[2]>>4;
      bw = (out[2]&0x0f)<<5;
      bw |= out[3]>>3;

      tx_off_sign = out[3]&0x04;

      tx_off_vu = (out[3]&0x07)<<6; 
      tx_off_vu |= (out[4]>>2);
      if(tx_off_sign==0) tx_off_vu *= -1;

      spacing = (out[4]&0x03)<<8;
      spacing |= out[5];

      base_freq = out[6]<<24 | out[7]<<16 | out[8] <<8 | out[9];

      bfreq_ptr = (base_freq_iden *) &bfreq_iden[iden];
      bfreq_ptr->iden = iden;
      bfreq_ptr->base_freq = base_freq*5;
      bfreq_ptr->bw = bw*125;
      bfreq_ptr->tx_off_vu = tx_off_vu;
      bfreq_ptr->spacing = spacing*125;

      if(config->logging>4) printf("\r\niden_up: %d, manf_id %d, base_freqx5 %u,  tx_off_vu %d,  spacingx125 %d", iden, manf_id, base_freq*5, tx_off_vu, spacing*125);

    }
    else if(opcode==0x00 ) {  //group voice grant
      manf_id = out[1];
      tgroup = out[5]<<8 | out[6];
      channel = (out[3]&0x0f)<<8 | out[4];
      channel_iden = out[3]>>4;


      if( is_encrypted_talkgroup(tgroup)) {
        if(config->logging>4) printf("\r\nignoring encrypted talkgroup %d", tgroup);
        return;
      }

      p25_time_since_grant=0;

      iden_to_use = channel_iden; 

      if(iden_to_use<0) {
        printf("\r\nwarning:  could not find record for tg %d associated with sys_id %d / nac %d.  can't determine base_freq_iden.  please add to table.", tgroup, config->p25_sys_id, current_nac);
        return;
      }

      if(iden_to_use >= 0) {

        bfreq_ptr = (base_freq_iden *) &bfreq_iden[iden_to_use];

        if(bfreq_ptr->base_freq==0) return;

        freq = bfreq_ptr->base_freq + (channel*bfreq_ptr->spacing);

        freq_f = (double) freq*10e-7;

        enabled = p25_is_tg_enabled(tgroup, &priority);
        if(enabled==0 && config->p25_follow!=tgroup) {
          if(config->logging>4) printf("\r\n (talkgroup %d disabled. skipping.)", tgroup);
          return;
        }

        if(freq==0) return;

        printf("\r\n\r\n  GRP_V_CH_GRANT 'group voice grant' (0x00)");
        printf("\r\n  ==============================================================================================");
        printf("\r\n  packet_idx %d,  tgroup %d:  freq %4.4f  nac %d  rf_channel %d", tsdu_blocks, tgroup, freq_f, current_nac, channel);

        descptr = p25_get_desc(tgroup);

        if(descptr==NULL) {
          memset(current_desc,0x00,32);
          strncpy(current_desc, "(unknown talkgroup)", 31);
          printf("  %s  %d  ", current_desc, tgroup);
        }
        else {
          memcpy( (uint8_t *) current_desc, (uint8_t *) descptr, 31);
          printf("\r\n  desc: %s", current_desc);
        }

        locptr = p25_get_loc(tgroup);

        if(locptr==NULL) {
          memset(current_loc,0x00,16);
          strncpy(current_loc, "(unknown location)", 15);
          printf("  %s  %d  ", current_loc, tgroup);
        }
        else {
          memcpy( (uint8_t *) current_loc, (uint8_t *) locptr, 15);
          printf("    location: %s", current_loc);
        }


        if( config->p25_follow>0 && tgroup!=config->p25_follow) {
          if(config->logging>4) printf("\r\nfollow is >zero and tgroup doesn't match.  skipping");
          return;  
        }
        if( config->p25_follow<0 && tgroup==abs(config->p25_follow)) {
          if(config->logging>4) printf("\r\nfollow is <zero and tgroup matches.  skipping");
          return;  
        }

        if( priority > current_priority ) { 
          current_talkgroup = tgroup;
          return_to_control_mod=1;
          goto_freq=freq_f;
          is_synced=1200;
          p25_reset_tick=0;
          current_priority = priority;
          previous_talkgroup = tgroup;
        }
        
        //is last block?
        //if( (out[0]&0x80) || tgroup==config->p25_follow || priority > 4) do_goto_freq=1;
        do_goto_freq=1; //we don't need to wait for last block on 0x00 type grant
      }
    }
    else if(opcode==0x02  ) {  //group voice grant update.  local system seems to use only 0x00 and 0x02 grant op-codes 
      using_second=0;

      manf_id = out[1];
      channel = (out[2]&0x0f)<<8 | out[3];
      channel_iden = out[2]>>4;
      tgroup1 = out[4]<<8 | out[5];

      //this grant carries two different records   - might have the same info or not...
      channel2 = (out[6]&0xf)<<8 | out[7];
      channel_iden2 = out[6]>>4;
      tgroup2 = out[8]<<8 | out[9];

      enabled=-1;
      enabled_1 = p25_is_tg_enabled(tgroup1, &priority_1);
      enabled_2 = p25_is_tg_enabled(tgroup2, &priority_2);

      if( is_encrypted_talkgroup(tgroup1)) {
        //if(config->logging>1) printf("\r\nignoring encrypted talkgroup %d", tgroup);
        enabled_1 = 0;
      }
      if( is_encrypted_talkgroup(tgroup2)) {
        //if(config->logging>1) printf("\r\nignoring encrypted talkgroup %d", tgroup);
        enabled_2 = 0;
      }

      //TODO-DONE:  process the channel2 info
      //  1)  check if both enabled
      //  2)  get priorities of both tg/channels
      //  3)  part of the priority should be if we have the chance to follow the previous GOTO
      //  4)  decide which one gets passed on
      if(enabled_1>0 && enabled_2>0) {
        if(tgroup1==previous_talkgroup) {
          priority = priority_1;
          tgroup = tgroup1;
          enabled=1;
        }
        else if(tgroup2==previous_talkgroup) {
          priority = priority_2;
          tgroup = tgroup2;
          enabled=1;
          using_second=1;
        }
        else if( priority_1 >= priority_2) {
          priority = priority_1;
          tgroup = tgroup1;
          enabled=1;
        }
        else {
          priority = priority_2;
          tgroup = tgroup2;
          channel = channel2;
          channel_iden = channel_iden2;
          using_second=1;
          enabled=1;
        }
      }
      else if( enabled_2>0) {
        priority = priority_2;
        tgroup = tgroup2;
        channel = channel2;
        channel_iden = channel_iden2;
        using_second=1;
        enabled=1;
      }
      else if( enabled_1>0) {
        tgroup = tgroup1;
        priority = priority_1;
        enabled=1;
      }
      else {
        if(enabled_1==0) enabled=0;
        if(enabled_2==0) enabled=0;
        tgroup = tgroup1;
        priority = priority_1;
      }



      p25_time_since_grant=0;

      iden_to_use = channel_iden; 

      if(iden_to_use<0) {
        printf("\r\nwarning:  could not find record for tg %d associated with sys_id %d.  can't determine base_freq_iden.  please add to table.", tgroup, config->p25_sys_id);
        return;
      }

      if(iden_to_use >= 0) {

        bfreq_ptr = (base_freq_iden *) &bfreq_iden[iden_to_use];
        if(bfreq_ptr->base_freq==0) return;

        freq = bfreq_ptr-> base_freq + (channel*bfreq_ptr->spacing);


        freq_f = (double) freq*10e-7;

        if(config->p25_grant < 0x02) {
          if(config->logging>1) printf("\r\n (grant 0x02 disabled. skipping.)");
          return;
        }


        if(enabled==0 && config->p25_follow!=tgroup) {
          if(config->logging>4) printf("\r\n (talkgroup %d disabled. skipping.)", tgroup);
          return;
        }

        if(freq==0) goto grant2_exit;

        printf("\r\n\r\n  GRP_V_CH_GRANT_UPDT 'group voice grant - conversation already in progress' (0x02)");
        if(using_second) printf("\r\n  ->using second record.  dropped tg %d  ", tgroup1);
        printf("\r\n  ==============================================================================================");
        printf("\r\n  packet_idx %d,  tgroup %d:  freq %4.4f  nac %d  rf_channel %d", tsdu_blocks, tgroup, freq_f, current_nac, channel);

        descptr = p25_get_desc(tgroup);

        if(descptr==NULL) {
          memset(current_desc,0x00,32);
          strncpy(current_desc, "(unknown talkgroup)", 31);
          printf("  %s  %d  ", current_desc, tgroup);
        }
        else {
          memcpy( (uint8_t *) current_desc, (uint8_t *) descptr, 31);
          printf("\r\n  desc: %s", current_desc);
        }

        locptr = p25_get_loc(tgroup);

        if(locptr==NULL) {
          memset(current_loc,0x00,16);
          strncpy(current_loc, "(unknown location)", 15);
          printf("  %s  %d  ", current_loc, tgroup);
        }
        else {
          memcpy( (uint8_t *) current_loc, (uint8_t *) locptr, 15);
          printf("    location: %s", current_loc);
        }


        if( config->p25_follow>0 && tgroup!=config->p25_follow) {
          if(config->logging>4) printf("\r\nfollow is >zero and tgroup doesn't match.  skipping");
          return;  
        }
        if( config->p25_follow<0 && tgroup==abs(config->p25_follow)) {
          if(config->logging>4) printf("\r\nfollow is <zero and tgroup matches.  skipping");
          return;  
        }

        if( priority >= current_priority ) { 
          current_talkgroup = tgroup;
          return_to_control_mod=1;
          goto_freq=freq_f;
          is_synced=1200;
          p25_reset_tick=0;
          current_priority = priority;
        }
grant2_exit:  
        if(current_priority!=INT_MIN) {
          ////is last block?
          //if( (out[0]&0x80) || tgroup==config->p25_follow || priority > 4) do_goto_freq=1;
          do_goto_freq=1;
          previous_talkgroup = tgroup;
        }
      }

    }
    else if(opcode==44) {  //unit registration response
      manf_id = out[1];
      sys_id = (out[2]&0x1f)<<8;
      sys_id |= out[3];
      src_id = out[4]<<24 | out[5]<<16 | out[6];
      src_addr = out[7]<<24 | out[8]<<16 | out[9];
      if(config->logging>4) printf("\r\nunit reg resp (44),  manf_id %d,  sys_id %d, src_id %d, src_addr %d", manf_id, sys_id, src_id, src_addr);
      if(sys_id != config->p25_sys_id) printf("\r\nwarning, unit registered with sys_id %d  and p25_sys_id for this channel is set to %d", sys_id, config->p25_sys_id);
    }
    else if(out[2]==0x00 && out[7]==0x03) {  //group voice grant explicit multi-block header 

      //TODO:  this appears to happen on the local network around 5:30 pm... probably to update all radio units
      if(config->logging > 4) printf("\r\ngroup voice grant explicit multi-block header (0x%02x):  - not handled  ", out[7]);
        int i;
        //for(i=0;i<12;i++) {
         // printf("%02x, ", out[i]);
        //}

    }
    else if(out[2]==0x00 && out[7]==59) {  //network status broadcast multi-part
      sys_id = out[4]<<4 | out[5];
      btf = out[6]&0x7f;
      if(config->logging>4) printf("\r\nnetwork status broadcast, multi-block header, sys_id %d,  btf %d", sys_id, btf);
    }
    else if(opcode==3) {  //grant update explicit
      channel_tx = out[4]<<8 | out[5];
      channel_rx = out[6]<<8 | out[7];
      talkgrp = out[8]<<8 | out[9];
      if(config->logging>4) printf("\r\ngrant update explicit:  TX: %d    RX: %d  TGRP: %d", channel_tx, channel_rx, talkgrp);

    }
    else if(opcode==0x35) {  //TIME_DATE_ANN
      if(config->logging>4) printf("\r\nTIME UPDATE ANNOUNCEMENT");
      //TODO: finishing filling this out if ever find a channel that broadcasts it 
    }
    else {
        //printf("\r\nunknown op-code (0x%02x):  ", opcode);
        //int i;
        //for(i=0;i<12;i++) {
          //printf("%02x, ", out[i]);
        //}

    }

}
