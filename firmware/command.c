

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



#include <string.h>
#include <math.h>
#include <float.h>
#include <complex.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "main.h"
#include "globals.h"
#include "command.h"
#include "config.h"
#include "telnet.h"
#include "std_io.h"
#include "telnet.h"
#include "main.h"
#include "agc.h"
#include "freqdem_cfrf_arm32.h"
#include "ampmodem_arm32.h"
#include "nco_cf_arm32.h"
#include "dmr_state.h"

uint8_t USBD_StringSerial[128];
static int reset_timer;
static int at_scan_timer;
void set_freq_mhz( double val );
static uint32_t prim;


extern I2S_HandleTypeDef hi2s2;
extern ALIGN_32BYTES( volatile uint16_t   audio_tx[256] __attribute__( ( section( ".AudioSection" ) ) ) ); //I2S audio buffer

const char * sa_mode_to_str(int val);
const char sa_modes[9][9] = { "fm", "iq", "p25", "am", "mode4", "adsb", "mode6", "dmr", "syncmon" };

const char * udp_mode_to_str(int val);
const char udp_modes[9][32] = { "off", "adc_raw", "iq16_decflt", "demod", "4fsk_sym_s16", 
                                "4fsk_dibit", "audio_48k_s16","audio_48k_s16_cont","voice8k","console"};

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
void parse_command( char *cmd )
{

  char cmdstr[CMD_BUFFER_LEN];
  strncpy( cmdstr, cmd, CMD_BUFFER_LEN );


  cmdstr[CMD_BUFFER_LEN - 1] = 0x00;

  char *ptr = cmdstr;

  //trim off end of cmd string
  ptr += strlen( ( char * ) cmdstr ) - 5;
  if( *ptr == '\r' || *ptr == '\n' ) *ptr = 0x00;
  ptr++;
  if( *ptr == '\r' || *ptr == '\n' ) *ptr = 0x00;
  ptr++;
  if( *ptr == '\r' || *ptr == '\n' ) *ptr = 0x00;
  ptr++;
  if( *ptr == '\r' || *ptr == '\n' ) *ptr = 0x00;


  const char s[2] = " "; //separator
  char *token;

  //eight arguments should be enough for anybody
  char *tok1 = NULL;
  char *tok2 = NULL;
  char *tok3 = NULL;
  char *tok4 = NULL;
  char *tok5 = NULL;
  char *tok6 = NULL;
  char *tok7 = NULL;
  char *tok8 = NULL;

  int toks = 0;

  token = strtok( cmdstr, s );
  while( token != NULL && toks < 8 ) {
    switch( ++toks ) {
    case 1 :
      tok1 = token;
      break;
    case 2 :
      tok2 = token;
      break;
    case 3 :
      tok3 = token;
      break;
    case 4 :
      tok4 = token;
      break;
    case 5 :
      tok5 = token;
      break;
    case 6 :
      tok6 = token;
      break;
    case 7 :
      tok7 = token;
      break;
    case 8 :
      tok8 = token;
      break;
    }
    token = strtok( NULL, s );
  }

#if 0
  if( tok1 != NULL ) printf( "\r\ntok1: %s", tok1 );
  if( tok2 != NULL ) printf( "\r\ntok2: %s", tok2 );
  if( tok3 != NULL ) printf( "\r\ntok3: %s", tok3 );
  if( tok4 != NULL ) printf( "\r\ntok4: %s", tok4 );
  if( tok5 != NULL ) printf( "\r\ntok5: %s", tok5 );
  if( tok6 != NULL ) printf( "\r\ntok6: %s", tok6 );
  if( tok7 != NULL ) printf( "\r\ntok7: %s", tok7 );
  if( tok8 != NULL ) printf( "\r\ntok8: %s", tok8 );
#endif

  int do_prompt = 1;
  int val1;

  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  if( strcmp( tok1, ( const char * ) "defaults" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();


    reset_config_to_defaults();

    set_bbatten( config->bb_atten );
    set_gain1( config->gain1 );
    set_atten( config->front_end_atten );
    set_bw( config->adc_flt_bw );

    reconfig_adc( config->channel_bw ); //narrow bw

    set_freq_mhz2(config->if_frequency);
    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "ip_addr" ) == 0 ) {

    const char ip_s[2] = "."; //separator
    char *ip_token;

    char *ip_tok1 = NULL;
    char *ip_tok2 = NULL;
    char *ip_tok3 = NULL;
    char *ip_tok4 = NULL;

    int ip_toks = 0;

    if(toks==2) {
      ip_token = strtok( tok2, ip_s );
      while( ip_token != NULL && ip_toks < 4 ) {
        switch( ++ip_toks ) {
        case 1 :
          ip_tok1 = ip_token;
          break;
        case 2 :
          ip_tok2 = ip_token;
          break;
        case 3 :
          ip_tok3 = ip_token;
          break;
        case 4 :
          ip_tok4 = ip_token;
          break;
      }
      ip_token = strtok( NULL, ip_s );
     }
    }

    if(ip_toks==4) {
      int ip1 = atoi(ip_tok1);
      int ip2 = atoi(ip_tok2);
      int ip3 = atoi(ip_tok3);
      int ip4 = atoi(ip_tok4);

      config->ip_addr[0] = (uint8_t) ip1;
      config->ip_addr[1] = (uint8_t) ip2;
      config->ip_addr[2] = (uint8_t) ip3;
      config->ip_addr[3] = (uint8_t) ip4;
    }

    printf("\r\nip_addr %d.%d.%d.%d", config->ip_addr[0],config->ip_addr[1],config->ip_addr[2],config->ip_addr[3]); 

  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "udp_send_addr" ) == 0 ) {

    const char ip_s[2] = "."; //separator
    char *ip_token;

    char *ip_tok1 = NULL;
    char *ip_tok2 = NULL;
    char *ip_tok3 = NULL;
    char *ip_tok4 = NULL;

    int ip_toks = 0;

    if(toks==2) {
      ip_token = strtok( tok2, ip_s );
      while( ip_token != NULL && ip_toks < 4 ) {
        switch( ++ip_toks ) {
        case 1 :
          ip_tok1 = ip_token;
          break;
        case 2 :
          ip_tok2 = ip_token;
          break;
        case 3 :
          ip_tok3 = ip_token;
          break;
        case 4 :
          ip_tok4 = ip_token;
          break;
      }
      ip_token = strtok( NULL, ip_s );
     }
    }

    if(ip_toks==4) {
      int ip1 = atoi(ip_tok1);
      int ip2 = atoi(ip_tok2);
      int ip3 = atoi(ip_tok3);
      int ip4 = atoi(ip_tok4);

      config->udp_ip_addr[0] = (uint8_t) ip1;
      config->udp_ip_addr[1] = (uint8_t) ip2;
      config->udp_ip_addr[2] = (uint8_t) ip3;
      config->udp_ip_addr[3] = (uint8_t) ip4;

      if(ip1==0 && ip2==0 && ip3==0 && ip4==0) {
        config->udp_ip_addr[0] = (uint8_t) udp_send_ip_bak>>24&0xff; 
        config->udp_ip_addr[1] = (uint8_t)  udp_send_ip_bak>>16&0xff;
        config->udp_ip_addr[2] = (uint8_t)  udp_send_ip_bak>>8&0xff;;
        config->udp_ip_addr[3] = (uint8_t)  udp_send_ip_bak&0xff;;
      }
    }


    printf("\r\nudp_send_addr %d.%d.%d.%d", config->udp_ip_addr[0],config->udp_ip_addr[1],config->udp_ip_addr[2],config->udp_ip_addr[3]); 

  }
  ///////////////////////////////////////////////////////////////
  // AUDIO
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "audio" ) == 0 ) {


    val1 = command_is_one( tok2 ); //on/off 0/1
    if( val1 >= 0 && toks == 2 ) {
      config->audio_on = val1;
      printf( "\r\naudio %d", config->audio_on );
    } else if( strncmp( tok2, "vol", 3 ) == 0 ) {

      if( toks == 3 ) {
        config->audio_volume_f = atof( tok3 ); //audio vol xxx.xf, 0.0 to 1.0

        if( config->audio_volume_f < 0.0f ) config->audio_volume_f = 0.0f;
        else if( config->audio_volume_f > 1.0f ) config->audio_volume_f = 1.0f;
        else if( isnan( config->audio_volume_f ) ) config->audio_volume_f = 0.75f;
      }

      printf( "\r\naudio vol %3.2f", config->audio_volume_f );
    } else if( toks == 1 ) {
      printf( "\r\naudio %d, audio vol %3.2f", config->audio_on, config->audio_volume_f );
    }
  }
  ///////////////////////////////////////////////////////////////
  // IF FREQ
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "if_freq" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    //read-only.  use if_low_high command to determine which if_frequency to use
    double val = config->if_frequency;
    printf( "\r\nif_freq %f", val );
    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "p25_audio_gain" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    double val = config->frequency;

    if( toks == 2 ) {
      val = ( double ) atof( tok2 );
      config->p25_audio_gain = val;
    }
    printf( "\r\np25_audio_gain %f", config->p25_audio_gain );
    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  // FREQ offset
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "freq_off" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    double val = config->freq_offset_mhz;

    if( toks == 2 ) {
      config->freq_offset_mhz = ( float ) atof( tok2 );
    }
    printf( "\r\nfreq_off %f", config->freq_offset_mhz );

    set_freq_mhz( config->frequency + config->if_frequency );

    float f1 =  (config->freq_offset_mhz*1e6)  / CH_NARROW_RATE_DIV4;
    f1 *= M_PI;
    f1 /= 2.0f;
    f1 *= -1.0f;

    if(config->channel_bw==0) printf("\r\nsuggest using nco value of %3.5f", f1);

    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "serialn" ) == 0 ) {
    Get_SerialNum();
    printf("\r\nserial number: %s", &USBD_StringSerial[0] );
  }
  ///////////////////////////////////////////////////////////////
  // FREQ
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "freq" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    double val = config->frequency;

    if( toks == 2 ) {
      val = ( double ) atof( tok2 );
      if( val < 0.5 || val > 1300.0 ) {
        printf( "\r\nfreq must be in the range of 0.5 to 1300.0" );
      } else {
        config->frequency = val;
        set_freq_mhz( val + config->if_frequency );
      }
    }
    printf( "\r\nfreq %f", val );
    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  // hf_val
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "hf_val" ) == 0 ) {
    printf("\r\nuse 'aud_dc_off_rate'");
    print_prompt();
  }
  ///////////////////////////////////////////////////////////////
  // aud_dc_off_rate
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "aud_dc_off_rate" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      double val = ( double ) atof( tok2 );
      if( val != 0 && ( val < 5 || val > 96000 ) ) {
        val = 128.0;
        printf( "\r\naud_dc_off_rate val >=5 and <96000. 0 to disable" );
      }
      config->aud_dc_off_rate = val;
    }

    init_audio_dc_offset( config->aud_dc_off_rate );

    printf( "\r\naud_dc_off_rate %f", config->aud_dc_off_rate );
    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  // nco
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "nco" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      float val = ( float ) atof( tok2 );
      nco_set_frequency( nco, val );
      config->nco_val = val;
    }

    printf( "\r\nnco %f", config->nco_val );
    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  // nco offset freq
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "nco_off" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      float val = ( float ) atof( tok2 );
      config->nco_offset_freq = val;

      set_freq_mhz( config->frequency + config->if_frequency );
    }

    printf( "\r\nnco_off freq %f", config->nco_offset_freq );
    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  // fmdev
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "fmdev" ) == 0 ) {

    if( toks == 2 ) {
      float val = ( float ) atof( tok2 );
      if(val < 0.1) val = 0.1;
      if(val > 0.9) val = 0.9;

      config->fm_deviation = val;
      freqdem_init( config->fm_deviation);
    } 
    printf( "\r\nfmdev %f", config->fm_deviation );

  }
  else if( strcmp( tok1, ( const char * ) "dcalpha" ) == 0 ) {
    printf("\r\nuse iq_dc_off_rate");
    print_prompt();
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "iq_dc_off_rate" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    float val = config->iq_dc_off_rate;

    if( toks == 2 ) {
      val = ( float ) atof( tok2 );
      if( val < 64 || val > 32768 ) {
        val = 800.0;
        printf( "\r\niq_dc_off_rate val >63 and <32768." );
      }
      config->iq_dc_off_rate = val;

      init_dc_correction( ( double ) config->iq_dc_off_rate );
    }

    printf( "\r\niq_dc_off_rate %f", config->iq_dc_off_rate );
    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "follow" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );

      config->p25_follow = val;
    }

    printf( "\r\nfollow %d", config->p25_follow );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "p25_sys_id" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );

      config->p25_sys_id = val;
    }

    printf( "\r\np25_sys_id %d", config->p25_sys_id );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "if_low_high" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    int val=0;
    if( toks == 2 ) {
      val = ( int ) atoi( tok2 );
      config->if_low_high = val;
    }
    if(val!=0 && val!=1) {
      val=0; 
      printf("\r\nif_low_high must be 0 or 1");
    }

    if(config->if_low_high==0) config->if_frequency=1552.0;
    if(config->if_low_high==1) config->if_frequency=1583.0;

    printf( "\r\nif_low_high %d,  if freq: %4.6f", config->if_low_high, config->if_frequency );

    set_freq_mhz2(config->if_frequency);
    set_freq_mhz(config->if_frequency + config->frequency);

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "aud_agc" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();
    if( toks == 2 ) {
      float val = ( float ) atof( tok2 );

      config->aud_agc = val;
    }

    if(config->aud_agc<0.01f) config->aud_agc=0.01f;
    if(config->aud_agc>0.99f) config->aud_agc=0.01f;

    printf( "\r\naud_agc %4.4f", config->aud_agc );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "aud_agc_en" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );

      config->aud_agc_en = val;
    }

    printf( "\r\naud_agc_en %d", config->aud_agc_en );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "p25_grant" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );

      config->p25_grant = val;
    }

    printf( "\r\np25_grant %d", config->p25_grant );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "is_control" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );

      config->is_control = val;
    }

    printf( "\r\nis_control %d", config->is_control );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "dmr_level" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );

      config->dmr_level_max = val;
    }

    if(config->dmr_level_max < 3000) config->dmr_level_max=5000;
    if(config->dmr_level_max > 7000) config->dmr_level_max=5000;

    printf( "\r\ndmr_level %d", config->dmr_level_max );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "adc3_frac" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );

      config->adc3_frac = val;
    }

    if(config->adc3_frac < 1) config->adc3_frac=1;
    if(config->adc3_frac > 8000) config->adc3_frac=8000;

    printf( "\r\nadc3_frac %d", config->adc3_frac );

    reconfig_adc( config->channel_bw );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  // lowif
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "lowif" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );

      config->do_low_if = val;
    }
    printf( "\r\nlowif %d", config->do_low_if );

    set_freq_mhz( config->frequency + config->if_frequency ); //reset freq because do_low_if_mix might have changed

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  // decimation filter M
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "dec" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );

      config->decimate = val;
      udp_out_len = 0;
    }
    printf( "\r\ndec %d", config->decimate );
    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "config_t" ) == 0 ) {
    printf( "\r\nconfig_t size: %d", sizeof( config_t ) );
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "mcu_ver" ) == 0 ) {
    printf( "\r\nmcu_ver %08x", config->mcu_ver );
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "mcu_id" ) == 0 ) {
    printf( "\r\nmcu_id %08x", config->mcu_unique_id );
  }
  ///////////////////////////////////////////////////////////////
  // adc filter
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "adcflt" ) == 0 ) {
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );
      set_bw( val );
    }
    printf( "\r\nadcflt %d", config->adc_flt_bw );
  }
  ///////////////////////////////////////////////////////////////
  // atten  front-end attenuator  (0-31)
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "atten" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    int val = 0;
    if( toks == 2 ) {
      val = ( int ) atoi( tok2 );
      val &= 0x3f;
      config->front_end_atten = val;
      set_atten( config->front_end_atten );
    }
    printf( "\r\natten %d", config->front_end_atten );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  // channel_bw
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "channel_bw" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );
      config->channel_bw = val;
      reconfig_adc( config->channel_bw );
    }
    printf( "\r\nchannel_bw %d", config->channel_bw );


    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "rssi_offset" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );
      config->rssi_offset = val;
      rssi_offset = val;
    }
    printf( "\r\nrssi_offset %d", config->rssi_offset );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  // ADSB
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "adsb" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );
      config->do_adsb = val;
    }
    printf( "\r\ndo_adsb %d", config->do_adsb );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  // AM gain
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "am_gain" ) == 0 ) {

    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      float am_gain = ( float ) atof( tok2 );
      config->am_gain = am_gain;
    }
    printf( "\r\nam_gain %f", config->am_gain );

    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  // audio filter
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "flt" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );
      config->audio_filter = val;
    }
    printf( "\r\nflt %d", config->audio_filter );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  // print rssi
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "rssi" ) == 0 ) {
    printf( "\r\nrssi: %d dBm, %3.1f dBFS, front-end atten: %d, bbatten %d, gain1 %d", current_rssi, dBFS, config->front_end_atten, config->bb_atten, config->gain1 );
  }
  ///////////////////////////////////////////////////////////////
  // print rssi
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "show_rssi" ) == 0 ) {
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );
      config->show_rssi = val;
    }
    printf( "\r\nshow_rssi %d", config->show_rssi );
  }
  ///////////////////////////////////////////////////////////////
  // print adc srate
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "show_srate" ) == 0 ) {
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );
      config->show_srate = val;
    }
    printf( "\r\nshow_srate %d", config->show_srate );
  }
  ///////////////////////////////////////////////////////////////
  // synth1 gain
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "s1gain" ) == 0 ) {

    uint8_t gain = 0;
    if( toks == 2 ) {
      gain = ( uint8_t ) atoi( tok2 );
      gain &= 0x03;
      gain <<= 3;
      uint8_t cgain = synth_read_reg( 0x0b );
      cgain &= 0xe7;
      cgain |= gain;
      synth_write_reg( 0x0b, cgain );  //update gain
    }

    uint8_t val = synth_read_reg( 0x0b );
    val >>= 3;
    val &= 0x03;
    printf( "\r\ns1gain %d", val );

  }
  ///////////////////////////////////////////////////////////////
  // synth2 gain
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "s2gain" ) == 0 ) {

    uint8_t gain = 0;
    if( toks == 2 ) {
      gain = ( uint8_t ) atoi( tok2 );
      gain &= 0x03;
      gain <<= 3;
      uint8_t cgain = synth2_read_reg( 0x0b );
      cgain &= 0xe7;
      cgain |= gain;
      synth2_write_reg( 0x0b, cgain );  //update gain
    }

    uint8_t val = synth2_read_reg( 0x0b );
    val >>= 3;
    val &= 0x03;
    printf( "\r\ns2gain %d", val );

  }
  ///////////////////////////////////////////////////////////////
  // gain1
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "gain1" ) == 0 ) {

    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );
      bb_gain = val; //over-ride
      set_gain1( val );
    }
    printf( "\r\ngain1 %d", config->gain1 );

  }
  ///////////////////////////////////////////////////////////////
  // dcoff
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "dcoff" ) == 0 ) {

    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 );
      config->dc_off = val;
    }
    printf( "\r\ndcoff %d", config->dc_off );

  }
  ///////////////////////////////////////////////////////////////
  // IGAIN
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "igain" ) == 0 ) {


    if( toks == 2 ) {
      float _igain = ( float ) atof( tok2 );
      config->i_gain = _igain;
    }
    printf( "\r\nigain %f", config->i_gain );

  }
  ///////////////////////////////////////////////////////////////
  // QOFF
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "qgain" ) == 0 ) {


    if( toks == 2 ) {
      float _qgain = ( float ) atof( tok2 );
      config->q_gain = _qgain;
    }
    printf( "\r\nqgain %f", config->q_gain );

  }
  ///////////////////////////////////////////////////////////////
  // IOFF
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "ioff" ) == 0 ) {

    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      float _ioff = ( float ) atof( tok2 );
      config->i_off = _ioff;
    }
    printf( "\r\nioff %f", config->i_off );

    if( !prim ) {
      __enable_irq();
    }
  }
  ///////////////////////////////////////////////////////////////
  // QOFF
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "qoff" ) == 0 ) {

    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      float _qoff = ( float ) atof( tok2 );
      config->q_off = _qoff;
    }
    printf( "\r\nqoff %f", config->q_off );

    if( !prim ) {
      __enable_irq();
    }


  }
  ///////////////////////////////////////////////////////////////
  // RFGAIN
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "rfgain" ) == 0 ) {


    if( toks == 2 ) {
      int _gain = ( int ) atoi( tok2 );
      if( _gain >= 0 && _gain <= 23 ) {
        config->rfgain = _gain;
        //at86rf215_set_gain(config->rfgain);
      }
    }
    printf( "\r\nrfgain %d", config->rfgain );

  }
  ///////////////////////////////////////////////////////////////
  // SQUELCH
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "squelch" ) == 0 ) {

    val1 = command_is_one( tok2 ); //on/off 0/1
    if( val1 >= 0 && toks == 2 ) {
      config->squelch = val1;
    } else if( toks == 2 ) {
      config->squelch = ( int ) atoi( tok2 ); //more detailed logging
    }
    printf( "\r\nsquelch %d", config->squelch );

  } else if( strcmp( tok1, ( const char * ) "pocsag_debug" ) == 0 ) {
    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 ); //more detailed logging
      pocsag_debug_on = val;
    }
    printf( "\r\npocsag_debug %d", pocsag_debug_on );
  }
  ///////////////////////////////////////////////////////////////
  // POCSAG
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "pagers" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 ); //more detailed logging
      config->do_pagers = val;
    }
    printf( "\r\npagers %d", config->do_pagers );

    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  // ACARS
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "acars" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    if( toks == 2 ) {
      int val = ( int ) atoi( tok2 ); //more detailed logging
      config->do_acars = val;
    }
    printf( "\r\nacars %d", config->do_acars );

    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  // AGC TARGET
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "agc_target" ) == 0 ) {

    if( toks == 2 ) {
      config->agc_target = ( int ) atoi( tok2 ); //more detailed logging
      set_atten( 0 );
      set_bbatten( 0 );
    }
    printf( "\r\nagc_target %d", config->agc_target );

  }
  ///////////////////////////////////////////////////////////////
  // AGC ATTACK
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "agc_attack" ) == 0 ) {

    if( toks == 2 ) {
      config->agc_attack = ( int ) atoi( tok2 ); //more detailed logging
      set_atten( 0 );
      set_bbatten( 0 );
    }
    printf( "\r\nagc_attack %d", config->agc_attack );
  }
  ///////////////////////////////////////////////////////////////
  // AGC ATTACK
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "agc_decay" ) == 0 ) {

    if( toks == 2 ) {
      config->agc_decay = ( int ) atoi( tok2 ); //more detailed logging
      set_atten( 0 );
      set_bbatten( 0 );
    }
    printf( "\r\nagc_decay %d", config->agc_decay );
  }
  ///////////////////////////////////////////////////////////////
  // AGC AUTO
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "agc_auto" ) == 0 ) {

    if( toks == 2 ) {
      config->agc_auto = ( int ) atoi( tok2 ); //more detailed logging
      set_atten( 0 );
      set_bbatten( 0 );
    }
    printf( "\r\nagc_auto %d", config->agc_auto );


  }
  ///////////////////////////////////////////////////////////////
  // AGC MODE
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "agc" ) == 0 ) {

    if( toks == 2 ) {
      config->agc_mode = ( int ) atoi( tok2 ); //more detailed logging
      set_atten( 0 );
      set_bbatten( 0 );
    }
    printf( "\r\nagc_mode %d", config->agc_mode );


  }
  ///////////////////////////////////////////////////////////////
  // SA MODE  (obsolete)
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "sa" ) == 0 ) {
    printf("\r\nsa command obsolete.  use 'mode' instead.");
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "udp" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    if( toks >= 2 ) {

      if( strcmp(tok2, "off") == 0) {
        config->udp_mode = UDP_STREAM_OFF;
      }
      else if( strcmp(tok2, "adc_raw") == 0) {
        config->udp_mode = UDP_STREAM_ADC_RAW;
      }
      else if( strcmp(tok2, "iq16_decflt") == 0) {
        config->udp_mode = UDP_STREAM_IQ16_DECFLT;
      }
      else if( strcmp(tok2, "demod") == 0) {
        config->udp_mode = UDP_STREAM_DEMOD;
      }
      else if( strcmp(tok2, "4fsk_sym_s16") == 0) {
        config->udp_mode = UDP_STREAM_4FSK_SYM_S16;
      }
      else if( strcmp(tok2, "4fsk_dibit") == 0) {
        config->udp_mode = UDP_STREAM_4FSK_DIBIT;
      }
      else if( strcmp(tok2, "audio_48k_s16") == 0) {
        config->udp_mode = UDP_STREAM_AUDIO_48K_S16;
      }
      else if( strcmp(tok2, "audio_48k_s16_cont") == 0) {
        config->udp_mode = UDP_STREAM_AUDIO_48K_S16_CONT;
      }
      else if( strcmp(tok2, "voice8k") == 0) {
        config->udp_mode = UDP_STREAM_VOICE8K;
      }
      else if( strcmp(tok2, "console") == 0) {
        config->udp_mode = UDP_STREAM_CONSOLE;
      }
      else {
        printf("\r\nunknown mode.  usage example: udp audio_48k_s16");
        list_udp_modes();
      }

      udp_out_len = 0;
    }

    printf( "\r\nudp %s", udp_mode_to_str( config->udp_mode ) );

    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  // MODE
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "mode" ) == 0 ) {
    prim = __get_PRIMASK();
    __disable_irq();

    int tmp_mode = config->channel_bw;

    if( toks >= 2 ) {

      config->do_adsb=0;
      config->do_pagers=0;
      config->do_acars=0;
      ampmodem_set_mode( LIQUID_AMPMODEM_DSB, 0);  //no supressed carrier

      //config->sa_mode = ( int ) atoi( tok2 ); //more detailed logging
      if( strcmp(tok2, "fm") == 0) {
        config->sa_mode = MODE_FM;
      }
      else if( strcmp(tok2, "iq") == 0) {
        config->sa_mode = MODE_IQ;
      }
      else if( strcmp(tok2, "am") == 0) {
        config->sa_mode = MODE_AM;
      }
      else if( strcmp(tok2, "p25") == 0) {
        config->sa_mode = MODE_P25;
        config->channel_bw=0;
        config->p25_audio_gain=1.0f;
      }
      else if( strcmp(tok2, "dmr") == 0) {
        config->sa_mode = MODE_DMR;
        config->channel_bw=0;
      }
      else if( strcmp(tok2, "adsb") == 0) {
        config->sa_mode = MODE_ADSB;
        config->do_adsb=1;
        config->i_off=128;
        config->q_off=128;
        config->channel_bw=1;
      }
      else if( strcmp(tok2, "acars") == 0) {
        config->sa_mode = MODE_AM;
        config->do_acars=1;
        config->channel_bw=0;
        config->udp_mode = UDP_STREAM_DEMOD;
        ampmodem_set_mode( LIQUID_AMPMODEM_DSB, 0);  //double-side band, no supressed carrier
      }
      else if( strcmp(tok2, "pagers") == 0) {
        config->sa_mode = MODE_FM;
        config->do_pagers=1;
        config->channel_bw=0;
        config->udp_mode = UDP_STREAM_DEMOD;
      }
      else if( strcmp(tok2, "syncmon") == 0) {
        config->sa_mode = MODE_SYNCMON;
        config->channel_bw=0;
      }
      else {
        printf("\r\nunknown mode.  usage example: mode fm narrow");
        //config->sa_mode = MODE_FM;
        list_decoders();
      }

      if(toks==3) {
        if( strcmp(tok3, "wide")==0 ) {
          config->channel_bw = 1;
          config->do_low_if = 0;
        }
        if( strcmp(tok3, "med")==0 ) {
          config->channel_bw = 2;
          config->do_low_if = 0;
        }
        if( strcmp(tok3, "narrow")==0 ) {
          config->channel_bw = 0;
        }

//enum {
 //   LIQUID_AMPMODEM_DSB=0,  // double side-band
  //  LIQUID_AMPMODEM_USB,    // single side-band (upper)
   // LIQUID_AMPMODEM_LSB     // single side-band (lower)
//};
        if( strcmp(tok3, "dsb")==0 ) {
           ampmodem_set_mode( LIQUID_AMPMODEM_DSB, 0);  //double-side band, no supressed carrier
           printf("  dsb");
        }
        if( strcmp(tok3, "lsb")==0 ) {
           ampmodem_set_mode( LIQUID_AMPMODEM_LSB, 1); //supressed carrier, lower side band
           printf("  lsb");
        }
        if( strcmp(tok3, "usb")==0 ) {
           ampmodem_set_mode( LIQUID_AMPMODEM_USB, 1); //supressed carrier, upper side band
           printf("  usb");
        }
      }
    }

    if(tmp_mode!=config->channel_bw) reconfig_adc(config->channel_bw);

    //handle special modes
    if(config->do_pagers && config->sa_mode==MODE_FM ) {
      printf("\r\nmode pagers"); 
    }
    else if(config->do_acars && config->sa_mode==MODE_AM ) {
      printf("\r\nmode acars"); 
    }
    else {
      printf( "\r\nmode %s", sa_mode_to_str( config->sa_mode ) );
    }

    udp_out_len = 0;

    switch( config->sa_mode ) {
    case  MODE_6 :
      config->channel_bw = 1;
      reconfig_adc( config->channel_bw );
      set_bw( 1 );
      break;

    case  MODE_DMR :
      dmr_s->mode = DMR_MODE_VOICE_PLAYBACK;
    break;

    default :
      break;
    }
    if( !prim ) {
      __enable_irq();
    }

  }
  ///////////////////////////////////////////////////////////////
  // P25_LOGGING
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "logging" ) == 0 ) {

    val1 = command_is_one( tok2 ); //on/off 0/1
    if( val1 >= 0 && toks == 2 ) {
      config->logging = val1;
    } else if( toks == 2 ) {
      config->logging = ( int ) atoi( tok2 ); //more detailed logging
    }
    printf( "\r\nlogging %d", config->logging );

  }
  ///////////////////////////////////////////////////////////////
  // WRITE_CONFIG
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "write_config" ) == 0 ) {
    __preset = 0;
    do_write_config = 1;
  }
  ///////////////////////////////////////////////////////////////
  // READ_CONFIG
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "read_config" ) == 0 ) {
    __preset = 0;
    do_read_config = 1;
  }
  ///////////////////////////////////////////////////////////////
  // f - toggles between follow / unfollow current talkgroup
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "f" ) == 0 ) {
    if( config->p25_follow != 0) {
      printf("\r\nunfollowing talkgroup %d", config->p25_follow);
      config->p25_follow=0;
    }
    else {
      config->p25_follow = current_talkgroup;
      printf("\r\nfollowing current talkgroup %d", config->p25_follow);
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "n" ) == 0 ) {
    if( __preset < 255 ) __preset++;
    do_read_config = 1;
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "p" ) == 0 ) {
    if( __preset > 0 ) __preset--;
    do_read_config = 1;
  }
  ///////////////////////////////////////////////////////////////
  // DOI
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "doi" ) == 0 ) {
    int val = mixer_read_reg( 0x0e );
    if( toks >= 2 ) {
      val = atoi( tok2 );
      config->mixer_doi=val;
      mixer_write_reg( 0x0e, val ); //I dc offset
    }
    printf( "\r\ndoi = %d", val );
  }
  ///////////////////////////////////////////////////////////////
  // DOQ
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "doq" ) == 0 ) {
    int val = mixer_read_reg( 0x0f );
    if( toks >= 2 ) {
      val = atoi( tok2 );
      config->mixer_doq=val;
      mixer_write_reg( 0x0f, val ); //Q dc offset
    }
    printf( "\r\ndoq = %d", val );
  }
  ///////////////////////////////////////////////////////////////
  //  BBATTEN
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "bbatten" ) == 0 ) {
    int val = mixer_read_reg( 0x10 );
    val >>= 3;

    if( toks >= 2 ) {
      val = atoi( tok2 );
      set_bbatten( val );
    }
    printf( "\r\nbbatten (0-31) = %d", val );
  }
  ///////////////////////////////////////////////////////////////
  //  BBAMPG
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "bbampg" ) == 0 ) {
    int val = mixer_read_reg( 0x15 );
    val >>= 4;

    if( toks >= 2 ) {
      val = atoi( tok2 );
      set_bbampg( val );
    }
    printf( "\r\nbbampg (0-7) = %d", val );
  }
  ///////////////////////////////////////////////////////////////
  //  BBGERR
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "bbgerr" ) == 0 ) {
    int val = mixer_read_reg( 0x11 );
    if( toks >= 2 ) {
      val = atoi( tok2 );
      mixer_write_reg( 0x11, val );
    }
    printf( "\r\nbbgerr = %d", val );
  }
  ///////////////////////////////////////////////////////////////
  // RESET
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "system_reset" ) == 0 ) {
    do_system_reset = 1;
    reset_timer = HAL_GetTick();
    printf( "\r\ninitiating system reset" );
  }
  ///////////////////////////////////////////////////////////////
  // HELP
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "help" ) == 0 ) {
    show_help();
  }
  ///////////////////////////////////////////////////////////////
  // SHOW_CONFIG
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "show_config" ) == 0 ) {
    show_config();
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "ls" ) == 0 ) {

    if(toks>=2) {
      if( strcmp( tok2, ( const char * ) "regs" ) == 0 ) {
        do_ls_regs = 1;
      }
      else if( strcmp( tok2, ( const char * ) "presets" ) == 0 ) {
        do_ls_presets = 1;
      }
      else if( strcmp( tok2, ( const char * ) "ac" ) == 0 ) {
        show_aircraft(0);
      }
      else if( strcmp( tok2, ( const char * ) "aca" ) == 0 ) {
        show_aircraft(1);
      }
      else if( strcmp( tok2, ( const char * ) "decoders" ) == 0 ) {
        list_decoders();
      }
      else if( strcmp( tok2, ( const char * ) "tg" ) == 0 ) {
        do_ls_talkgroups=1;
        ls_talkgroups_parm = 0;
        if(toks==3) ls_talkgroups_parm = atoi(tok3); 
      }
      else if( strcmp( tok2, ( const char * ) "base_freqs" ) == 0 ) {
        p25_list_base_freqs();
      }
    }
    else {
      do_ls_presets = 1;
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "preset" ) == 0 ) {

    if( toks == 1 ) {
      printf( "\r\ncurrent preset: %d, %s", __preset, config->preset_desc_str );
      print_prompt();
      return;
    }

    if( toks >= 3 ) {
      if( strcmp( tok2, ( const char * ) "w" ) == 0 ) {

        if(toks==4) strncpy( config->preset_desc_str, tok4, sizeof(config->preset_desc_str) );

        do_write_config = 1;
        __preset = atoi( tok3 );

      } else if( strcmp( tok2, ( const char * ) "r" ) == 0 ) {
        do_read_config = 1;
        __preset = atoi( tok3 );
      } else if( strcmp( tok2, ( const char * ) "d" ) == 0 ) {
        do_delete_config = 1;
        __preset = atoi( tok3 );
      } else if( strcmp( tok2, ( const char * ) "s" ) == 0 ) {
        config->preset_desc_str[64] = 0;
        strncpy( config->preset_desc_str, tok3, 64 );
        printf( "\r\npreset desc: %s", config->preset_desc_str );
      }
    } else {
      printf( "\r\nusage: preset [r][w] index [preset_name]" );
    }
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "pll" ) == 0 ) {
    wait_for_lock();
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "pll2" ) == 0 ) {
    wait_for_lock2();
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else if( strcmp( tok1, ( const char * ) "dfu_boot" ) == 0 ) {
    //user manual shows 2 different addresses.  which one? neither seem to work
    HAL_SYSCFG_CM7BootAddConfig( SYSCFG_BOOT_ADDR0, 0x1FF00000 );
    //HAL_SYSCFG_CM7BootAddConfig(SYSCFG_BOOT_ADDR0, 0x08000000);
    //HAL_SYSCFG_CM7BootAddConfig(SYSCFG_BOOT_ADDR0, 0x1FFF0000);
    //NVIC_SystemReset();
  }
  ///////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////
  else {
    if( toks > 0 ) printf( "\r\nunknown command. try 'help'" );
  }

  if( do_prompt ) print_prompt();
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
int command_is_one( char *str )
{
  if( strcmp( str, "1" ) == 0 ) return 1;
  if( strcmp( str, "on" ) == 0 ) return 1;
  if( strcmp( str, "0" ) == 0 ) return 0;
  if( strcmp( str, "off" ) == 0 ) return 0;
  return -1;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
void read_config( void )
{


  uint32_t prim = __get_PRIMASK();
  __disable_irq();

  HAL_I2S_DMAStop( &hi2s2 );
  delay_ms_ni( 1 );

  printf( "\r\nreading preset %d from flash.", __preset );

  flash_read_preset( __preset );  //this will call reconfig_adc and re-init the ADC

  printf( "  freq: %f", config->frequency );
  printf( ", freq_off: %f", config->freq_offset_mhz );
  printf( "  , desc: %s", config->preset_desc_str );
  print_prompt();

  HAL_I2S_Transmit_DMA( &hi2s2, audio_tx, 256 );
  delay_ms_ni( 1 );

  if( !prim ) {
    __enable_irq();
  }

}
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
void command_tick( void )
{

  if( do_write_config ) {
    prim = __get_PRIMASK();
    __disable_irq();

    HAL_I2S_DMAStop( &hi2s2 );

    do_write_config = 0;
    printf( "\r\nwriting preset %d to flash.", __preset );
    write_configuration_to_flash();
    print_prompt();

    HAL_I2S_Transmit_DMA( &hi2s2, audio_tx, 256 );

    if( !prim ) {
      __enable_irq();
    }
  }

  if( do_read_config ) {
    do_read_config = 0;
    read_config();
  }

  if( do_delete_config ) {
    prim = __get_PRIMASK();
    __disable_irq();

    HAL_I2S_DMAStop( &hi2s2 );

    do_delete_config = 0;
    printf( "\r\ndeleting preset %d from flash.", __preset );
    delete_configuration_from_flash();
    print_prompt();

    HAL_I2S_Transmit_DMA( &hi2s2, audio_tx, 256 );

    if( !prim ) {
      __enable_irq();
    }
  }

  if( do_system_reset ) {
    if( HAL_GetTick() - reset_timer > 500 ) {
      close_telnet();
    }
    if( HAL_GetTick() - reset_timer > 1500 ) {
      NVIC_SystemReset();
    }
  }

  if( do_ls_regs ) {
    do_ls_regs = 0;
    dump_regs();
    print_prompt();
  }

  if( do_ls_presets ) {
    do_ls_presets = 0;
    list_presets();
    print_prompt();
  }

  if(do_ls_talkgroups) {
    do_ls_talkgroups=0;
    list_talkgroups(ls_talkgroups_parm);
    print_prompt();
  }

}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
void show_help( void )
{
  //TODO: fill this in
  list_decoders();
}
//////////////////////////////////////////////////////////////////////////////////////////////////
void show_config( void )
{
  //TODO: fill this in
  printf( "\r\naudio %d, audio vol %3.2f", config->audio_on, config->audio_volume_f );
  printf( "\r\nlogging %d", config->logging );
  printf( "\r\n " );
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
void list_decoders(void) {
  printf("\r\n\r\nAvailable SuperH+ built-in demod/decoders");
  printf("\r\n------------------");
  printf("\r\nFM Analog narrow/med/wide (mode fm)");
  printf("\r\nAM Analog narrow/med/wide (mode am)");
  printf("\r\nIQ 16-bit over UDP (mode iq) + GNURadio Driver");
  printf("\r\nP25P1 Voice + Trunking Control Channel (mode p25)");
  printf("\r\nDMR Voice + ConnectPlus Trunking Control Channel (mode dmr)");
  printf("\r\nADSB Mode-S - Console output (mode adsb)");
  printf("\r\nACARS - Console output (mode acars)");
  printf("\r\nFLEX-4FSK-1600 - Console output (mode pagers, fm)");
  printf("\r\nPOCSAG 1200 - Console output (mode pagers, fm)");
  print_prompt();
}
///////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////
void list_udp_modes(void) {
  printf("\r\n\r\nAvailable udp modes");
  printf("\r\n------------------");
  printf("\r\noff");
  printf("\r\nadc_raw (not implemented)");
  printf("\r\niq16_decflt");
  printf("\r\ndemod");
  printf("\r\n4fsk_sym_s16 (not implemented)");
  printf("\r\n4fsk_dibit (not implemented)");
  printf("\r\naudio_48k_s16");
  printf("\r\naudio_48k_s16_cont");
  printf("\r\nvoice8k");
  printf("\r\nconsole (not implemented)");
  print_prompt();
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
const char * sa_mode_to_str(int val) {
  return (char *) &sa_modes[val];
}
const char * udp_mode_to_str(int val) {
  return (char *) &udp_modes[val];
}






void Get_SerialNum(void)
{

  uint32_t *ptr = 0x1ff1e800; //device unique id location

  uint32_t val3 = *ptr++;
  uint32_t val2 = *ptr++;
  uint32_t val1 = *ptr++;

  IntToUnicode(val1, &USBD_StringSerial[0], 8);
  IntToUnicode(val2, &USBD_StringSerial[8], 8);
  IntToUnicode(val3, &USBD_StringSerial[16], 8);


}

/**
  * @brief  Convert Hex 32Bits value into char 
  * @param  value: value to convert
  * @param  pbuf: pointer to the buffer 
  * @param  len: buffer length
  * @retval None
  */
void IntToUnicode(uint32_t value, uint8_t * pbuf, int len)
{
  int idx = 0;
  uint32_t v = value;

  for (idx = 0; idx < len; idx++)
  {
    if (((v >> 28)) < 0xA)
    {
      *pbuf++ = (v >> 28) + '0';
    }
    else
    {
      *pbuf++ = (v >> 28) + 'A' - 10;
    }

    v = v << 4;

  }

  *pbuf = 0x00;
}
