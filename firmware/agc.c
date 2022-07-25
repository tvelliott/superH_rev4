

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

#include "globals.h"
#include "command.h"
#include "telnet.h"
#include "std_io.h"
#include "telnet.h"
#include "main.h"
#include "rfdemod.h"
#include "p25_decode.h"
#include "p25_stats.h"
#include "agc.h"

volatile float dBFS;
volatile int current_rssi = -127;
static volatile int agc_mod;


#define AUD_AGC_LEN 128 //must be power of two
#define RF_AGC_LEN 32  //must be power of two

//audio agc related
static volatile float audio_max[AUD_AGC_LEN];
static volatile int audio_max_idx;
static volatile float aout_gain=1.0f;
static volatile float aout_abs;
static volatile float aud_agc_max;
static volatile float gainfactor;
static volatile float gaindelta;
static volatile float maxbuf;

static volatile float rf_audio_max[RF_AGC_LEN];
static volatile int rf_audio_max_idx;
static volatile float rf_aout_gain=1.0f;
static volatile float rf_aout_abs;
static volatile float rf_aud_agc_max;
static volatile float rf_gainfactor;
static volatile float rf_gaindelta;
static volatile float rf_maxbuf;

//needed for the int16_t version of audio agc
static volatile float tmp_buffer_f[1024];

#define ATTEN_MIN_VAL 0

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  gain controls:
//
//  //ADC 1 and ADC 2  in I/Q interleaved mode, differential
//  RF_CONNECTOR ->  SKY12347 -> LTC5586_ATTEN -> LTC5586_IF_AMP -> LTC6910_AMP -> LTC1564_AMP_FILTER ->ADC_DIFF_ENDED
//
//  //ADC 3   in I/Q regular conversion mode, single ended.
//  Note that the only baseband for ADC3 filtering is the interstage filter on the 2nd mixer.
//  RF_CONNECTOR ->  SKY12347 -> LTC5586_ATTEN -> LTC5586_IF_AMP -> LTC6910_AMP -> ADC_SINGLE_ENDED
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//   Attenuator Front-End :  PE43712   0-32 dB
//   0 = insertion loss  (0.5dB)  (all 0 bits on hardware)  (default)
//   15 = 8    dB
//   31 = 16   dB
//   63 = 31.5 dB   (all one bits on hardware)
//
//
//   LTC5586  Attenuator, Back-End,  5-bit , 0 to 32 dB attenuation  (bbatten 0-31)
//   0  = 0 dB  (default)
//   31 = 31 dB
//
//   LTC5586  IF AMP Gain,  8 Steps, 10dB  (default 6)   (bbampg 0-7)
//
//
//  LTC6910 gain settings  (gain1  1-5),
//    0 = -120dB  (off)
//    1 = 0  dB
//    2 = 6  dB    (default)
//    3 = 14  dB
//    4 = 20  dB
//    5 = 26  dB
//
//    6 = 34  dB   (not used)
//    7 = 40  dB   (not used)
//
//////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////
////////////////////////////////////////////////
static void agc_up( void )
{

  int delta = abs( ( int ) dBFS - config->agc_target );

  if( delta > 3 ) {   //3dB

    switch( config->agc_mode ) {

    case  1 :
      if( delta <= 3 ) delta = 1;
      if( delta > 6 ) delta = 3;
      if( delta > 12 ) delta = 6;

      if( config->bb_atten > 0 ) {
        set_bbatten( config->bb_atten - delta );
        return;
      }

      if( config->front_end_atten > ATTEN_MIN_VAL ) {
        set_atten( config->front_end_atten - delta );
        return;
      }
    case  2 :

      if( delta <= 3 ) delta = 1;
      if( delta > 6 ) delta = 3;
      if( delta > 12 ) delta = 6;


      if( config->front_end_atten > ATTEN_MIN_VAL ) {
        set_atten( config->front_end_atten - delta );
        return;
      }

      if( config->bb_atten > 0 ) {
        set_bbatten( config->bb_atten - delta );
        return;
      }
      break;

    case  5 :
      if( config->bb_atten > 0 ) {
        set_bbatten( config->bb_atten - 1 );
        return;
      } else if( config->front_end_atten > ATTEN_MIN_VAL ) {
        if( config->front_end_atten > 15 ) set_atten( config->front_end_atten - 1 );
        return;
      }
      break;

    case  6 :
      if( config->front_end_atten > ATTEN_MIN_VAL ) {

        //TODO: FIX THIS
#if 0
        //due to bad wiring any value between 1 and 15 is 15
        set_atten( config->front_end_atten - 15 );
#else
        set_atten( config->front_end_atten - 1 );
#endif
        return;
      }
      break;

    }
  }

}
////////////////////////////////////////////////
////////////////////////////////////////////////
static void agc_down( void )
{


  int delta = abs( ( int ) dBFS - config->agc_target );

  if( delta > 3 ) {

    switch( config->agc_mode ) {
    case  1 :
    case  3 :   //mode 3, we just increase the front-end atten first and leave it there until freq change

      if( delta <= 3 ) delta = 1;
      if( delta > 6 ) delta = 3;
      if( delta > 12 ) delta = 6;

      if( config->front_end_atten < 63 ) {
        set_atten( config->front_end_atten + delta );
        return;
      }

      if( config->bb_atten < 31 ) {
        set_bbatten( config->bb_atten + delta );
        return;
      }
      break;

    case  2 :
    case  4 :

      if( delta <= 3 ) delta = 1;
      if( delta > 6 ) delta = 3;
      if( delta > 12 ) delta = 6;


      if( config->bb_atten < 31 ) {
        set_bbatten( config->bb_atten + delta );
        return;
      }

      if( config->front_end_atten < 63 ) {
        set_atten( config->front_end_atten + delta );
        return;
      }


      break;

    case  5 :

      if( config->bb_atten < 31 ) {
        set_bbatten( config->bb_atten + 1 );
        return;
      } else if( config->front_end_atten < 63 ) {
        set_atten( config->front_end_atten + 1 );
        return;
      }
      break;

    case  6 :
      if( config->front_end_atten < 63 ) {
        set_atten( config->front_end_atten + 1 );
        return;
      }
      break;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
int get_rssi_i(void) {
  return current_rssi;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// called from interrupt handler
//////////////////////////////////////////////////////////////////////////////////////////////////////
void update_agc()
{
  int i;
  current_rssi = ( int ) get_rssi(); //updates dBFS

  if( config->agc_mode == 0 ) return;



#if 1
  if( config->channel_bw == 1 ) { //200khz channel updates faster
    config->agc_attack = 32; //always fast attack
    config->agc_decay = 64; //always fast decay
  }
  if( config->channel_bw == 0 ) { //14.x channel, less delay for slower sample rates
    config->agc_attack = 3; //always fast attack
    config->agc_decay = 6; //always fast decay
  }
  if( config->channel_bw == 2 ) { //14.x channel, less delay for slower sample rates
    config->agc_attack = 6; //always fast attack
    config->agc_decay = 12; //always fast decay
  }

  config->agc_auto = 0;
  //config->gain1 = 5;
#endif

  agc_mod++;
  if( ( int ) dBFS < config->agc_target && ( agc_mod % config->agc_decay ) == 0 ) {
    agc_up();
    return;
  }

  if( ( int ) dBFS > config->agc_target && ( agc_mod % config->agc_attack ) == 0 ) {
    agc_down();
    return;
  }

  if( config->agc_mode == 5 && config->front_end_atten < 15 ) {
    //set_atten( 15 );
    set_atten( config->front_end_atten + 1 );
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void set_bw( int bw )
{

  config->adc_flt_bw = bw;
  HAL_GPIO_WritePin( FLT_CS_GPIO_Port, FLT_CS_Pin, GPIO_PIN_SET );

  switch( config->adc_flt_bw ) {

  case  0 : //mute,  0 gain
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_RESET );
    break;

  case  1 :   //20khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_RESET );
    break;

  case  2 :   //40khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_RESET );
    break;

  case  3 :   //60khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_RESET );
    break;

  case  4 :   //80khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_RESET );
    break;

  case  5 :   //100khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_RESET );
    break;

  case  6 :   //120khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_RESET );
    break;

  case  7 :   //140khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_RESET );
    break;

  case  8 :   //160khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_SET );
    break;

  case  9 :   //180khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_SET );
    break;

  case  10 :   //200khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_SET );
    break;

  case  11 :   //220khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_SET );
    break;

  case  12 :   //240khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_SET );
    break;

  case  13 :   //260khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_SET );
    break;

  case  14 :   //280khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_SET );
    break;

  case  15 :   //300khz bw
    HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_SET );
    break;
  }

  HAL_GPIO_WritePin( FLT_CS_GPIO_Port, FLT_CS_Pin, GPIO_PIN_RESET );
  delay_us( 1 );
  HAL_GPIO_WritePin( FLT_CS_GPIO_Port, FLT_CS_Pin, GPIO_PIN_SET );
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//  LTC6910 gain settings  (gain1  1-5),
//    0 = -120dB  (off)
//    1 = 0  dB
//    2 = 6  dB    (default)
//    3 = 14  dB
//    4 = 20  dB
//    5 = 26  dB
//
//    6 = 34  dB   (not used)
//    7 = 40  dB   (not used)
//////////////////////////////////////////////////////////////////////////////////////////////////////
void set_gain1( int gain )
{


#if 1
  gain = bb_gain; //over-ride with preset 0 setting

  if( config->agc_mode != 0 ) {
    if( config->channel_bw == 0 ) gain = 4;
    if( config->channel_bw == 1 ) gain = 4;
  }

  if( config->sa_mode==MODE_ADSB && config->channel_bw==1) gain=5;
#endif


  config->gain1 = gain;

  switch( config->gain1 ) {
  case  0 :
    HAL_GPIO_WritePin( postmix_gain0_GPIO_Port, postmix_gain0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( postmix_gain1_GPIO_Port, postmix_gain1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( postmix_gain2_GPIO_Port, postmix_gain2_Pin, GPIO_PIN_RESET );
    break;
  case  1 :
    HAL_GPIO_WritePin( postmix_gain0_GPIO_Port, postmix_gain0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( postmix_gain1_GPIO_Port, postmix_gain1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( postmix_gain2_GPIO_Port, postmix_gain2_Pin, GPIO_PIN_RESET );
    break;
  case  2 :
    HAL_GPIO_WritePin( postmix_gain0_GPIO_Port, postmix_gain0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( postmix_gain1_GPIO_Port, postmix_gain1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( postmix_gain2_GPIO_Port, postmix_gain2_Pin, GPIO_PIN_RESET );
    break;
  case  3 :
    HAL_GPIO_WritePin( postmix_gain0_GPIO_Port, postmix_gain0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( postmix_gain1_GPIO_Port, postmix_gain1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( postmix_gain2_GPIO_Port, postmix_gain2_Pin, GPIO_PIN_RESET );
    break;
  case  4 :
    HAL_GPIO_WritePin( postmix_gain0_GPIO_Port, postmix_gain0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( postmix_gain1_GPIO_Port, postmix_gain1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( postmix_gain2_GPIO_Port, postmix_gain2_Pin, GPIO_PIN_SET );
    break;
  case  5 :
    HAL_GPIO_WritePin( postmix_gain0_GPIO_Port, postmix_gain0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( postmix_gain1_GPIO_Port, postmix_gain1_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( postmix_gain2_GPIO_Port, postmix_gain2_Pin, GPIO_PIN_SET );
    break;
  case  6 :
    HAL_GPIO_WritePin( postmix_gain0_GPIO_Port, postmix_gain0_Pin, GPIO_PIN_RESET );
    HAL_GPIO_WritePin( postmix_gain1_GPIO_Port, postmix_gain1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( postmix_gain2_GPIO_Port, postmix_gain2_Pin, GPIO_PIN_SET );
    break;
  case  7 :
    HAL_GPIO_WritePin( postmix_gain0_GPIO_Port, postmix_gain0_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( postmix_gain1_GPIO_Port, postmix_gain1_Pin, GPIO_PIN_SET );
    HAL_GPIO_WritePin( postmix_gain2_GPIO_Port, postmix_gain2_Pin, GPIO_PIN_SET );
    break;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
// val = 0-63  (0-31 dB)
//////////////////////////////////////////////////////////////////////////////////////////////////////
void set_atten( int val )
{

#if 0
  //no attenuator installed
  config->front_end_atten = 0;
  return;
#endif

  if( val < ATTEN_MIN_VAL ) val = ATTEN_MIN_VAL; //always at least 0.5dB of attenuation
  if( val > 63 ) val = 63;

  val &= 0x3f;

  //zero attenuation, all-ones
  HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin( data_4_GPIO_Port, data_4_Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin( data_5_GPIO_Port, data_5_Pin, GPIO_PIN_RESET );

  if( val & 0x01 ) HAL_GPIO_WritePin( data_0_GPIO_Port, data_0_Pin, GPIO_PIN_SET );
  if( val & 0x02 ) HAL_GPIO_WritePin( data_1_GPIO_Port, data_1_Pin, GPIO_PIN_SET );
  if( val & 0x04 ) HAL_GPIO_WritePin( data_2_GPIO_Port, data_2_Pin, GPIO_PIN_SET );
  if( val & 0x08 ) HAL_GPIO_WritePin( data_3_GPIO_Port, data_3_Pin, GPIO_PIN_SET );
  if( val & 0x10 ) HAL_GPIO_WritePin( data_4_GPIO_Port, data_4_Pin, GPIO_PIN_SET );
  if( val & 0x20 ) HAL_GPIO_WritePin( data_5_GPIO_Port, data_5_Pin, GPIO_PIN_SET );

  //atten latch enable
  HAL_GPIO_WritePin( atten_le_GPIO_Port, atten_le_Pin, GPIO_PIN_SET );
  delay_us( 1 );
  HAL_GPIO_WritePin( atten_le_GPIO_Port, atten_le_Pin, GPIO_PIN_RESET );

  config->front_end_atten = val;
}

///////////////////////////////////////////////////////////////
//  atten = 0-31  (0-31 dB)
///////////////////////////////////////////////////////////////
void set_bbatten( int atten )
{

  if( atten < 0 ) atten = 0;
  if( atten > 31 ) atten = 31;

  uint8_t val = atten << 3;
  val |= 0x04; //IP3IC = 4
  mixer_write_reg( 0x10, val );

  config->bb_atten = atten;
}
///////////////////////////////////////////////////////////////
//  gain = 0-7   (0-7 dB)
///////////////////////////////////////////////////////////////
void set_bbampg( int gain )
{
  uint8_t val = gain << 4;
  val |= 0x0a; //defaults in register
  mixer_write_reg( 0x15, val );
  config->bb_ampg = gain;
}
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
float get_rssi( void )
{

#if 0
  config->gain1 = 5;
#endif

  float ref;
  switch( config->gain1 ) {
  case  7 :
    ref = -72.0;
    break;
  case  6 :
    ref = -66.0;
    break;
  case  5 :
    ref = -58.0;
    break;
  case  4 :
    ref = -52.0;
    break;
  case  3 :
    ref = -46.0;
    break;
  case  2 :
    ref = -38.0;
    break;
  case  1 :
    ref = -32.0;
    break;
  }

  /*

  //based on measurements
  //accounting for FIR filters, etc
  switch( config->decimate ) {
  case  1 :
    switch( config->channel_bw ) {
    case  0 :
      ref += 10.0;
      break;

    case  1 :
      ref += 4.0;
      break;
    }
    break;

  case  2 :
    switch( config->channel_bw ) {
    case  0 :
      ref += -4.0;
      break;

    case  1 :
      ref += -11.0;
      break;
    }
    break;

  case  4 :
    switch( config->channel_bw ) {
    case  0 :
      ref += -4.0;
      break;

    case  1 :
      ref += -11.0;
      break;
    }
    break;

  case  8 :
    switch( config->channel_bw ) {
    case  0 :
      ref += 3.0;
      break;

    case  1 :
      ref += -5.0;
      break;
    }
    break;

  case  16 :
    switch( config->channel_bw ) {
    case  0 :
      ref += 2.0;
      break;

    case  1 :
      ref += -11.0;
      break;
    }
    break;
  }
  */

  ref += -4;  //dec=4, ch_bw=0 from above commented

  #if 1
    //octave:7> actual=[-8,-2,-0,2];
    //octave:8> measured=[10.5,500.5,900.5,1100.5];
    //octave:9> fit=polyfit(measured,actual,1);
      
    double freq_correction = config->frequency;
    //freq_correction *= 0.0088336; 
    //freq_correction += -7.5474877; 
    freq_correction *= 0.0088336; 
    freq_correction += -7.5474877; 
  #else
    double freq_correction = 0.0f; 
  #endif

  ref += freq_correction;  //linear offset, adjusted for cable loss
  ref += ( 7 - config->bb_ampg );
  ref += config->bb_atten;
  ref += rssi_offset; //rssi_offset is read from config->rssi_offset in preset 0

  //TODO: FIX THIS
#if 0
  //due to bad wiring any value between 1 and 15 is 15
  if( ( config->front_end_atten & 0x0f ) > 0 ) {
    config->front_end_atten |= 0x0f;
  }
  ref += config->front_end_atten / 2.0; //only for rev2 proto
#else
  ref += config->front_end_atten / 2.0;
#endif


  //calculate dBFS
  p25_stats_init( stats_buffer, STATS_SIZE );
  float rms = p25_stats_rms();
  dBFS = 20.0 * log10f( rms / 32768.0 ) + 20.0;

#if 0   //not very stable
  if( p25_stats_max() > 32000 ) {
    channel_settle = 1; //ADC overload
    set_atten( config->front_end_atten + 3 );
  }
#endif

  if( dBFS > 0 ) dBFS = 0.0f;

  if( config->agc_auto && config->frequency > 30.0 && config->sa_mode != MODE_ADSB ) { //don't change gains in ADSB mode 5
    if( ( int )( dBFS + ref ) > -60 ) {
      config->agc_mode = 1;
    } else if( ( int )( dBFS + ref ) > -75 ) {
      config->agc_mode = 5;
    } else {
      config->agc_mode = 2;
    }
    set_gain1( 2 );
  } else if( config->agc_auto && config->frequency <= 30.0 ) {
    //ham radio freqs
    config->agc_mode = 0;
    set_bbatten( 31 );
    //set_gain1(2);
  }

  //return dBm
  return dBFS + ref - 6; //6 is offset correction
}


//////////////////////////////////////////////////////////////////////////////////
// audio agc
//////////////////////////////////////////////////////////////////////////////////
float update_gain_s16(int16_t *audio, int len, float target, float log_mult) {
  int i;

  for(i=0;i<len;i++) {
    tmp_buffer_f[i] = (float) audio[i]; 
  }

  float gain_used = update_gain_f32(tmp_buffer_f,len, target, log_mult);

  for(i=0;i<len;i++) {
    audio[i] = (int16_t) tmp_buffer_f[i];
  }

  return gain_used;
}

//////////////////////////////////////////////////////////////////////////////////
// audio agc
//////////////////////////////////////////////////////////////////////////////////
float update_gain_f32(float *audio, int len, float target, float log_mult) {

  int i, n;

  if(!config->aud_agc_en) return 1.0f;

  audio_max_idx &= (AUD_AGC_LEN-1);

  // detect max level
  aud_agc_max = 0;
  for (n = 0; n < len; n++) {
    aout_abs = fabsf(audio[n]);
    if(aout_abs > aud_agc_max) aud_agc_max = aout_abs;
  }
  audio_max[audio_max_idx++] = aud_agc_max;
  audio_max_idx &= (AUD_AGC_LEN-1);

  // lookup max history
  for (i = 0; i < AUD_AGC_LEN; i++) {
    maxbuf = audio_max[i];
    if(audio_max[i] > aud_agc_max) aud_agc_max = audio_max[i];
  }

  // determine optimal gain level
  if (aud_agc_max > 0.0f) {
    gainfactor = (target / aud_agc_max);
  } else {
    gainfactor = 50.1f;
  }
  if (gainfactor < aout_gain) {
    aout_gain = gainfactor;
    gaindelta = 0.0f;
  } else {
    if (gainfactor > 50.0f) {
        gainfactor = log10f(gainfactor+1.0f)*log_mult;
    }
    gaindelta = gainfactor - aout_gain;
    if (gaindelta > (config->aud_agc * aout_gain)) {
        gaindelta = (config->aud_agc * aout_gain);
    }
  }

  // adjust output gain
  aout_gain += gaindelta;
  for (n = 0; n < len; n++) {
    audio[n] *= aout_gain;
  }

  return aout_gain;
}
//////////////////////////////////////////////////////////////////////////////////
// agc  -returns gain value to use without modification of input data
//////////////////////////////////////////////////////////////////////////////////
float update_gain_cf(float *samples, int len, float target) {

  int i, n;

  if(!config->aud_agc_en) return 1.0f;

  rf_audio_max_idx &= (RF_AGC_LEN-1);

  // detect max level
  rf_aud_agc_max = 0;
  for (n = 0; n < len; n++) {
    rf_aout_abs = fabsf( samples[n]  );
    if (rf_aout_abs > rf_aud_agc_max) rf_aud_agc_max = rf_aout_abs;
  }
  rf_audio_max[rf_audio_max_idx++] = rf_aud_agc_max;
  rf_audio_max_idx &= (RF_AGC_LEN-1);

  // lookup max history
  for (i = 0; i < RF_AGC_LEN; i++) {
    if (rf_audio_max[i] > rf_aud_agc_max) rf_aud_agc_max = rf_audio_max[i];
  }

  // determine optimal gain level
  if (rf_aud_agc_max > 0.0f) {
    rf_gainfactor = (target / rf_aud_agc_max);
  } else {
    rf_gainfactor = 50.0f;
  }
  if (rf_gainfactor < rf_aout_gain) {
    rf_aout_gain = rf_gainfactor;
    rf_gaindelta = 0.0f;
  } else {
    if (rf_gainfactor > 50.0f) {
      rf_gainfactor = 50.0f; 
    }
    rf_gaindelta = rf_gainfactor - rf_aout_gain;
    if (rf_gaindelta > (0.01f * rf_aout_gain)) {
        rf_gaindelta = (0.01f * rf_aout_gain);
    }
  }

  // adjust output gain
  rf_aout_gain += rf_gaindelta;

  return rf_aout_gain;
}
