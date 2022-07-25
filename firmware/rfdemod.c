

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
#include "channel_filter.h"
#include "p25_decode.h"
#include "p25_stats.h"
#include "agc.h"

#include "symsync_config.h"
#include "firdes_arm32.h"
#include "symsync_rf_arm32.h"
#include "freqdem_cfrf_arm32.h"
#include "resamp_cf_arm32.h"
#include "resamp_rf_arm32.h"
#include "window_functions_arm32.h"

#include "math_arm32.h"
#include "msb_index_arm32.h"
#include "firpfb_cf_arm32.h"
#include "window_cf_arm32.h"
#include "dotprod_cf_arm32.h"
#include "resamp_cf_arm32.h"
#include "resamp2_cf_arm32.h"
#include "rkaiser_arm32.h"
#include "window_functions_arm32.h"
#include "firdes_arm32.h"
#include "resamp_config.h"
#include "symsync_config.h"
#include "firfilt_rf_arm32.h"
#include "firfilt_cf_arm32.h"
#include "nco_cf_arm32.h"
#include "freqmod_arm32.h"
#include "ampmodem_arm32.h"
#include "eqlms_rf_arm32.h"
#include "wdelay_rf_arm32.h"
#include "firhilb_arm32.h"

#include "config.h"

// P25 symbol sync config
int          symsync_ftype = LIQUID_FIRFILT_RRC; // filter type

unsigned int symsync_k_4fsk           =   10;     // samples/symbol  48Ksps/4.8k symbols/sec
unsigned int symsync_m_4fsk           =   2;     // filter delay
float        symsync_beta        = 0.365f;    //filter excess bandwidth factor (P25 decode best somewhere between 0.36 to 0.37)
unsigned int symsync_npfb        = SYMSYNC_NFILTERS;  //number of polyphase filters in bank

static volatile float gain_f;

float symsync_buf_in[1];
float symsync_buf_out[1];

static volatile float hf_last_input;

struct symsync_s symsync_4fsk;
struct symsync_s *symsync_q_4fsk;

#define POST_FM_FLT_BW  12500.0f  //narrow-band -  bw after FM demodulator  (audio)

float ch_resamp_r_narrow = ( DECODER_FS / CH_NARROW_RATE_DIV4 ); // resampling rate (output/input)
float ch_resamp_r_med = ( DECODER_FS / CH_MED_RATE_DIV4 ); // resampling rate (output/input)
float ch_resamp_r_wide  = ( DECODER_FS / CH_WIDE_RATE_DIV4 ); // resampling rate (output/input)

unsigned int ch_resamp_m    = RESAMP_CF_M;     // resampling filter semi-length (filter delay)
float ch_resamp_As          = -60.0f;  // resampling filter stop-band attenuation [dB]
float ch_resamp_bw_narrow          = 0.462;  // resampling filter bandwidth
float ch_resamp_bw_wide          = 0.125;  // resampling filter bandwidth
float ch_resamp_bw_med          = 0.462;  // resampling filter bandwidth
unsigned int ch_resamp_npfb = RESAMP_CF_NFILTERS;     // number of filters in bank (timing resolution)
static volatile float complex samples[ADC_SIZE];
static int16_t iq16[ADC_SIZE];

static struct resamp_cf_s ch_rs_narrow;
struct resamp_cf_s *ch_resamp_q_narrow;

static struct resamp_cf_s ch_rs_wide;
struct resamp_cf_s *ch_resamp_q_wide;

static struct resamp_cf_s ch_rs_med;
struct resamp_cf_s *ch_resamp_q_med;

#if 0
unsigned int half_band_m = 7;
float half_band_as = -60.0f;
float complex half_band_x;
float complex half_band_2x[2];

static struct resamp2_s halfband_s1;
struct resamp2_s1 *halfband_q1;
static struct resamp2_s halfband_s2;
struct resamp2_s2 *halfband_q2;
static struct resamp2_s halfband_s3;
struct resamp2_s3 *halfband_q3;

float complex hb_samples1[ADC_SIZE];
float complex hb_samples2[ADC_SIZE];
float complex hb_samples3[ADC_SIZE];
#endif


// channel resamp arrays
//float complex ch_resamp_x[8];
float complex ch_resamp_y[( ADC_SIZE * 2 )]; //48 is enough for 16.28/100.0
unsigned int ch_ny = 0;


//up-sample voice audio from 8 Ksps  to 16.276 Ksps
//float audio_resamp_r           = 2.0345f;   //resampling rate==2.0345 (output/input) to give desired
float audio_resamp_r           = 6.0f;
unsigned int audio_resamp_m    = RESAMP_RF_M;     // resampling filter semi-length (filter delay)
float audio_resamp_As          = -60.0f;  // resampling filter stop-band attenuation [dB]
float audio_resamp_bw          = 0.5f;  // resampling filter bandwidth ,  bw of 0.45 takes edge off
unsigned int audio_resamp_npfb = RESAMP_RF_NFILTERS;     // number of filters in bank (timing resolution)

static struct resamp_rf_s audio_rs;
struct resamp_rf_s *audio_resamp_q;
// audio resamp arrays
static float audio_resamp_x[160];
static float audio_resamp_y[160 * 6];
volatile short upsampled[160 * 6];
volatile int upsample_ready;

volatile char udp_out_buffer[1514];

volatile uint8_t dmr_delay_buffer[40];
volatile int dmr_delay_idx;
volatile int dmr_cc_val;
volatile int16_t dmr_lp[4];
volatile int dmr_lp_idx;

//////////////////////////////////////////////////////////////////
//post FM demod audio filter before being sent to decoder
//////////////////////////////////////////////////////////////////
static struct firfilt_rf_s audio_fir_s;//good for P25 decoding
static struct firfilt_rf_s *audio_fir;
static float audio_fir_bw1 = ( POST_FM_FLT_BW / DECODER_FS ) * 0.5;
static int audio_fir_len1 = 8;

static struct firfilt_rf_s audio_fir_s2;
static struct firfilt_rf_s *audio_fir2;
static float audio_fir_bw2 = ( POST_FM_FLT_BW / DECODER_FS ) * 0.5 * 0.5;
static int audio_fir_len2 = 32;

static struct firfilt_rf_s audio_fir_s3;
static struct firfilt_rf_s *audio_fir3;
static float audio_fir_bw3 = ( POST_FM_FLT_BW / DECODER_FS ) * 0.5 * 0.5 * 0.75;
static int audio_fir_len3 = 64;

static struct firfilt_rf_s audio_fir_s4;
static struct firfilt_rf_s *audio_fir4;
static float audio_fir_bw4 = ( POST_FM_FLT_BW / DECODER_FS ) * 0.5 * 0.5 * 0.75 * 0.85;
static int audio_fir_len4 = 128;

static struct firfilt_rf_s audio_fir_s5;
static struct firfilt_rf_s *audio_fir5;
static float audio_fir_bw5 = 0.25;
static int audio_fir_len5 = 64;

static struct firfilt_rf_s audio_fir_s6;
static struct firfilt_rf_s *audio_fir6;
static float audio_fir_bw6 = ( POST_FM_FLT_BW / DECODER_FS ) * 0.5;
static int audio_fir_len6 = 32;


static float audio_fir_as = -60.0f;
static float fm_filtered;


struct nco_cf_s *nco;
struct freqmod_s *freqmod;

static struct ampmodem_s *am_dem;

struct eqlms_rf_s *eqlms;
// options
unsigned int eqlms_k = 10;         // filter samples/symbol
unsigned int eqlms_m = 3;         // filter semi-length (symbols)
float eqlms_beta = 0.3f;          // filter excess bandwidth factor
float eqlms_mu = 0.200f;          // LMS equalizer learning rate
float eqlms_d[1];
float eqlms_dhat[1];

static float complex fm_in[8];
static float fm_out[( ADC_SIZE * 4 )];
volatile static int avg_i;
volatile static int avg_q;
volatile static int avg_iq_cnt;
int eqlms_trained = 0;
volatile int fm_cnt;
float audio_out_level = 5000.0f;
static volatile short audio_sample;
volatile int udp_out_len;


//remove dc level
volatile double ALPHA_D_i;
volatile double d_alpha_i;
volatile double d_beta_i;
volatile double d_avg_i;

volatile double ALPHA_D_q;
volatile double d_alpha_q;
volatile double d_beta_q;
volatile double d_avg_q;

//remove dc level from audio
volatile double ALPHA_F_i;
volatile double f_alpha_i;
volatile double f_beta_i;
volatile double f_avg_i;

//correct audio resample rate
volatile double ALPHA_A;
volatile double a_alpha;
volatile double a_beta;
volatile double a_avg;
volatile float audio_adj_rate;

volatile float stats_buffer[STATS_SIZE];
volatile float stats_buffer_I[STATS_SIZE];
volatile float stats_buffer_Q[STATS_SIZE];
volatile uint8_t stats_idx;

static struct firhilb_s *firhilb;
static volatile int16_t audio_buffer[1024];

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void init_rfdemod( void )
{

  //demodulate
  freqdem_init( 0.5f );

  //modulate
  //freqmod = freqmod_create( 0.15f );

  am_dem = ampmodem_create( 0.5, 0.0f, LIQUID_AMPMODEM_DSB, 0 );



  ch_resamp_q_narrow = resamp_cf_create( &ch_rs_narrow, ch_resamp_r_narrow, ch_resamp_m, ch_resamp_bw_narrow, ch_resamp_As, ch_resamp_npfb );
  ch_resamp_q_wide = resamp_cf_create( &ch_rs_wide, ch_resamp_r_wide, ch_resamp_m, ch_resamp_bw_wide, ch_resamp_As, ch_resamp_npfb );
  ch_resamp_q_med = resamp_cf_create( &ch_rs_med, ch_resamp_r_med, ch_resamp_m, ch_resamp_bw_med, ch_resamp_As, ch_resamp_npfb );

  symsync_q_4fsk = symsync_create_rnyquist( &symsync_4fsk, symsync_ftype, symsync_k_4fsk, symsync_m_4fsk, symsync_beta, symsync_npfb );

  audio_resamp_q = resamp_rf_create( &audio_rs, audio_resamp_r, audio_resamp_m, audio_resamp_bw, audio_resamp_As, audio_resamp_npfb );

  audio_fir = firfilt_rf_create_kaiser( &audio_fir_s, audio_fir_len1, audio_fir_bw1, audio_fir_as, 0.0f );
  audio_fir2 = firfilt_rf_create_kaiser( &audio_fir_s2, audio_fir_len2, audio_fir_bw2, audio_fir_as, 0.0f );
  audio_fir3 = firfilt_rf_create_kaiser( &audio_fir_s3, audio_fir_len3, audio_fir_bw3, audio_fir_as, 0.0f );
  audio_fir4 = firfilt_rf_create_kaiser( &audio_fir_s4, audio_fir_len4, audio_fir_bw4, audio_fir_as, 0.0f );
  audio_fir5 = firfilt_rf_create_kaiser( &audio_fir_s5, audio_fir_len5, audio_fir_bw5, -120, 0.0f );
  audio_fir6 = firfilt_rf_create_kaiser( &audio_fir_s6, audio_fir_len6, audio_fir_bw6, audio_fir_as, 0.0f );

  //channel_fir = firfilt_cf_create_kaiser( &channel_fir_s, channel_fir_len, channel_fir_bw, channel_fir_as, 0.0f );
  nco = nco_create( LIQUID_NCO );
  nco_set_frequency( nco, config->nco_val );

  firhilb = firhilb_create(5, 60.0f); //semi-len=5, sidelobe supression=60dB

#if 0
  halfband_q1 = resamp2_create( &halfband_s1, half_band_m, 0.0f, half_band_as );
  halfband_q2 = resamp2_create( &halfband_s2, half_band_m, 0.0f, half_band_as );
  halfband_q3 = resamp2_create( &halfband_s3, half_band_m, 0.0f, half_band_as );

  //eqlms = eqlms_rf_create_rnyquist( LIQUID_FIRFILT_RRC, eqlms_k, eqlms_m, eqlms_beta, 0 );
  //eqlms_rf_set_bw( eqlms, eqlms_mu );
#endif
}

//////////////////////////////////////////////////////////////////////////
//  keep in mind,  most of this demod code is handled from an interrupt
//////////////////////////////////////////////////////////////////////////
void downconvert_and_demod( int16_t *data, int len, int decimate )
{
  int16_t *_adc_sptr = data;
  volatile float _I;
  volatile float _Q;
  volatile int _rfi;

  adc_rate += len;
  if( stats_buffer_ready++ < 3 ) return;


  //convert to doubles and apply I/Q corrections
  for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ); _rfi++ ) {

    _I = ( float ) * _adc_sptr++;
    _Q = ( float ) * _adc_sptr++;

    if( config->dc_off == 1 && channel_settle == 0 ) {
        d_avg_i = d_avg_i * d_beta_i + _I * d_alpha_i;
        d_avg_q = d_avg_q * d_beta_q + _Q * d_alpha_q;

        _I -= d_avg_i;
        _Q -= d_avg_q;


    } else if( config->dc_off != 0 ) {
        _I -= d_avg_i;
        _Q -= d_avg_q;
    }

    #if 0
      if(config->channel_bw==0) firhilb_r2c_execute(firhilb, _Q, &samples[_rfi]); //works
        else samples[_rfi] = _Q + _Complex_I * _I;  //not enough cpu to do this with hilbert
    #else
     samples[_rfi] = _Q + _Complex_I * _I;
    #endif
  }


  if( config->do_low_if ) {
    nco_mix_block_down( nco, samples, samples, (ADC_SIZE/2) );
  }



#if 0
  float complex *fptr1 = samples;
  float complex *fptr2 = hb_samples1;

  for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate / 2; _rfi++ ) {
    resamp2_decim_execute( halfband_q1, fptr1, fptr2 );
    fptr1++;
    fptr1++;
    fptr2++;
  }

  fptr1 = hb_samples1;
  fptr2 = samples;

  for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate / 4; _rfi++ ) {
    resamp2_decim_execute( halfband_q2, fptr1, fptr2 );
    fptr1++;
    fptr1++;
    fptr2++;
  }
  decimate *= 4;
#endif

  //(ADC_SIZE/2) samples float complex input,  decimate by 4,  32 samples out in iout, qout

  if( config->sa_mode != MODE_4 && config->sa_mode != MODE_6 ) {

    switch( decimate ) {
    case  1 :
      for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ); _rfi++ ) {
        iout[_rfi] = creal( samples[_rfi] );
        qout[_rfi] = cimag( samples[_rfi] );
      }
      break;

    case  2 :
      do_filter_decimate_2( samples );
      break;

    case  4 :
      do_filter_decimate_4( samples );
      break;

    case  8 :
      do_filter_decimate_8( samples );
      break;

    case  16 :
      do_filter_decimate_16( samples );
      break;

    default :
      decimate = 4;
      do_filter_decimate_4( samples );
      break;
    }



    //must be after dc offset correction
    for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
      stats_buffer_I[_rfi] = iout[_rfi];
      stats_buffer_Q[_rfi] = qout[_rfi];
      stats_buffer[( int ) stats_idx++] = 1.0 / Q_rsqrt( iout[_rfi] * iout[_rfi] + qout[_rfi] * qout[_rfi] );
      stats_idx &= STATS_SIZE - 1;
    }
  } else {


#if 0
    for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
      stats_buffer[( int ) stats_idx++] = ( float ) creal( samples[_rfi] ) * fft_win_128[_rfi];
      stats_buffer[( int ) stats_idx++] = ( float ) cimag( samples[_rfi] ) * fft_win_128[_rfi];
      stats_idx &= STATS_SIZE - 1;
    }
#else
    decimate = 8;
    do_filter_decimate_8( samples );
    for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
      stats_buffer[( int ) stats_idx++] = ( float ) iout[_rfi];
      stats_buffer[( int ) stats_idx++] = ( float ) qout[_rfi];
      stats_idx &= STATS_SIZE - 1;
    }
#endif
    if( stats_idx == 0 ) fft_buffer_ready = 1;
    return;
  }




  switch( config->sa_mode ) {
  case  MODE_FM :

    gain_f = update_gain_cf(iout, (ADC_SIZE/2) , 22050.0f); 
    for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
      iout[_rfi] *= gain_f;
      qout[_rfi] *= gain_f;
    }


    //convert to float complex
    for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
      samples[_rfi] = ( float complex )( ( ( ( ( float ) iout[_rfi] ) / 1.0 ) + _Complex_I * ( ( ( float ) qout[_rfi] ) / 1.0 ) ) );
    }
    if( config->audio_filter == 1 ) {
      do_fm_demod( ( ADC_SIZE / 2 ), decimate );
    }
    else {
      do_fm_demod2( ( ADC_SIZE / 2 ), decimate );
    }




    if(config->udp_mode==UDP_STREAM_IQ16_DECFLT) {  //send rf data over udp instead of audio
      if( udp_out_len == 0 ) {
        //convert back to shorts
        _adc_sptr = ( int16_t * ) &udp_out_buffer[0];
        for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
          *_adc_sptr++ = ( int16_t )( iout[_rfi] / 1.0 );
          *_adc_sptr++ = ( int16_t )( qout[_rfi] / 1.0 );
        }
        udp_out_len = ( ( ADC_SIZE / 2 ) / decimate ) * 4;
      } else if( udp_out_len == ( ( ADC_SIZE / 2 ) / decimate ) * 4 ) {
        //convert back to shorts
        _adc_sptr = ( int16_t * ) &udp_out_buffer[( ( ADC_SIZE / 2 ) / decimate ) * 4];
        for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
          *_adc_sptr++ = ( int16_t )( iout[_rfi] / 1.0 );
          *_adc_sptr++ = ( int16_t )( qout[_rfi] / 1.0 );
        }
        udp_out_len = ( ( ADC_SIZE / 2 ) / decimate ) * 8;
      }
    }

    break;
  case  MODE_P25 :
  case  MODE_DMR :
  case  MODE_SYNCMON :

    gain_f = update_gain_cf(iout, (ADC_SIZE/2) , 22050.0f); 
    for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
      iout[_rfi] *= gain_f;
      qout[_rfi] *= gain_f;
    }

    //convert to float complex
    for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
      samples[_rfi] = ( float complex )( ( ( ( ( float ) iout[_rfi] ) / 1.0 ) + _Complex_I * ( ( ( float ) qout[_rfi] ) / 1.0 ) ) );
    }
    if(config->channel_bw==0) do_fm_demod( ( ADC_SIZE / 2 ), decimate );
    if(config->channel_bw==1) do_fm_demod2( ( ADC_SIZE / 2 ), decimate );
    if(config->channel_bw==2) do_fm_demod2( ( ADC_SIZE / 2 ), decimate );

    if(config->udp_mode==UDP_STREAM_IQ16_DECFLT) {  //send rf data over udp instead of audio
      if( udp_out_len == 0 ) {
        //convert back to shorts
        _adc_sptr = ( int16_t * ) &udp_out_buffer[0];
        for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
          *_adc_sptr++ = ( int16_t )( iout[_rfi] / 1.0 );
          *_adc_sptr++ = ( int16_t )( qout[_rfi] / 1.0 );
        }
        udp_out_len = ( ( ADC_SIZE / 2 ) / decimate ) * 4;
      } else if( udp_out_len == ( ( ADC_SIZE / 2 ) / decimate ) * 4 ) {
        //convert back to shorts
        _adc_sptr = ( int16_t * ) &udp_out_buffer[( ( ADC_SIZE / 2 ) / decimate ) * 4];
        for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
          *_adc_sptr++ = ( int16_t )( iout[_rfi] / 1.0 );
          *_adc_sptr++ = ( int16_t )( qout[_rfi] / 1.0 );
        }
        udp_out_len = ( ( ADC_SIZE / 2 ) / decimate ) * 8;
      }
    }

    break;
  case  MODE_AM :
    //convert to float complex
    for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
      samples[_rfi] = ( float complex )( ( ( ( ( float ) iout[_rfi] ) / 1.0 ) + _Complex_I * ( ( ( float ) qout[_rfi] ) / 1.0 ) ) );
    }
    do_am_demod( ( ADC_SIZE / 2 ), decimate );

    if(config->udp_mode==UDP_STREAM_IQ16_DECFLT) {  //send rf data over udp instead of audio
      if( udp_out_len == 0 ) {
        //convert back to shorts
        _adc_sptr = ( int16_t * ) &udp_out_buffer[0];
        for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
          *_adc_sptr++ = ( int16_t )( iout[_rfi] / 1.0 );
          *_adc_sptr++ = ( int16_t )( qout[_rfi] / 1.0 );
        }
        udp_out_len = ( ( ADC_SIZE / 2 ) / decimate ) * 4;
      } else if( udp_out_len == ( ( ADC_SIZE / 2 ) / decimate ) * 4 ) {
        //convert back to shorts
        _adc_sptr = ( int16_t * ) &udp_out_buffer[( ( ADC_SIZE / 2 ) / decimate ) * 4];
        for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
          *_adc_sptr++ = ( int16_t )( iout[_rfi] / 1.0 );
          *_adc_sptr++ = ( int16_t )( qout[_rfi] / 1.0 );
        }
        udp_out_len = ( ( ADC_SIZE / 2 ) / decimate ) * 8;
      }
    }
    break;

  case  MODE_IQ :
    if( udp_out_len == 0 ) {
      //convert back to shorts
      _adc_sptr = ( int16_t * ) &udp_out_buffer[0];
      for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
        *_adc_sptr++ = ( int16_t )( iout[_rfi] / 1.0 );
        *_adc_sptr++ = ( int16_t )( qout[_rfi] / 1.0 );
      }
      udp_out_len = ( ( ADC_SIZE / 2 ) / decimate ) * 4;
    } else if( udp_out_len == ( ( ADC_SIZE / 2 ) / decimate ) * 4 ) {
      //convert back to shorts
      _adc_sptr = ( int16_t * ) &udp_out_buffer[( ( ADC_SIZE / 2 ) / decimate ) * 4];
      for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
        *_adc_sptr++ = ( int16_t )( iout[_rfi] / 1.0 );
        *_adc_sptr++ = ( int16_t )( qout[_rfi] / 1.0 );
      }
      udp_out_len = ( ( ADC_SIZE / 2 ) / decimate ) * 8;
    }
    break;

  case  MODE_4 :
    if( udp_out_len == 0 ) {
      //convert back to shorts
      _adc_sptr = ( int16_t * ) &udp_out_buffer[0];
      for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
        *_adc_sptr++ = ( int16_t ) creal( samples[_rfi] );
        *_adc_sptr++ = ( int16_t ) cimag( samples[_rfi] );
      }
      udp_out_len = ( ( ADC_SIZE / 2 ) / decimate ) * 4;
    } else if( udp_out_len == ( ( ADC_SIZE / 2 ) / decimate ) * 4 ) {
      //convert back to shorts
      _adc_sptr = ( int16_t * ) &udp_out_buffer[( ( ADC_SIZE / 2 ) / decimate ) * 4];
      for( _rfi = 0; _rfi < ( ADC_SIZE / 2 ) / decimate; _rfi++ ) {
        *_adc_sptr++ = ( int16_t ) creal( samples[_rfi] );
        *_adc_sptr++ = ( int16_t ) cimag( samples[_rfi] );
      }
      udp_out_len = ( ( ADC_SIZE / 2 ) / decimate ) * 8;
    }
    break;
  }

  do_sample_process = 1;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void resamp_reset( void )
{
  resamp_cf_reset( ch_resamp_q_narrow );
  resamp_cf_reset( ch_resamp_q_wide );
  firfilt_rf_reset( audio_fir );
  firfilt_rf_reset( audio_fir2 );
  firfilt_rf_reset( audio_fir3 );
  firfilt_rf_reset( audio_fir4 );
  firfilt_rf_reset( audio_fir5 );
  firfilt_rf_reset( audio_fir6 );
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void do_am_demod( int len, int decimate )
{


  int i;
  int _rfi;


  switch( config->channel_bw ) {
  case  0 :
    resamp_cf_execute_block( ch_resamp_q_narrow, samples, len / decimate, ch_resamp_y, &ch_ny );
    break;

  case  1 :
    resamp_cf_execute_block( ch_resamp_q_wide, samples, len / decimate, ch_resamp_y, &ch_ny );
    break;

  case  2 :
    resamp_cf_execute_block( ch_resamp_q_med, samples, len / decimate, ch_resamp_y, &ch_ny );
    break;
  }

  ampmodem_demodulate_block( am_dem, ch_resamp_y, ch_ny, fm_out );

  uint8_t *ptr;

  for( _rfi = 0; _rfi < ch_ny; _rfi++ ) {

    switch( config->audio_filter ) {
    case  0 :
      fm_filtered = fm_out[_rfi];  //no post-FM filtering.  for testing
      audio_out_level = 0.3;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  1 :
      //audio fir with slow roll-off.  Good for P25, and wider signals
      firfilt_rf_push( audio_fir, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir, &fm_filtered );
      audio_out_level = 0.3;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  2 :
      //3khz or so
      firfilt_rf_push( audio_fir2, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir2, &fm_filtered );
      audio_out_level = 0.3;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  3 :
      firfilt_rf_push( audio_fir3, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir3, &fm_filtered );
      audio_out_level = 0.3;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  4 :
      firfilt_rf_push( audio_fir4, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir4, &fm_filtered );
      audio_out_level = 0.3;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  5 :
      firfilt_rf_push( audio_fir5, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir5, &fm_filtered );
      audio_out_level = 32000.0 * 5.0;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    default :
      //audio fir with slow roll-off.  Good for P25, and wider signals
      firfilt_rf_push( audio_fir, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir, &fm_filtered );
      audio_out_level = 0.3;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    }

    audio_buffer[_rfi] = audio_sample;
  }

  //keep the target output low until we know there is a signal there
  if(get_rssi()>-110) {
    update_gain_s16(audio_buffer, ch_ny, 8000.0f, 30.0f); 
  }
  else {
    update_gain_s16(audio_buffer, ch_ny, 1000.0f, 30.0f); 
  }

  for( _rfi = 0; _rfi < ch_ny; _rfi++ ) {
    audio_sample = audio_buffer[_rfi];

    out_buffer[out_e++] = audio_buffer[_rfi];
    out_e &= OUT_BUFFER_SIZE - 1;

    audio_out_rate++;

    if( udp_out_len < ( ADC_SIZE * 2 ) && config->udp_mode!=UDP_STREAM_IQ16_DECFLT) {

      if( config->udp_mode==UDP_STREAM_AUDIO_48K_S16_CONT && current_rssi < config->squelch ) {
        audio_sample=0;
      }
      else if( config->udp_mode==UDP_STREAM_AUDIO_48K_S16 && current_rssi < config->squelch ) {
        udp_out_len=0; 
        break;
      }

      ptr = ( uint8_t * ) &audio_sample;
      udp_out_buffer[udp_out_len++] = *ptr++;
      udp_out_buffer[udp_out_len++] = *ptr++;
    }
  }

}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void do_fm_demod( int len, int decimate )
{


  int i;
  int _rfi;

  resamp_cf_execute_block( ch_resamp_q_narrow, samples, len / decimate, ch_resamp_y, &ch_ny );

  freqdem_demodulate( ch_resamp_y, ch_ny, fm_out ); //use fast ATAN method for demod

  uint8_t *ptr;

  for( _rfi = 0; _rfi < ch_ny; _rfi++ ) {

    switch( config->audio_filter ) {
    case  0 :
      fm_filtered = fm_out[_rfi];  //no post-FM filtering.  for testing
      audio_out_level = 40000.0f * 4.0;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  1 :
      //audio fir with slow roll-off.  Good for P25, and wider signals
      firfilt_rf_push( audio_fir, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir, &fm_filtered );
      audio_out_level = 32000.0f;

      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  2 :
      //3khz or so
      firfilt_rf_push( audio_fir2, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir2, &fm_filtered );
      audio_out_level = 18000.0f;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  3 :
      firfilt_rf_push( audio_fir3, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir3, &fm_filtered );
      audio_out_level = 5000.0f * 3.3;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  4 :
      firfilt_rf_push( audio_fir4, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir4, &fm_filtered );
      audio_out_level = 5000.0f;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  5 :
      firfilt_rf_push( audio_fir5, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir5, &fm_filtered );
      audio_out_level = 32000.0 * 5.0;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    default :
      //audio fir with slow roll-off.  Good for P25, and wider signals
      firfilt_rf_push( audio_fir, fm_out[_rfi] );
      firfilt_rf_execute( audio_fir, &fm_filtered );
      audio_out_level = 10000.0f * 3.3;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    }


    if( config->aud_dc_off_rate != 0.0 ) {
      //dc offset correction
      f_avg_i = f_avg_i * f_beta_i + audio_sample * f_alpha_i;
      audio_sample -= f_avg_i;
    }


    switch( config->sa_mode ) {

    case  MODE_FM :
      out_buffer[out_e++] = audio_sample;
      out_e &= OUT_BUFFER_SIZE - 1;
      break;

    case  MODE_P25 :
    case  MODE_DMR :
    case  MODE_SYNCMON :
      if( in_buffer_len < sizeof( in_buffer ) - ( ADC_SIZE ) ) {
        ptr = ( uint8_t * ) &audio_sample;
        in_buffer[in_buffer_len++] = *ptr++;
        in_buffer[in_buffer_len++] = *ptr++;
      }
      break;
    }

    audio_out_rate++;

    if( udp_out_len < ( ADC_SIZE * 2 ) && config->udp_mode!=UDP_STREAM_IQ16_DECFLT) {

      if( config->udp_mode==UDP_STREAM_AUDIO_48K_S16_CONT && current_rssi < config->squelch ) {
        audio_sample=0;
      }
      else if( config->udp_mode==UDP_STREAM_AUDIO_48K_S16 && current_rssi < config->squelch ) {
        udp_out_len=0; 
        break;
      }

      ptr = ( uint8_t * ) &audio_sample;
      udp_out_buffer[udp_out_len++] = *ptr++;
      udp_out_buffer[udp_out_len++] = *ptr++;
    }
  }

}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void audio_adjust_rate( float rate )
{
  uint32_t prim;
  prim = __get_PRIMASK();

  __disable_irq();

  a_avg = a_avg * a_beta + rate * a_alpha;
  audio_adj_rate = a_avg;

  if( config->sa_mode == MODE_P25 || config->sa_mode==MODE_DMR || config->sa_mode==MODE_SYNCMON) {
    //this doesn't work very well, but not really needed anyway since P25 is not constant streaming audio
    //resamp_rf_set_rate( audio_resamp_q, audio_adj_rate/8000.0);  //P25 audio resample 8k -> 48k
  } else {

    switch( config->channel_bw ) {
    case  0 :
      //resamp_rf_adjust_rate(ch_resamp_q_narrow, rate);
      resamp_rf_set_rate( ch_resamp_q_narrow, audio_adj_rate / CH_NARROW_RATE_DIV4 ); //14.x khz -> 48khz
      break;

    case  1 :
      //resamp_rf_adjust_rate(ch_resamp_q_wide, rate);
      resamp_rf_set_rate( ch_resamp_q_wide, audio_adj_rate / CH_WIDE_RATE_DIV4 ); //200 khz -> 48khz
      break;

    case  2 :
      //resamp_rf_adjust_rate(ch_resamp_q_narrow, rate);
      resamp_rf_set_rate( ch_resamp_q_med, audio_adj_rate / CH_MED_RATE_DIV4 ); //14.x khz -> 48khz
      break;
    }

  }
  if( !prim ) {
    __enable_irq();
  }

}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void do_fm_demod2( int len, int decimate )
{


  int i;
  int _rfi;
  float fm_i;
  float fm_q;


  freqdem_demodulate( samples, len / decimate, fm_out ); //use fast ATAN method for demod
  for( _rfi = 0; _rfi < ADC_SIZE / decimate; _rfi++ ) {
    samples[_rfi] = ( float complex )( ( ( ( ( float ) fm_out[_rfi] ) ) + _Complex_I * ( ( ( float ) 0.0f ) ) ) );
  }

  switch( config->channel_bw ) {
  case  0 :
    resamp_cf_execute_block( ch_resamp_q_narrow, samples, len / decimate, ch_resamp_y, &ch_ny );
    break;

  case  1 :
    resamp_cf_execute_block( ch_resamp_q_wide, samples, len / decimate, ch_resamp_y, &ch_ny );
    break;
  case  2 :
    resamp_cf_execute_block( ch_resamp_q_med, samples, len / decimate, ch_resamp_y, &ch_ny );
    break;
  }


  uint8_t *ptr;


  for( _rfi = 0; _rfi < ch_ny; _rfi++ ) {


    switch( config->audio_filter ) {
    case  0 :
      fm_filtered = creal( ch_resamp_y[_rfi] ); //no post-FM filtering.  for testing
      audio_out_level = 32000.0f;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  1 :
      //audio fir with slow roll-off.  Good for P25, and wider signals
      firfilt_rf_push( audio_fir, creal( ch_resamp_y[_rfi] ) );
      firfilt_rf_execute( audio_fir, &fm_filtered );
      audio_out_level = 20000.0f;
      if(config->channel_bw==2) audio_out_level *= 2.0;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  2 :
      //3khz or so
      firfilt_rf_push( audio_fir2, creal( ch_resamp_y[_rfi] ) );
      firfilt_rf_execute( audio_fir2, &fm_filtered );
      audio_out_level = 6500.0;
      audio_sample = ( short )( fm_filtered * audio_out_level );

      break;

    case  3 :
      firfilt_rf_push( audio_fir3, creal( ch_resamp_y[_rfi] ) );
      firfilt_rf_execute( audio_fir3, &fm_filtered );
      audio_out_level = 5000.0;
      audio_sample = ( short )( fm_filtered * audio_out_level );

      break;

    case  4 :
      firfilt_rf_push( audio_fir4, creal( ch_resamp_y[_rfi] ) );
      firfilt_rf_execute( audio_fir4, &fm_filtered );
      audio_out_level = 5000.0;
      audio_sample = ( short )( fm_filtered * audio_out_level );

      break;

    case  5 :
      firfilt_rf_push( audio_fir5, creal( ch_resamp_y[_rfi] ) );
      firfilt_rf_execute( audio_fir5, &fm_filtered );
      audio_out_level = 32000.0;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  6 :
      //3khz or so
      firfilt_rf_push( audio_fir6, creal( ch_resamp_y[_rfi] ) );
      firfilt_rf_execute( audio_fir6, &fm_filtered );
      audio_out_level = 16000.0;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    case  7 :
      //audio fir with slow roll-off.  Good for P25, and wider signals
      firfilt_rf_push( audio_fir, creal( ch_resamp_y[_rfi] ) );
      firfilt_rf_execute( audio_fir, &fm_filtered );
      audio_out_level = 32000.0f;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    default :
      //audio fir with slow roll-off.  Good for P25, and wider signals
      firfilt_rf_push( audio_fir, creal( ch_resamp_y[_rfi] ) );
      firfilt_rf_execute( audio_fir, &fm_filtered );
      audio_out_level = 32000.0;
      audio_sample = ( short )( fm_filtered * audio_out_level );
      break;

    }



    if( config->aud_dc_off_rate != 0.0 ) {
      //dc offset correction
      f_avg_i = f_avg_i * f_beta_i + audio_sample * f_alpha_i;
      audio_sample -= f_avg_i;
    }


    switch( config->sa_mode ) {

    case  MODE_FM :

      out_buffer[out_e++] = audio_sample;
      out_e &= OUT_BUFFER_SIZE - 1;
      break;

    case  MODE_P25 :
    case  MODE_DMR :
    case  MODE_SYNCMON :
      if( in_buffer_len < sizeof( in_buffer ) - ADC_SIZE ) {
        ptr = ( uint8_t * ) &audio_sample;
        in_buffer[in_buffer_len++] = *ptr++;
        in_buffer[in_buffer_len++] = *ptr++;
      }
      break;
    }

    audio_out_rate++;

    if( udp_out_len < ( ADC_SIZE * 2 ) && config->udp_mode!=UDP_STREAM_IQ16_DECFLT) {

      if( config->udp_mode==UDP_STREAM_AUDIO_48K_S16_CONT && current_rssi < config->squelch ) {
        audio_sample=0;
      }
      else if( config->udp_mode==UDP_STREAM_AUDIO_48K_S16 && current_rssi < config->squelch ) {
        udp_out_len=0; 
        break;
      }

      ptr = ( uint8_t * ) &audio_sample;
      udp_out_buffer[udp_out_len++] = *ptr++;
      udp_out_buffer[udp_out_len++] = *ptr++;
    }
  }

}
///////////////////////////////////////////////////////////////////////////////////////////////
// up-sample voice audio from 8 Ksps to 48 Ksps using poly-phase fir filter
///////////////////////////////////////////////////////////////////////////////////////////////
int upsample_audio( uint8_t *buffer, int len )
{

  if( len > 320 ) return 0; //not audio

#if 1

  int i;
  short *ptr = ( short * ) buffer;
  for( i = 0; i < len / 2; i++ ) {
    audio_resamp_x[i] = ( float ) * ptr++;
  }

  //upsample from 8kHz to 48kHz 
  unsigned int a_ny = 0;
  resamp_rf_execute_block( audio_resamp_q, audio_resamp_x, len / 2, audio_resamp_y, &a_ny );

  for( i = 0; i < a_ny; i++ ) {
    upsampled[i] = ( short ) audio_resamp_y[i];
  }

  upsample_ready = 1;

  return a_ny;
#endif
}


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void init_audio_dc_offset( double val )
{
  uint32_t prim;
  prim = __get_PRIMASK();

  __disable_irq();

  ALPHA_F_i = 1.0 / val;
  f_alpha_i = ALPHA_F_i;
  f_beta_i = 1.0 - f_alpha_i;
  f_avg_i = 0.0;

  if( !prim ) {
    __enable_irq();
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void init_dc_correction( double val )
{
  uint32_t prim;
  prim = __get_PRIMASK();

  __disable_irq();
  ALPHA_D_i = 1.0 / val;
  d_alpha_i = ALPHA_D_i;
  d_beta_i = 1.0 - d_alpha_i;
  //d_avg_i = 0.0;  //don't reset the integrator

  ALPHA_D_q = 1.0 / val;
  d_alpha_q = ALPHA_D_q;
  d_beta_q = 1.0 - d_alpha_i;
  //d_avg_q = 0.0;  //don't reset the integrator

  ALPHA_A = 1.0 / 10.0;
  a_alpha = ALPHA_A;
  a_beta = 1.0 - a_alpha;
  a_avg = DECODER_FS;

  if( !prim ) {
    __enable_irq();
  }
}



