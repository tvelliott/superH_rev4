

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
#include "rtl_tcp_driver.h"
#include "freqsynth.h"
#include "config.h"

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void dump_regs()
{

  uint8_t address;
  uint8_t reg;

  printf( "\r\n synth1" );

  for( address = 0; address < 0x0f; address++ ) {
    reg = synth_read_reg( address );
    printf( "\r\n%02x: %02x", address, reg );
  }

  printf( "\r\n synth2" );

  for( address = 0; address < 0x0f; address++ ) {
    reg = synth2_read_reg( address );
    printf( "\r\n%02x: %02x", address, reg );
  }

  printf( "\r\n mixer" );

  for( address = 0; address < 0x18; address++ ) {
    reg = mixer_read_reg( address );
    printf( "\r\n%02x: %02x", address, reg );
  }

  printf( "\r\n" );
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
uint8_t wait_for_lock( void )
{
  uint8_t lock;
  int cnt = 50;
  return;

  while( --cnt > 0 ) {
    lock = ( synth_read_reg( 0x00 ) & 0x24 );
    if( lock == 4 ) {
      //if(config->sa_mode!=MODE_P25 && config->sa_mode!=MODE_6) printf("\r\npll1 locked");
      return 0x01;
    }
    delay_ms_ni( 2 );
  }

  if( cnt == 0 ) {
    printf( "\r\npll1 did not lock!!, %02x", lock );
  }

  return 0x00;
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
uint8_t wait_for_lock2( void )
{
  uint8_t lock;
  int cnt = 50;
  return;

  while( --cnt > 0 ) {
    lock = ( synth2_read_reg( 0x00 ) & 0x24 );
    if( lock == 4 ) {
      //if(config->sa_mode!=MODE_P25 && config->sa_mode!=MODE_6) printf("\r\npll2 locked");
      return 0x01;
    }
    delay_ms_ni( 2 );
  }

  if( cnt == 0 ) {
    printf( "\r\npll2 did not lock!!, %02x", lock );
  }

  return 0x00;
}

///////////////////////////////////////////////////////////////////////////////
//  ODIV 1-6
//  R = 1-31
//  F = 1 - 262143   (2^18-1)
//  N = 32-511
///////////////////////////////////////////////////////////////////////////////
void set_freq_mhz( double freq )
{
  uint32_t prim;

  uint8_t ref_div = REF_DIV;
  double PFD = REF_ACTUAL / ( double ) ref_div; //PFD frequency must be <= 100 MHz

  return;


  if( freq < 0.5 + config->if_frequency || freq > 1500.0 + config->if_frequency ) {
    printf( "\r\nfreq is out of range (0.5 - 1500.0)" );
    return;
  }

  prim = __get_PRIMASK();
  __disable_irq();

  int tmp = config->gain1;

  //config->gain1 = 5;

  uint8_t synth1_gain;
  int tmp_atten = config->front_end_atten;
  uint8_t tmp_bw = config->adc_flt_bw;
  uint8_t tmp_gain1 = config->gain1;

  set_atten( 63 );
  set_gain1( 0 );
  set_bw( 0 );

  //config->frequency = freq - config->if_frequency;

  double tune_freq = freq - config->if_frequency;

#if 0
  if( tune_freq < 100.0 ) synth1_gain = 2;
  else if( tune_freq >= 100.0 && tune_freq < 300.0 ) synth1_gain = 2;
  else if( tune_freq >= 300.0 && tune_freq < 550.0 ) synth1_gain = 0;
  else if( tune_freq >= 550.0 && tune_freq < 1000.0 ) synth1_gain = 2;
  else synth1_gain = 3;
#else
  synth1_gain = 3;
#endif

  //TODO: FIX after new parts arrive
  //synth1_gain=0;

  //printf("\r\nsynth1 gain: %d", synth1_gain);

  synth1_gain &= 0x03;
  synth1_gain <<= 3;

  uint8_t cgain = synth_read_reg( 0x0b );
  cgain &= 0xe7;
  cgain |= synth1_gain;
  synth_write_reg( 0x0b, cgain );  //update gain



  double odiv = 1.0;

  freq += config->freq_offset_mhz;
  //if( config->do_low_if ) freq = freq + ( double ) + config->nco_offset_freq; //low-IF offset before down-mix

  int pll_retries = PLL_LOCK_RETRIES;

pll_retry:

  if( pll_retries < PLL_LOCK_RETRIES ) {
    init_synth1();
  }

#ifdef SYNTH1_LTC6948_V4
  //prefer highest output divider possible,  LTC6948-4
  if( freq > 700.0 && freq <= 1065.0 ) {
    odiv = 6;
  } else if( freq > 1065.0 && freq <= 1278.0 ) {
    odiv = 5;
  } else if( freq > 1278.0 && freq <= 1598.0 ) {
    odiv = 4;
  } else if( freq > 1598.0 && freq <= 2130.0 ) {
    odiv = 3;
  } else if( freq > 2130.0 && freq <= 3195.0 ) {
    odiv = 2;
  } else if( freq > 4200.0 && freq <= 6390.0 ) {
    odiv = 1;
  }
#endif

#ifdef SYNTH1_LTC6948_V1
  //prefer highest output divider possible,  LTC6948-1
  if( freq > 373.0 && freq <= 623.0 ) {
    odiv = 6;
  } else if( freq > 448.0 && freq <= 748.0 ) {
    odiv = 5;
  } else if( freq > 560.0 && freq <= 935.0 ) {
    odiv = 4;
  } else if( freq > 747.0 && freq <= 1247.0 ) {
    odiv = 3;
  } else if( freq > 1120.0 && freq <= 1870.0 ) {
    odiv = 2;
  } else if( freq > 2240.0 && freq <= 3740.0 ) {
    odiv = 1;
  }
#endif

#ifdef SYNTH1_LTC6948_V3
  //prefer highest output divider possible,  LTC6948-3
  if( freq > 640.0 && freq <= 965.0 ) {
    odiv = 6;
  } else if( freq > 768.0 && freq <= 1158.0 ) {
    odiv = 5;
  } else if( freq > 960.0 && freq <= 1448.0 ) {
    odiv = 4;
  } else if( freq > 1280.0 && freq <= 1930.0 ) {
    odiv = 3;
  } else if( freq > 1920.0 && freq <= 2895.0 ) {
    odiv = 2;
  } else if( freq > 3840.0 && freq <= 5790.0 ) {
    odiv = 1;
  }
#endif


  if( pll_retries == 2 ) odiv += 1;
  if( pll_retries == 1 ) odiv -= 1;

  double fvco = freq * odiv;

  uint8_t lkwin = 1;
  //FOR CPLE=1
  if( fvco >= 2970 ) {
    lkwin = 0;
  } else if( fvco >= 2000.0 ) {
    lkwin = 1;
  } else if( fvco >= 1390.0 ) {
    lkwin = 2;
  } else if( fvco >= 941.0 ) {
    lkwin = 3;
  } else if( fvco >= 646.0 ) {
    lkwin = 4;
  } else if( fvco >= 431.0 ) {
    lkwin = 5;
  } else if( fvco >= 294.0 ) {
    lkwin = 6;
  } else if( fvco >= 196.0 ) {
    lkwin = 7;
  }
  uint8_t reg0c = synth_read_reg( 0x0c );
  reg0c &= 0x1f;
  reg0c |= ( lkwin << 5 );
  //reg0c |= 0x18;  //lkct = 3
  reg0c |= 0x08;  //lkct = 1
  synth_write_reg( 0x0c, reg0c );


  double N = 1.0;

  while( ( fvco / N ) > PFD ) {
    N += 1.0;
    if( N >= 1023 ) break;
  }


  double freq_int = N * PFD;
  double freq_err = ( freq_int - fvco );
  double F = 262143 - ( ( freq_err / PFD ) * 262143 );

  //if(freq_err!=0) N-=1.0;
  N -= 1.0;

  uint32_t FINT = ( int ) F;

  uint32_t F1 = FINT >> 12;
  uint32_t F2 = ( ( FINT >> 4 ) & 0xff );
  uint32_t F3 = ( FINT & 0x0f ) << 4;

  uint8_t reg08 = synth_read_reg( 0x08 );
  uint8_t reg09 = synth_read_reg( 0x09 );
  uint8_t reg0a = synth_read_reg( 0x0a );

  reg08 = F1;

  reg09 = F2;

  reg0a &= 0x0f;
  reg0a |= F3;

  synth_write_reg( 0x08, reg08 );
  synth_write_reg( 0x09, reg09 );
  synth_write_reg( 0x0a, reg0a );

  uint32_t NINT = ( int ) N;
  uint32_t N1 = NINT >> 8;
  uint32_t N2 = ( NINT & 0xff );

  uint8_t reg06 = synth_read_reg( 0x06 );
  reg06 &= 0xe0;
  reg06 |= ( uint8_t ) N1;
  reg06 |= ref_div << 3;

  uint8_t reg07 = ( uint8_t ) N2;

  synth_write_reg( 0x06, reg06 ); //N value
  synth_write_reg( 0x07, reg07 ); //N value

  uint8_t reg0b = synth_read_reg( 0x0b );
  reg0b &= 0xf8;
  reg0b |= ( uint8_t ) odiv;
  synth_write_reg( 0x0b, reg0b ); //ODIV value




#if 0
  printf( "\r\nN1 = 0x%02x", ( uint8_t ) N1 & 0xff );
  printf( "\r\nN2 = 0x%02x", ( uint8_t ) N2 & 0xff );

  printf( "\r\nF1 = 0x%02x", ( uint8_t ) F1 & 0xff );
  printf( "\r\nF2 = 0x%02x", ( uint8_t ) F2 & 0xff );
  printf( "\r\nF3 = 0x%02x", ( uint8_t ) F3 & 0xff );

  printf( "\r\nref = %f", PFD );
  printf( "\r\nfvco = %f", fvco );
  printf( "\r\nO = %d", ( int ) odiv );
  printf( "\r\nN = %d", ( int ) N );
  printf( "\r\nF = %d", ( int ) F );
  printf( "\r\nLKWIN = %d", ( int ) lkwin );
#endif

  if( wait_for_lock() == 0 && pll_retries-- > 0 ) goto pll_retry;

  channel_settle = CHANNEL_SETTLE_TIME;
  stats_buffer_ready = 0;


#if 0
  if( freq >= 30.0 ) {
    switch( config->agc_mode ) {
    case  1 :
    case  2 :
    case  3 :
    case  4 :
      set_atten( 0 );
      set_bbatten( 0 );
      break;

    case  5 :
      set_atten( 15 );
      break;

    case  6 :
      set_atten( 0 );
      set_bbatten( 31 );
      break;

    default :
      break;
    }
  }
#endif

  //set_freq_mhz2( config->if_frequency );
  //wait_for_lock2();

  set_gain1( tmp_gain1 );
  set_bw( tmp_bw );
  set_atten( tmp_atten );

  //resamp_reset();

  if( !prim ) {
    __enable_irq();
  }
}

///////////////////////////////////////////////////////////////////////////////
//  ODIV 1-6
//  R = 1-31
//  F = 1 - 262143   (2^18-1)
//  N = 32-511
///////////////////////////////////////////////////////////////////////////////
void set_freq_mhz2( double freq )
{
  uint8_t ref_div = REF_DIV;
  double PFD = REF_ACTUAL / ( double ) ref_div; //PFD frequency must be <= 100 MHz

  double odiv = 1.0;
  uint32_t prim;

  return;

  prim = __get_PRIMASK();
  __disable_irq();

  //config->gain1 = 5;

  //works best from 1554 to 1583 MHz,  best to push it to one side or the other
  if(freq<1550.0) freq = 1550.0;  
  if(freq>1587.0) freq = 1587.0;  

  int tmp_atten = config->front_end_atten;
  uint8_t tmp_bw = config->adc_flt_bw;
  uint8_t tmp_gain1 = config->gain1;

  set_atten( 63 );
  set_gain1( 0 );
  set_bw( 0 );


  int pll_retries = PLL_LOCK_RETRIES;

pll_retry2:

  if( pll_retries < PLL_LOCK_RETRIES ) {
    init_synth2();
  }

  //rf_mute();

#ifdef SYNTH2_LTC6948_V4
  //prefer highest output divider possible,  LTC6948-4
  if( freq > 700.0 && freq <= 1065.0 ) {
    odiv = 6;
  } else if( freq > 1065.0 && freq <= 1278.0 ) {
    odiv = 5;
  } else if( freq > 1278.0 && freq <= 1598.0 ) {
    odiv = 4;
  } else if( freq > 1598.0 && freq <= 2130.0 ) {
    odiv = 3;
  } else if( freq > 2130.0 && freq <= 3195.0 ) {
    odiv = 2;
  } else if( freq > 4200.0 && freq <= 6390.0 ) {
    odiv = 1;
  }
#endif

#ifdef SYNTH2_LTC6948_V1
  //prefer highest output divider possible,  LTC6948-1
  if( freq > 373.0 && freq <= 623.0 ) {
    odiv = 6;
  } else if( freq > 448.0 && freq <= 748.0 ) {
    odiv = 5;
  } else if( freq > 560.0 && freq <= 935.0 ) {
    odiv = 4;
  } else if( freq > 747.0 && freq <= 1247.0 ) {
    odiv = 3;
  } else if( freq > 1120.0 && freq <= 1870.0 ) {
    odiv = 2;
  } else if( freq > 2240.0 && freq <= 3740.0 ) {
    odiv = 1;
  }
#endif

#ifdef SYNTH2_LTC6948_V3
  //prefer highest output divider possible,  LTC6948-3
  if( freq > 640.0 && freq <= 965.0 ) {
    odiv = 6;
  } else if( freq > 768.0 && freq <= 1158.0 ) {
    odiv = 5;
  } else if( freq > 960.0 && freq <= 1448.0 ) {
    odiv = 4;
  } else if( freq > 1280.0 && freq <= 1930.0 ) {
    odiv = 3;
  } else if( freq > 1920.0 && freq <= 2895.0 ) {
    odiv = 2;
  } else if( freq > 3840.0 && freq <= 5790.0 ) {
    odiv = 1;
  }
#endif

  double fvco = freq * odiv;
  if( pll_retries == 2 ) odiv += 1;
  if( pll_retries == 1 ) odiv -= 1;

  uint8_t lkwin = 1;
  //FOR CPLE=1
  if( fvco >= 2970 ) {
    lkwin = 0;
  } else if( fvco >= 2000.0 ) {
    lkwin = 1;
  } else if( fvco >= 1390.0 ) {
    lkwin = 2;
  } else if( fvco >= 941.0 ) {
    lkwin = 3;
  } else if( fvco >= 646.0 ) {
    lkwin = 4;
  } else if( fvco >= 431.0 ) {
    lkwin = 5;
  } else if( fvco >= 294.0 ) {
    lkwin = 6;
  } else if( fvco >= 196.0 ) {
    lkwin = 7;
  }
  uint8_t reg0c = synth2_read_reg( 0x0c );
  reg0c &= 0x1f;
  reg0c |= ( lkwin << 5 );
  //reg0c |= 0x18;  //lkct = 3
  reg0c |= 0x08;  //lkct = 1
  synth2_write_reg( 0x0c, reg0c );

  double N = 1.0;

  while( ( fvco / N ) > PFD ) {
    N += 1.0;
    if( N >= 1023 ) break;
  }

  double freq_int = N * PFD;
  double freq_err = ( freq_int - fvco );
  double F = 262143 - ( ( freq_err / PFD ) * 262143 );

  //if(freq_err!=0) N-=1.0;
  N -= 1.0;

  uint32_t FINT = ( int ) F;

  uint32_t F1 = FINT >> 12;
  uint32_t F2 = ( ( FINT >> 4 ) & 0xff );
  uint32_t F3 = ( FINT & 0x0f ) << 4;

  uint8_t reg08 = synth2_read_reg( 0x08 );
  uint8_t reg09 = synth2_read_reg( 0x09 );
  uint8_t reg0a = synth2_read_reg( 0x0a );

  reg08 = F1;

  reg09 = F2;

  reg0a &= 0x0f;
  reg0a |= F3;

  synth2_write_reg( 0x08, reg08 );
  synth2_write_reg( 0x09, reg09 );
  synth2_write_reg( 0x0a, reg0a );

  uint32_t NINT = ( int ) N;
  uint32_t N1 = NINT >> 8;
  uint32_t N2 = ( NINT & 0xff );

  uint8_t reg06 = synth2_read_reg( 0x06 );
  reg06 &= 0xe0;
  reg06 |= ( uint8_t ) N1;
  reg06 |= ref_div << 3;

  uint8_t reg07 = ( uint8_t ) N2;

  synth2_write_reg( 0x06, reg06 ); //N value
  synth2_write_reg( 0x07, reg07 ); //N value

  uint8_t reg0b = synth2_read_reg( 0x0b );
  reg0b &= 0xf8;
  reg0b |= ( uint8_t ) odiv;
  synth2_write_reg( 0x0b, reg0b ); //ODIV value


  uint8_t synth2_gain;
#if 0
#else
  synth2_gain = 3;
#endif

  synth2_gain &= 0x03;
  synth2_gain <<= 3;

  uint8_t cgain = synth2_read_reg( 0x0b );
  cgain &= 0xe7;
  cgain |= synth2_gain;
  synth2_write_reg( 0x0b, cgain );  //update gain




  /*
  printf("\r\nN1 = 0x%02x", (uint8_t) N1&0xff);
  printf("\r\nN2 = 0x%02x", (uint8_t) N2&0xff);

  printf("\r\nF1 = 0x%02x", (uint8_t) F1&0xff);
  printf("\r\nF2 = 0x%02x", (uint8_t) F2&0xff);
  printf("\r\nF3 = 0x%02x", (uint8_t) F3&0xff);

  printf("\r\nref = %f", REF);
  printf("\r\nfvco = %f", fvco);
  printf("\r\nO = %d", (int) odiv);
  printf("\r\nN = %d", (int) N);
  printf("\r\nF = %d", (int) F);
  */

  if( wait_for_lock2() == 0 && pll_retries-- > 0 ) goto pll_retry2;
  //rf_unmute();



  set_gain1( tmp_gain1 );
  set_bw( tmp_bw );
  set_atten( tmp_atten );

  if( !prim ) {
    __enable_irq();
  }
}


///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void rf_mute( void )
{
  synth_write_reg( 0x02, 0x06 ); //mute RF output
  synth2_write_reg( 0x02, 0x06 ); //mute RF output
}
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void rf_unmute( void )
{
  synth_write_reg( 0x02, 0x04 ); //un-mute RF output
  synth2_write_reg( 0x02, 0x04 ); //un-mute RF output
}

#define SYNTH_DELAY 50

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void synth1_select()
{
  DelayClk3( SYNTH_DELAY );
  HAL_GPIO_WritePin( synth1_cs_GPIO_Port, synth1_cs_Pin, GPIO_PIN_RESET );
  DelayClk3( SYNTH_DELAY );
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void synth1_deselect()
{
  DelayClk3( SYNTH_DELAY );
  HAL_GPIO_WritePin( synth1_cs_GPIO_Port, synth1_cs_Pin, GPIO_PIN_SET );
  DelayClk3( SYNTH_DELAY );
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void synth2_select()
{
  DelayClk3( SYNTH_DELAY );
  HAL_GPIO_WritePin( synth2_cs_GPIO_Port, synth2_cs_Pin, GPIO_PIN_RESET );
  DelayClk3( SYNTH_DELAY );
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void synth2_deselect()
{
  DelayClk3( SYNTH_DELAY );
  HAL_GPIO_WritePin( synth2_cs_GPIO_Port, synth2_cs_Pin, GPIO_PIN_SET );
  DelayClk3( SYNTH_DELAY );
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void mixer_select()
{
  DelayClk3( SYNTH_DELAY );
  HAL_GPIO_WritePin( mixer_cs_GPIO_Port, mixer_cs_Pin, GPIO_PIN_RESET );
  DelayClk3( SYNTH_DELAY );
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void mixer_deselect()
{
  DelayClk3( SYNTH_DELAY );
  HAL_GPIO_WritePin( mixer_cs_GPIO_Port, mixer_cs_Pin, GPIO_PIN_SET );
  DelayClk3( SYNTH_DELAY );
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
uint8_t synth_read_reg( uint8_t a )
{
  HAL_StatusTypeDef status;
  uint8_t addr = a;
  uint8_t val;
  _xmit_spi( 0xff );

  DelayClk3( SYNTH_DELAY );
  synth1_select();
  DelayClk3( SYNTH_DELAY );

  addr <<= 1;
  addr |= 0x01;  //read operation

  _xmit_spi( addr );
  val = _xmit_spi( 0xff );

  DelayClk3( SYNTH_DELAY );
  synth1_deselect();
  DelayClk3( SYNTH_DELAY );
  return val;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void synth_write_reg( uint8_t a, uint8_t val )
{
  _xmit_spi( 0xff );

  DelayClk3( SYNTH_DELAY );
  synth1_select();
  DelayClk3( SYNTH_DELAY );

  uint8_t addr = a;

  addr <<= 1;
  addr &= 0xfe;  //write operation

  _xmit_spi( addr );
  _xmit_spi( val );

  DelayClk3( SYNTH_DELAY );
  synth1_deselect();
  DelayClk3( SYNTH_DELAY );
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
uint8_t synth2_read_reg( uint8_t a )
{
  HAL_StatusTypeDef status;
  uint8_t addr = a;
  uint8_t val;
  _xmit_spi( 0xff );

  DelayClk3( SYNTH_DELAY );
  synth2_select();
  DelayClk3( SYNTH_DELAY );

  addr <<= 1;
  addr |= 0x01;  //read operation

  _xmit_spi( addr );
  val = _xmit_spi( 0xff );

  DelayClk3( SYNTH_DELAY );
  synth2_deselect();
  DelayClk3( SYNTH_DELAY );
  return val;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void synth2_write_reg( uint8_t a, uint8_t val )
{
  _xmit_spi( 0xff );

  DelayClk3( SYNTH_DELAY );
  synth2_select();
  DelayClk3( SYNTH_DELAY );

  uint8_t addr = a;

  addr <<= 1;
  addr &= 0xfe;  //write operation

  _xmit_spi( addr );
  _xmit_spi( val );

  DelayClk3( SYNTH_DELAY );
  synth2_deselect();
  DelayClk3( SYNTH_DELAY );
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
uint8_t mixer_read_reg( uint8_t a )
{
  HAL_StatusTypeDef status;
  uint8_t addr = a;
  uint8_t val;
  _xmit_spi( 0xff );

  DelayClk3( SYNTH_DELAY );
  mixer_select();
  DelayClk3( SYNTH_DELAY );

  addr |= 0x80;  //read operation

  _xmit_spi( addr );
  val = _xmit_spi( 0xff );

  DelayClk3( SYNTH_DELAY );
  mixer_deselect();
  DelayClk3( SYNTH_DELAY );
  return val;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void mixer_write_reg( uint8_t a, uint8_t val )
{

  _xmit_spi( 0xff );

  DelayClk3( SYNTH_DELAY );
  mixer_select();
  DelayClk3( SYNTH_DELAY );

  uint8_t addr = a;

  addr &= 0x7f;  //write operation

  _xmit_spi( addr );
  _xmit_spi( val );

  DelayClk3( SYNTH_DELAY );
  mixer_deselect();
  DelayClk3( SYNTH_DELAY );
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
uint8_t _xmit_spi( uint8_t out_reg )
{

  uint8_t in_reg;
  uint8_t tx = out_reg;

  HAL_StatusTypeDef status;
  //status = HAL_SPI_TransmitReceive_IT(&hspi3, &tx, &in_reg, 1);
  status = HAL_SPI_TransmitReceive( &hspi3, &tx, &in_reg, 1, 5 );
  return in_reg;
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void init_synth1( void )
{

  uint32_t prim;

  prim = __get_PRIMASK();
  __disable_irq();

reset:
  leds_on();
  delay_ms_ni( 150 );
  synth_write_reg( 0x02, 0x07 ); //power-on reset,  reset bit auto-clears
  delay_ms_ni( 150 );           //wait for reset

  if( synth_read_reg( 0x0e ) != 0x23 ) goto reset; //version LTC6948-3.  Keep resetting until this is correct

  rf_mute();
  leds_off();


  synth_write_reg( 0x03, 0x7e ); //TURN ON ALC functions.  working much better with this

  synth_write_reg( 0x05, 0x11 );  //default seed value


  synth_write_reg( 0x0d, 0xc0 );  //allow the loop to lock,  make cpmid and cprst = 0


  //synth_write_reg( 0x04, 0xbe); //LDOV = 2   50mhz pfd
  synth_write_reg( 0x04, 0xbc ); //LDOV = 0    10mhz pfd

  synth_write_reg( 0x06, 0x10 ); //ref divider = 2
  synth_write_reg( 0x07, 127 ); //N value

  //synth_write_reg( 0x06, 0x41); //ref divider = 8
  //synth_write_reg( 0x07, 0x50); //N value


  synth_write_reg( 0x08, 0x1f ); //MSB NUM
  synth_write_reg( 0x09, 0x7f ); //write default value to force cal

  synth_write_reg( 0x0b, 0xfe ); //output divider = 6 , FLT = 3, BST=1

  //synth_write_reg( 0x0c, 0x0d);
  synth_write_reg( 0x0c, 0x48 );  //CP=0


  wait_for_lock();
  rf_unmute();

  if( !prim ) {
    __enable_irq();
  }
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void init_synth2( void )
{
  uint32_t prim;

  prim = __get_PRIMASK();
  __disable_irq();

reset:

  leds_on();
  delay_ms_ni( 150 );
  synth2_write_reg( 0x02, 0x07 ); //power-on reset.  reset bit auto-clears
  delay_ms_ni( 150 );

  if( synth2_read_reg( 0x0e ) != 0x21 ) goto reset; //version LTC6948-3.  Keep resetting until version is correct

  rf_mute();
  leds_off();


  synth2_write_reg( 0x03, 0x7e ); //TURN ON ALC functions.  working much better with this

  //synth2_write_reg( 0x03, 0x3d); //integer N mode for 2LO

  synth2_write_reg( 0x05, 0x11 );  //default seed value

  synth2_write_reg( 0x0d, 0xc0 );  //allow the loop to lock

  //synth2_write_reg( 0x04, 0xbe); //LDOV = 2   50mhz pfd
  synth2_write_reg( 0x04, 0xbc ); //LDOV = 0    10mhz pfd

  synth2_write_reg( 0x06, 0x10 ); //ref divider = 2
  synth2_write_reg( 0x07, 127 ); //N value

  //synth2_write_reg( 0x06, 0x41); //ref divider = 8
  //synth2_write_reg( 0x07, 0x50); //N value


  synth2_write_reg( 0x08, 0x1f ); //MSB NUM
  synth2_write_reg( 0x09, 0x7f ); //write default value to force cal

  synth2_write_reg( 0x0b, 0xfe ); //output divider = 6 , FLT = 3, BST=1

  //synth2_write_reg( 0x0c, 0x0d);
  synth2_write_reg( 0x0c, 0x48 );  //CP=0


  set_freq_mhz2( config->if_frequency );

  wait_for_lock2();
  rf_unmute();

  if( !prim ) {
    __enable_irq();
  }
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void init_mixer( void )
{
  uint32_t prim;

  prim = __get_PRIMASK();
  __disable_irq();

reset:

  delay_ms_ni( 150 );
  mixer_write_reg( 0x16, 0xf8 ); //power-on reset, bit auto-clears
  delay_ms_ni( 150 );

  if( mixer_read_reg( 0x15 ) != 0x6a )  goto reset; //keep resetting until default of 0x6a is read

  //mixer_write_reg( 0x12, 0xf1); //cf1=17,  bias current set to zero is better than default
  //mixer_write_reg( 0x13, 0xda); //band=1, cf2=26, LF1=2
  mixer_write_reg( 0x12, 0xf0 );
  mixer_write_reg( 0x13, 0xf0 );

  mixer_write_reg( 0x0e, 1 ); //I dc offset, leave at 1 for best sensitivity
  mixer_write_reg( 0x0f, 1 ); //Q dc offset, leave at 1 for best sensitivity
  mixer_write_reg( 0x11, 130 ); //BB gain err

  mixer_write_reg( 0x10, 0x04 ); //atten

  //mixer_write_reg( 0x15, 0x6a); //gain
//  mixer_write_reg( 0x03, 128); //im3ix
// mixer_write_reg( 0x02, 0); //im3iy
//
// mixer_write_reg( 0x01, 128); //im3qx
  //mixer_write_reg( 0x00, 0); //im3qy

  if( !prim ) {
    __enable_irq();
  }

}
