

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






#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "ir_remote.h"
#include "main.h"
#include "globals.h"

enum {
  IR_STATE_NA,
  IR_STATE_IDLE,
  IR_STATE_JUNK,
  IR_STATE_SOF,
  IR_STATE_RX,
};

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

static volatile int ir_remote_state = IR_STATE_IDLE;
volatile int ir_nbits;
volatile uint32_t ir_code;
static volatile int ir_code_ready;
volatile int ir_timeout;
volatile int ir_lockout;
static volatile int val_thresh = 600;

volatile int tone_timeout;
volatile int ir_found_start;
volatile uint8_t ir_histo[256];
volatile int timer3_on;

#define IR_TIMEOUT 40

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void ir_remote_ms_tick( void )
{
  uint32_t prim;


  //if( tone_timeout == 0 && ir_lockout == 0 && ir_timeout == 0 ) return;

  if( tone_timeout > 0 ) {
    tone_timeout--;
    if( tone_timeout == 0 ) do_audio_tone = 0;
  }


  //TODO: FIX

  #if 1
  return;
  #endif


  if( ir_lockout > 0 ) {
    ir_lockout--;
  }

  if( ir_lockout == 0 && ir_timeout > 0 ) {
    ir_timeout--;
    if( ir_timeout == 0 ) {

      if( ir_nbits >= 42 ) ir_code_ready = 1;

      ir_nbits = 0;

      if( timer3_on ) {
        HAL_TIM_Base_Stop( &htim3 );
        timer3_on = 0;
      }
    }
  }

  if( ir_code_ready ) {

    prim = __get_PRIMASK();
    __disable_irq();

    //printf("\r\nnbits: %d", ir_nbits);
    ir_remote_state = IR_STATE_IDLE;

    ir_timeout = 0;
    ir_code_ready = 0;


    ir_process_command( ir_code );


    if( !prim ) {
      __enable_irq();
    }
  }

}

///////////////////////////////////////////////////////////////////////////////
//  must be quick here... interrupts are disabled
///////////////////////////////////////////////////////////////////////////////
void ir_remote_rx( int val )
{
  //TODO: FIX

  #if 1
  return;
  #endif

  if( val < 20 ) return;
  if( ir_lockout ) return;

//printf("\r\nval: %d", val);

  if( val > 1000 ) {
    ir_nbits = 0;
    ir_timeout = 0;
    ir_lockout = 0;
    if( timer3_on ) {
      HAL_TIM_Base_Stop( &htim3 );
      timer3_on = 0;
    }
    return;
  }

  if( HAL_GPIO_ReadPin( IR_INPUT_GPIO_Port, IR_INPUT_Pin )  && ir_nbits == 0 && val > 600 ) {
    ir_nbits++;
    //ir_timeout = IR_TIMEOUT;
    return;
  }

  if( HAL_GPIO_ReadPin( IR_INPUT_GPIO_Port, IR_INPUT_Pin )  && ir_nbits == 1 ) {
    //printf("\r\n ");
    ir_nbits++;
    return;
  }

  if( HAL_GPIO_ReadPin( IR_INPUT_GPIO_Port, IR_INPUT_Pin )  && ir_nbits == 2 ) {
    ir_nbits++;
    return;
  }

  if( HAL_GPIO_ReadPin( IR_INPUT_GPIO_Port, IR_INPUT_Pin )  && ir_nbits == 3 ) {

    memset( ir_histo, 0x00, 0xff );
    ir_code = 0;
    ir_timeout = IR_TIMEOUT;

    if( !timer3_on ) {
      __HAL_TIM_SetCounter( &htim3, 65535 );
      HAL_TIM_Base_Start_IT( &htim3 );
      timer3_on = 1;
    }
  }

  if( HAL_GPIO_ReadPin( IR_INPUT_GPIO_Port, IR_INPUT_Pin )  && ir_nbits > 3 ) {
    ir_histo[( val & 0xff ) ]++;
  }

}

///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
void ir_process_command( uint32_t val )
{

  int i;

  //TODO: FIX

  #if 1
  return;
  #endif


#if 0
  for( i = 0; i < 256; i++ ) {
    printf( "\r\n%d,%d", i, ir_histo[i] );
  }
#endif

  val &= 0x1fff;
  if( val == 0x0000 ) return;

  printf( "\r\nIR code: 0x%04x", val );

  ir_lockout = IR_TIMEOUT;

  switch( val ) {

  case  0x1b3f  : //menu
    config->show_rssi ^= 0x01;
    break;

  case  0x1bdf  : //right-arrow, nex,  next preset
    parse_command( "n" );
    printf( "\r\nnext" );
    do_audio_tone = 1; //send out short sine-wave audio tone for user feedback
    tone_timeout = 24;
    break;

  case  0x1bff  : //left-arrow, prev, previous preset
    parse_command( "p" );
    printf( "\r\nprev" );
    do_audio_tone = 1; //send out short sine-wave audio tone for user feedback
    tone_timeout = 24;
    break;

  case  0x1dff  : //vol up
    if( config->audio_volume_f <= 1.0 - 0.05 ) config->audio_volume_f = config->audio_volume_f + 0.05f;
    config->audio_on = 1;
    printf( "\r\nvolup" );
    break;

  case  0x1ddf  : //vol down
    if( config->audio_volume_f >= 0.05 ) config->audio_volume_f = config->audio_volume_f - 0.05f;
    config->audio_on = 1;
    printf( "\r\nvoldown" );
    break;

  case  0x149f  :
    parse_command( "preset r 0" );
    printf( "\r\ndefault preset" );
    break;

  case  0x1e1f  :
    parse_command( "preset r 16" );
    printf( "\r\np25 preset" );
    break;

  case  0x1e3f  : //audio mute toggle
    config->audio_on ^= 0x01;
    printf( "\r\naudio on/off" );
    break;
  }
}
