
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




#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "fsk_decode_12.h"

#define SAMPLE_RATE_F 48000.0f
#define BAUD_RATE_F 1200.0f


static uint32_t sreg;
static uint32_t decode_sreg; 
static int decode_nbits; 
static int is_synced;

//360 degrees of phase_inc is 1 bit period
static float phase_inc = (360.0f/SAMPLE_RATE_F)*BAUD_RATE_F; 
static float current_phase;
static float bit_period_mid;

static int sample_avg[4];
static int sample_avg_idx;

static float loop_filter_bw = 0.3f;

////////////////////////////////////////////////////
////////////////////////////////////////////////////
uint32_t fsk_12_get_decode_bits() {
  decode_nbits=0;
  return decode_sreg;
}

////////////////////////////////////////////////////
////////////////////////////////////////////////////
void fsk12_set_loop_filter_bw(float f) {
  loop_filter_bw = f;
}
////////////////////////////////////////////////////
////////////////////////////////////////////////////
void fsk12_set_srate(float srate, float brate) {
  phase_inc = (360.0f/srate)*brate; 
}

////////////////////////////////////////////////////
// returns number of bits decoded 
////////////////////////////////////////////////////
int fsk_12_decode_nrz(int16_t *buffer, int len) {
int i;

  //run through the audio buffer
  for(i=0; i<len; i++) {


    sreg <<= 1; //roll the previous chip 
    //pos or neg?
    if( buffer[i]>0 ) {
      sreg |= 0x01; //1 or 0?  did we match mark or space more closely for this buffer? 
    }

      //transition?  keep transistions at 180 degrees
    if( (sreg ^ sreg>>1) & 0x01 ) {
      bit_period_mid = (current_phase  - 180.0 );

      current_phase -= bit_period_mid*loop_filter_bw;
    }

    //advance phase as we step through the buffer
    current_phase += phase_inc;

    sample_avg[ sample_avg_idx++ ] = buffer[i];
    sample_avg_idx &= 0x03;

    //have we completed a full cycle of the symbol period yet?, then time for a bit decision
    if( current_phase >= 360.0f) {
        current_phase -= 360.0f; //keep any overflow 


        decode_sreg <<= 1; //roll previous bit decision

        int savg = sample_avg[0]+sample_avg[1]+sample_avg[2]+sample_avg[3]; 
        savg /= 4;

        if(savg > 0) sreg=1;
          else sreg =0;

        //hard decision
        decode_sreg |= (sreg&1)^1;   //was the last sample positive or negative  (sampled in middle of bit period)
        //decode_sreg |= (sreg&1);   //was the last sample positive or negative  (sampled in middle of bit period)

        decode_nbits++; //new bit is ready

    }
  }

  return decode_nbits;
}
