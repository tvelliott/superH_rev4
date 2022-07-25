

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



#ifndef __CHAN_FILTER_H__
#define __CHAN_FILTER_H__

#define ARM_MATH_CM4
#include <stdint.h>
#include <float.h>
#include <complex.h>
#include "arm_math.h"
#include "arm_const_structs.h"  //DSP includes

#define NSAMPLES 256

void do_filter_decimate_16( float complex *iqdata );
void do_filter_decimate_8( float complex *iqdata );
void do_filter_decimate_4( float complex *iqdata );
void do_filter_decimate_2( float complex *iqdata );
void cf_fft_shift( float32_t *data, int len );
extern int8_t init_fir_dec_16;
extern int8_t init_fir_dec_8;
extern int8_t init_fir_dec_4;
extern int8_t init_fir_dec_2;
extern const float32_t channel_filter_quarterbw[];
extern float32_t qout[256];
extern float32_t iout[256];

#endif
