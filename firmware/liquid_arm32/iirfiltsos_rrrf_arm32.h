

//
//t.elliott converted from original liquid-dsp code to something that would be usable on the STM32H7 series
//
/*
 * Copyright (c) 2007 - 2015 Joseph Gaeddert
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef __IIRFILTSOS_RRRF_H__
#define __IIRFILTSOS_RRRF_H__

#include "symsync_config.h"


struct iirfiltsos_rrrf_s {
  float b[3];    // feed-forward coefficients
  float a[3];    // feed-back coefficients

  // internal buffering
  float x[3];    // Direct form I  buffer (input)
  float y[3];    // Direct form I  buffer (output)
  float v[3];    // Direct form II buffer

};

float iirfiltsos_rrrf_groupdelay( struct iirfiltsos_rrrf_s *_q, float _fc );
void iirfiltsos_rrrf_execute_df1( struct iirfiltsos_rrrf_s *_q, float _x, float *_y );
void iirfiltsos_rrrf_execute_df2( struct iirfiltsos_rrrf_s *_q, float _x, float *_y );
void iirfiltsos_rrrf_execute( struct iirfiltsos_rrrf_s *_q, float _x, float *_y );
void iirfiltsos_rrrf_reset( struct iirfiltsos_rrrf_s *_q );
void iirfiltsos_rrrf_set_coefficients( struct iirfiltsos_rrrf_s *_q, float *_b, float *_a );
struct iirfiltsos_rrrf_s *iirfiltsos_rrrf_create( struct iirfiltsos_rrrf_s *q, float *_b, float *_a );

#endif
