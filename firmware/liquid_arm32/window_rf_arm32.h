

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
#ifndef __WINDOW_RF_H__
#define __WINDOW_RF_H__

#include "symsync_config.h"


struct window_rf_s {
  //float v[SYMSYNC_WINDOW_V_SIZE];                       // allocated array pointer
  float *v;                       // allocated array pointer
  unsigned int len;           // length of window
  unsigned int m;             // floor(log2(len)) + 1
  unsigned int n;             // 2^m
  unsigned int mask;          // n-1
  unsigned int num_allocated; // number of elements allocated
  // in memory
  unsigned int read_index;
};
void window_rf_write( struct window_rf_s *_q, float *_v, unsigned int _n );
void window_rf_push( struct window_rf_s *_q, float _v );
void window_rf_index( struct window_rf_s *_q, unsigned int _i, float *_v );
void window_rf_read( struct window_rf_s *_q, float **_v );
void window_rf_reset( struct window_rf_s *_q );
struct window_rf_s *window_rf_create( struct window_rf_s *q, unsigned int _n );

#endif
