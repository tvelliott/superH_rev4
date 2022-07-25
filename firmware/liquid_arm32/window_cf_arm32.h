

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
#ifndef __WINDOW_CF_H__
#define __WINDOW_CF_H__

#include <complex.h> 
#include "symsync_config.h"


struct window_cf_s {
  //float complex v[SYMSYNC_WINDOW_V_SIZE];                       // allocated array pointer
  float complex *v;                       // allocated array pointer
  unsigned int len;           // length of window
  unsigned int m;             // floor(log2(len)) + 1
  unsigned int n;             // 2^m
  unsigned int mask;          // n-1
  unsigned int num_allocated; // number of elements allocated
  // in memory
  unsigned int read_index;
};
void window_cf_write( struct window_cf_s *_q, float complex *_v, unsigned int _n );
void window_cf_push( struct window_cf_s *_q, float complex _v );
void window_cf_index( struct window_cf_s *_q, unsigned int _i, float complex *_v );
void window_cf_read( struct window_cf_s *_q, float complex **_v );
void window_cf_reset( struct window_cf_s *_q );
struct window_cf_s *window_cf_create( struct window_cf_s *q, unsigned int _n );

#endif
