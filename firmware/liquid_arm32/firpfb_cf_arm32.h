

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

#ifndef __FIRPFB_CF_H__
#define __FIRPFB_CF_H__

#include "resamp_config.h"

#include <float.h>
#include <complex.h>

#include "window_cf_arm32.h"
#include "dotprod_cf_arm32.h"

struct firpfb_cf_s {
  float complex *h;                      // filter coefficients array
  unsigned int h_len;         // total number of filter coefficients
  unsigned int h_sub_len;     // sub-sampled filter length
  unsigned int num_filters;   // number of filters

  struct window_cf_s *w;                 // window buffer
  struct dotprod_cf_s *dp[RESAMP_CF_NFILTERS];              // array of vector dot product objects
  float scale;                   // output scaling factor
};
void firpfb_cf_execute_block( struct firpfb_cf_s *_q, unsigned int _i, float complex *_x, unsigned int _n, float complex *_y );
void firpfb_cf_execute( struct firpfb_cf_s *_q, unsigned int _i, float complex *_y );
void firpfb_cf_push( struct firpfb_cf_s *_q, float complex _x );
void firpfb_cf_set_scale( struct firpfb_cf_s *_q, float _scale );
struct firpfb_cf_s *firpfb_cf_create_drnyquist( struct firpfb_cf_s *q, int _type, unsigned int _M, unsigned int _k, unsigned int _m, float _beta, struct window_cf_s *_window, struct dotprod_cf_s *_dp );
struct firpfb_cf_s *firpfb_cf_create_rnyquist( struct firpfb_cf_s *q, int _type, unsigned int _M, unsigned int _k, unsigned int _m, float _beta, struct window_cf_s *_window, struct dotprod_cf_s *_dp );
struct firpfb_cf_s *firpfb_cf_create_kaiser( struct firpfb_cf_s *q, unsigned int _M, unsigned int _m, float _fc, float _As, struct window_cf_s *_window, struct dotprod_cf_s *_dp );
void firpfb_cf_reset( struct firpfb_cf_s *_q );
struct firpfb_cf_s *firpfb_cf_create( struct firpfb_cf_s *q, unsigned int _M, float complex *_h, unsigned int _h_len, struct window_cf_s *_window, struct dotprod_cf_s *_dp );
#endif
