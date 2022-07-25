

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
#ifndef __DOTPROD_CF_H__
#define __DOTPROD_CF_H__

#include <complex.h>

// portable structured dot product object
struct dotprod_cf_s {
  //float complex h[MAX_DOTPROD_HLEN];              // coefficients array
  float complex *h;              // coefficients array
  unsigned int n;     // length
};
void dotprod_cf_execute( struct dotprod_cf_s *_q, float complex *_x, float complex *_y );
struct dotprod_cf_s *dotprod_cf_create( struct dotprod_cf_s *q, float complex *_h, unsigned int _n );
//void dotprod_cf_run4( float complex *_h, float complex *_x, unsigned int _n, float complex *_y );
//void dotprod_cf_run( float complex *_h, float complex *_x, unsigned int _n, float complex *_y );
#endif
