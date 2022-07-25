

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
#ifndef __DOTPROD_RF_H__
#define __DOTPROD_RF_H__

// portable structured dot product object
struct dotprod_rf_s {
  //float h[MAX_DOTPROD_HLEN];              // coefficients array
  float *h;              // coefficients array
  unsigned int n;     // length
};
void dotprod_rf_execute( struct dotprod_rf_s *_q, float *_x, float *_y );
struct dotprod_rf_s *dotprod_rf_create( struct dotprod_rf_s *q, float *_h, unsigned int _n );
//void dotprod_rf_run4( float *_h, float *_x, unsigned int _n, float *_y );
//void dotprod_rf_run( float *_h, float *_x, unsigned int _n, float *_y );
#endif
