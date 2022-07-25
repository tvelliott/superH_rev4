

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
#ifndef __FIRHILB_H__
#define __FIRHILB_H__

#include <complex.h>

#include "window_rf_arm32.h"
#include "dotprod_rf_arm32.h"

struct firhilb_s {
    float * h;                  // filter coefficients
    float complex * hc;         // filter coefficients (complex)
    unsigned int h_len;     // length of filter
    float As;               // filter stop-band attenuation [dB]

    unsigned int m;         // filter semi-length, h_len = 4*m+1

    // quadrature filter component
    float * hq;                 // quadrature filter coefficients
    unsigned int hq_len;    // quadrature filter length (2*m)

    // input buffers
    struct window_rf_s *w0;            // input buffer (even samples)
    struct window_rf_s *w1;            // input buffer (odd samples)

    // vector dot product
    struct dotprod_rf_s *dpq;

    // regular real-to-complex/complex-to-real operation
    unsigned int toggle;
};

void firhilb_interp_execute_block(struct firhilb_s *_q,float complex *_x,unsigned int _n,float *_y);
void firhilb_interp_execute(struct firhilb_s *_q,float complex _x,float *_y);
void firhilb_decim_execute_block(struct firhilb_s *_q,float *_x,unsigned int _n,float complex *_y);
void firhilb_decim_execute(struct firhilb_s *_q,float *_x,float complex *_y);
void firhilb_c2r_execute(struct firhilb_s *_q,float complex _x,float *_y);
void firhilb_r2c_execute(struct firhilb_s *_q,float _x,float complex *_y);
void firhilb_reset(struct firhilb_s *_q);
struct firhilb_s *firhilb_create(unsigned int _m,float _As);

#endif
