

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
#ifndef __RESAMP2_CF_H__
#define __RESAMP2_CF_H__

#include <float.h>
#include <complex.h>
#include <dotprod_cf_arm32.h>
#include <window_cf_arm32.h>

struct resamp2_s {
    float complex * h;                 // filter prototype
    unsigned int m;         // primitive filter length
    unsigned int h_len;     // actual filter length: h_len = 4*m+1
    float f0;               // center frequency [-1.0 <= f0 <= 1.0]
    float As;               // stop-band attenuation [dB]

    // filter component
    float complex * h1;                // filter branch coefficients
    struct dotprod_cf_s *dp;           // inner dot product object
    unsigned int h1_len;    // filter length (2*m)

    // input buffers
    struct window_cf_s *w0;            // input buffer (even samples)
    struct window_cf_s *w1;            // input buffer (odd samples)

    // halfband filter operation
    unsigned int toggle;
};

void resamp2_interp_execute(struct resamp2_s *_q,float complex _x,float complex *_y);
void resamp2_decim_execute(struct resamp2_s *_q,float complex *_x,float complex *_y);
void resamp2_synthesizer_execute(struct resamp2_s *_q,float complex *_x,float complex *_y);
void resamp2_analyzer_execute(struct resamp2_s *_q,float complex *_x,float complex *_y);
void resamp2_filter_execute(struct resamp2_s *_q,float complex _x,float complex *_y0,float complex *_y1);
unsigned int resamp2_get_delay(struct resamp2_s *_q);
void resamp2_reset(struct resamp2_s *_q);
struct resamp2_s *resamp2_create(struct resamp2_s *q, unsigned int _m,float _f0,float _As);
#endif
