

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

// firfilt object structure
#ifndef __FIRFILT_CF_H__
#define __FIRFILT_CF_H__

#include <float.h>
#include <complex.h>
#include "dotprod_cf_arm32.h"

struct firfilt_cf_s {
    float complex *h;             // filter coefficients array [size; h_len x 1]
    unsigned int h_len; // filter length

    // use array as internal buffer (faster)
    float complex *w;                 // internal buffer object
    unsigned int w_len;     // window length
    unsigned int w_mask;    // window index mask
    unsigned int w_index;   // window read index
    struct dotprod_cf_s *dp;           // dot product object
    float scale;               // output scaling factor
};

void firfilt_cf_freqresponse(struct firfilt_cf_s *_q,float _fc,float complex *_H);
unsigned int firfilt_cf_get_length(struct firfilt_cf_s *_q);
void firfilt_cf_execute_block(struct firfilt_cf_s *_q, float complex *_x, unsigned int _n, float complex *_y);
void firfilt_cf_execute(struct firfilt_cf_s *_q, float complex *_y);
void firfilt_cf_push(struct firfilt_cf_s *_q, float complex _x);
void firfilt_cf_get_scale(struct firfilt_cf_s *_q,float *_scale);
void firfilt_cf_set_scale(struct firfilt_cf_s *_q,float _scale);
struct firfilt_cf_s *firfilt_cf_create_rect(struct firfilt_cf_s *q,unsigned int _n);
struct firfilt_cf_s *firfilt_cf_create_rnyquist(struct firfilt_cf_s *q, int _type, unsigned int _k, unsigned int _m, float _beta, float _mu);
struct firfilt_cf_s *firfilt_cf_create_kaiser(struct firfilt_cf_s *q, unsigned int _n, float _fc, float _As, float _mu);
void firfilt_cf_reset(struct firfilt_cf_s *_q);
struct firfilt_cf_s *firfilt_cf_create(struct firfilt_cf_s *q,float complex *_h,unsigned int _n);
#endif
