

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
#ifndef __EQLMS_CF_H__
#define __EQLMS_CF_H__

#include <math.h>
#include <complex.h>
#include <window_cf_arm32.h>
#include <wdelay_cf_arm32.h>

struct eqlms_cf_s {
    unsigned int h_len;     // filter length
    float        mu;        // LMS step size

    // internal matrices
    float complex *h0;        // initial coefficients
    float complex *w0;        // weights [px1]
    float complex *w1;        // weights [px1]

    unsigned int count;     // input sample count
    int          buf_full;  // input buffer full flag
    struct window_cf_s *buffer;    // input buffer
    struct wdelay_cf_s *x2;        // buffer of |x|^2 values
    float  x2_sum;    // sum{ |x|^2 }
};

void eqlms_cf_train(struct eqlms_cf_s *_q,float complex *_w,float complex *_x,float complex *_d,unsigned int _n);
void eqlms_cf_get_weights(struct eqlms_cf_s *_q,float complex *_w);
void eqlms_cf_step(struct eqlms_cf_s *_q,float complex _d,float complex _d_hat);
void eqlms_cf_step_blind(struct eqlms_cf_s *_q,float complex _d_hat);
void eqlms_cf_execute_block(struct eqlms_cf_s *_q,unsigned int _k,float complex *_x,unsigned int _n,float complex *_y);
void eqlms_cf_execute(struct eqlms_cf_s *_q,float complex *_y);
void eqlms_cf_push_block(struct eqlms_cf_s *_q,float complex *_x,unsigned int _n);
void eqlms_cf_update_sumsq(struct eqlms_cf_s *_q,float complex _x);
void eqlms_cf_push(struct eqlms_cf_s *_q,float complex _x);
void eqlms_cf_set_bw(struct eqlms_cf_s *_q,float _mu);
float eqlms_cf_get_bw(struct eqlms_cf_s *_q);
struct eqlms_cf_s *eqlms_cf_create_lowpass(unsigned int _h_len,float _fc);
struct eqlms_cf_s *eqlms_cf_create_rnyquist(int _type,unsigned int _k,unsigned int _m,float _beta,float _dt);
void eqlms_cf_reset(struct eqlms_cf_s *_q);
struct eqlms_cf_s *eqlms_cf_create(float complex *_h,unsigned int _h_len);

#endif
