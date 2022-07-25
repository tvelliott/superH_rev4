

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

#ifndef __EQLMS_RF_H__
#define __EQLMS_RF_H__

struct eqlms_rf_s {
    unsigned int h_len;     // filter length
    float        mu;        // LMS step size

    // internal matrices
    float *h0;        // initial coefficients
    float *w0;        // weights [px1]
    float *w1;        // weights [px1]

    unsigned int count;     // input sample count
    int          buf_full;  // input buffer full flag
    struct window_rf_s *buffer;    // input buffer
    struct wdelay_rf_s *x2;        // buffer of |x|^2 values
    float        x2_sum;    // sum{ |x|^2 }
};

void eqlms_rf_train(struct eqlms_rf_s *_q,float *_w,float *_x,float *_d,unsigned int _n);
void eqlms_rf_get_weights(struct eqlms_rf_s *_q,float *_w);
void eqlms_rf_step(struct eqlms_rf_s *_q,float _d,float _d_hat);
void eqlms_rf_step_blind(struct eqlms_rf_s *_q,float _d_hat);
void eqlms_rf_execute_block(struct eqlms_rf_s *_q,unsigned int _k,float *_x,unsigned int _n,float *_y);
void eqlms_rf_execute(struct eqlms_rf_s *_q,float *_y);
void eqlms_rf_push_block(struct eqlms_rf_s *_q,float *_x,unsigned int _n);
void eqlms_rf_update_sumsq(struct eqlms_rf_s *_q,float _x);
void eqlms_rf_push(struct eqlms_rf_s *_q,float _x);
void eqlms_rf_set_bw(struct eqlms_rf_s *_q,float _mu);
float eqlms_rf_get_bw(struct eqlms_rf_s *_q);
struct eqlms_rf_s *eqlms_rf_create_lowpass(unsigned int _h_len,float _fc);
struct eqlms_rf_s *eqlms_rf_create_rnyquist(int _type,unsigned int _k,unsigned int _m,float _beta,float _dt);
void eqlms_rf_reset(struct eqlms_rf_s *_q);
struct eqlms_rf_s *eqlms_rf_create(float *_h,unsigned int _h_len);

#endif
