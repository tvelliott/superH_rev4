

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
#ifndef __AMPMODEM_H__
#define __AMPMODEM_H__

#include "firhilb_arm32.h"
#include "nco_cf_arm32.h"

enum {
    LIQUID_AMPMODEM_DSB=0,  // double side-band
    LIQUID_AMPMODEM_USB,    // single side-band (upper)
    LIQUID_AMPMODEM_LSB     // single side-band (lower)
};

struct ampmodem_s {
    float m;                    // modulation index
    int type;  // modulation type
    int suppressed_carrier;     // suppressed carrier flag
    float fc;                   // carrier frequency

    // demod objects
    struct nco_cf_s *oscillator;

    // suppressed carrier
    // TODO : replace DC bias removal with iir filter object
    float ssb_alpha;    // dc bias removal
    float ssb_q_hat;

    // single side-band
    struct firhilb_s *hilbert;   // hilbert transform

};

void ampmodem_demodulate_block(struct ampmodem_s *_q,float complex *_r,unsigned int _n,float *_m);
void ampmodem_demodulate(struct ampmodem_s *_q,float complex _y,float *_x);
void ampmodem_modulate_block(struct ampmodem_s *_q,float *_m,unsigned int _n,float complex *_s);
void ampmodem_modulate(struct ampmodem_s *_q,float _x,float complex *_y);
void ampmodem_reset(struct ampmodem_s *_q);
struct ampmodem_s * ampmodem_create(float _m,float _fc,int _type,int _suppressed_carrier);
#endif
