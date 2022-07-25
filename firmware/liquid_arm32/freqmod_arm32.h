

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
// freqmod
#ifndef __FREQMOD_H__
#define __FREQMOD_H__

struct freqmod_s {
    float kf;   // modulation factor for FM
    float ref;  // phase reference: kf*2^16

    // look-up table
    unsigned int sincos_table_len;      // table length: 10 bits
    uint16_t     sincos_table_phase;    // accumulated phase: 16 bits
    float complex *sincos_table;          // sin|cos look-up table: 2^10 entries
};

void freqmod_modulate_block(struct freqmod_s *_q,float *_m,unsigned int _n,float complex *_s);
void freqmod_modulate(struct freqmod_s *_q,float _m,float complex *_s);
void freqmod_reset(struct freqmod_s *_q);
struct freqmod_s *freqmod_create(float _kf);

#endif
