

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

#ifndef __WINDOW_FUNCTIONS_LIQUID_H__
#define __WINDOW_FUNCTIONS_LIQUID_H__
float liquid_rcostaper_windowf(unsigned int _n,unsigned int _t,unsigned int _N);
float triangular(unsigned int _n,unsigned int _N,unsigned int _L);
float flattop(unsigned int _n,unsigned int _N);
float blackmanharris7(unsigned int _n,unsigned int _N);
float blackmanharris(unsigned int _n,unsigned int _N);
float hann(unsigned int _n,unsigned int _N);
float hamming(unsigned int _n,unsigned int _N);
void liquid_kbd_window(unsigned int _n,float _beta,float *_w);
float kaiser(unsigned int _n,unsigned int _N,float _beta,float _mu);
float liquid_kbd(unsigned int _n,unsigned int _N,float _beta);
#endif
