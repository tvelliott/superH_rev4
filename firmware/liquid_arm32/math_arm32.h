

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
#ifndef __MATH_LIQUID_H__
#define __MATH_LIQUID_H__
float liquid_factorialf(unsigned int _n);
float liquid_uppergammaf(float _z,float _alpha);
float liquid_lowergammaf(float _z,float _alpha);
float liquid_lnuppergammaf(float _z,float _alpha);
float liquid_lnlowergammaf(float _z,float _alpha);
float liquid_gammaf(float _z);
float liquid_besselj0f(float _z);
float liquid_besseljf(float _nu,float _z);
float liquid_besseli0f(float _z);
float liquid_lngammaf(float _z);
float liquid_lnbesselif(float _nu,float _z);
float liquid_nchoosek(unsigned int _n,unsigned int _k);
unsigned int liquid_nextpow2(unsigned int _x);
float sincf(float _x);
float liquid_besselif(float _nu,float _z);
float liquid_MarcumQ1f(float _alpha,float _beta);
float liquid_MarcumQf(int _M,float _alpha,float _beta);
float liquid_Qf(float _z);
#endif
