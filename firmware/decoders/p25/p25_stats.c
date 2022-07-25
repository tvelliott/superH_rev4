
//MIT License
//
//Copyright (c) 2019 tvelliott
//
//Permission is hereby granted, free of charge, to any person obtaining a copy
//of this software and associated documentation files (the "Software"), to deal
//in the Software without restriction, including without limitation the rights
//to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//copies of the Software, and to permit persons to whom the Software is
//furnished to do so, subject to the following conditions:
//
//The above copyright notice and this permission notice shall be included in all
//copies or substantial portions of the Software.
//
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//SOFTWARE.




#include <stdio.h>
#include <math.h>
#include <float.h>
#include "p25_stats.h"

static float _buffer[2048];
static int _len;
static float min;
static float max;
static float mean;
static float sum;

///////////////////////////////////////////////////
///////////////////////////////////////////////////
void p25_stats_init( float *fptr, int len )
{
  int i;
  for( i = 0; i < len; i++ ) {
    _buffer[i] = fptr[i];
  }
  _len = len;
}


///////////////////////////////////////////////////
///////////////////////////////////////////////////
float p25_stats_max()
{
  max = FLT_MIN;
  int i;
  for( i = 0; i < _len; i++ ) {
    if( _buffer[i] > max ) max = _buffer[i];
  }
  return max;
}
///////////////////////////////////////////////////
///////////////////////////////////////////////////
float p25_stats_min()
{
  min = FLT_MAX;
  int i;
  for( i = 0; i < _len; i++ ) {
    if( _buffer[i] < min ) min = _buffer[i];
  }
  return min;
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////
float p25_stats_mean()
{
  sum = 0.0f;
  int i;
  for( i = 0; i < _len; i++ ) {
    sum += _buffer[i];
  }

  return sum / _len;
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////
float p25_stats_rms()
{
  sum = 0.0f;
  int i;
  for( i = 0; i < _len; i++ ) {
    sum += _buffer[i] * _buffer[i];
  }

  return ( 1.0 / Q_rsqrt( sum ) ) / _len;
}
///////////////////////////////////////////////////
///////////////////////////////////////////////////
float p25_stats_mean_deviation()
{
  mean = p25_stats_mean();
  float temp = 0.0f;
  float a;
  int i;

  for( i = 0; i < _len; i++ ) {
    a = _buffer[i];
    temp += fabs( ( mean - a ) );
  }

  return temp / _len;
}

///////////////////////////////////////////////////
///////////////////////////////////////////////////
float p25_stats_variance()
{
  mean = p25_stats_mean();
  float temp = 0.0f;
  float a;
  int i;

  for( i = 0; i < _len; i++ ) {
    a = _buffer[i];
    temp += ( mean - a ) * ( mean - a );
  }

  return temp / _len;
}
///////////////////////////////////////////////////
///////////////////////////////////////////////////
float p25_stats_stddev()
{
  float v = p25_stats_variance();
  return 1.0 / Q_rsqrt( v );
}

///////////////////////////////////////////////////////////////////
//  The following code is the fast inverse square root
//  implementation from Quake III Arena, stripped of C
//  preprocessor directives, but including the exact
//  original comment text
//
//  https://en.wikipedia.org/wiki/Fast_inverse_square_root
///////////////////////////////////////////////////////////////////
float Q_rsqrt( float number )
{
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = number * 0.5F;
  y  = number;
  i  = * ( long * ) &y;                       // evil floating point bit level hacking
  i  = 0x5f3759df - ( i >> 1 );               // what the fuck?
  y  = * ( float * ) &i;
  y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
  //  y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

  return y;
}
