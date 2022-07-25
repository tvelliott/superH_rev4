

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





#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <complex.h>

int audio_fd;

////////////////////////////////////////////////////////
//  usage:
//
//  set the superH+ to deliver fm or am demodulated
//  and filtered samples over udp.  use netcat to 
//  capture the udp data and output it on stdout.
//
//  pipe the output into your decoder based on this
//  template code.
//
//  example:
//
//  netcat -l -u -p 8889 | ./my_decoder
//
//  -l listen for udp
//  -u udp
//  -p 8889  udp port number  (default superH+ udp port)
//
////////////////////////////////////////////////////////
int main( char *argv, int argc )
{

  int16_t _I;
  int16_t _Q;

  if( ( audio_fd = open( "/dev/stdin", O_RDONLY ) ) == -1 ) {
    printf("\r\ncouldn't open stdin device\r\n"); //should never happen
    exit( 0 );
  }

  while( 1 ) {
    if( read( audio_fd, (char *) &_I, 2 ) == 2 && read( audio_fd, (char *) &_Q, 2 ) == 2) {
      do_decode_sample_s16(_I,_Q);
    } 
  }
}


////////////////////////////////////////////////////////
//  incoming demodulated / decimated / filtered samples
//  standard superH+ sample rate is 48000 sps,
//  16-bit signed samples
////////////////////////////////////////////////////////
void do_decode_sample_s16(int16_t ii, int16_t qq) {
  printf("\r\n%d,%d", ii,qq);
}
