

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




#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "std_io.h"

uint8_t net_buffer[MAX_NET_BUFFER];
uint8_t printf_buf[MAX_PRINTF_BUFFER];
volatile uint32_t netbuf_s=0; 
volatile uint32_t netbuf_e=0; 

extern int telnet_session_count;
static int verbose_level=0;


///////////////////////////////////////////////////////////////////////////////
//  printf implementation doesn't work with single characters.  Could add this
//  call to handle it, but better just to leave it separate since a single
//  character doesn't need formatted output and this is much more efficient.
///////////////////////////////////////////////////////////////////////////////
void putchar_stdio(uint8_t c) {
  net_buffer[netbuf_e++] = c; 
  netbuf_e &= (uint32_t) (MAX_NET_BUFFER-1);
}

///////////////////////////////////////////////////////////////////////////////
/*
  Note:  net_buffer can hold up to 16kB with default configuration
         If you need to output more than that before returning from a function,
         you can flush the buffer with code similar to the following.

    if( linesout % 16 == 0 ) {  //every 16 records, wait for console buffer to flush
                                //as telnet / lwip has some time to get it out...
      while( netbuf_s!=netbuf_e ) {  
        main_tick();
      }
    }
*/
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
int printf( const char *format, ... )
{

  uint16_t ret = 0;
  va_list args;

  if( is_telnet_closed() ) {
    return 0;
  }

  memset(printf_buf,0x00,sizeof(printf_buf));

  va_start( args, format );

  ret = vsnprintf( printf_buf, sizeof( printf_buf ) - 1, format, args );
  printf_buf[MAX_PRINTF_BUFFER - 1] = 0;

  uint8_t *ptr = printf_buf;

  if(ret>0) {
    while(*ptr!=0x00) {
      net_buffer[netbuf_e++] = *ptr++;
      netbuf_e &= (uint32_t) (MAX_NET_BUFFER-1);
    }
  }

  va_end( args );


  return ret; //length sent to device
}

///////////////////////////////////////////////////////////////////////////////
//  This function is used by the Flex pager decoder
//  Could replace calls to this with printf and let the user adjust 
//  config->logging level like the rest of the code.
///////////////////////////////////////////////////////////////////////////////
int verbprintf(int verb_level, const char *format, ...)
{
  uint16_t ret = 0;
  va_list args;

  if (verb_level > verbose_level) return;

  if( is_telnet_closed() ) {
    return 0;
  }

  memset(printf_buf,0x00,sizeof(printf_buf));

  va_start( args, format );

  ret = vsnprintf( printf_buf, sizeof( printf_buf ) - 1, format, args );
  printf_buf[MAX_PRINTF_BUFFER - 1] = 0;

  uint8_t *ptr = printf_buf;

  if(ret>0) {
    while(*ptr!=0x00) {
      net_buffer[netbuf_e++] = *ptr++;
      netbuf_e &= (uint32_t) (MAX_NET_BUFFER-1);
    }
  }

  va_end( args );


  return ret; //length sent to device
}
