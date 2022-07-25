
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



#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>

#include "pocsag.h"
#include "globals.h"
#include "fsk_decode_12.h"


//moved to header file
//states
//enum {
//  POCSAG_NOSYNC,
//  POCSAG_ADDRESS,
//  POCSAG_MESSAGE,
//}pocsag_state;

#define STATE_SYNC 1
#define STATE_DATA 2

#define POCSAG_SYNC_WORD 0x7cd215d8
#define POCSAG_IDLE 0x7a89c197
#define EOM 0x17

#define BCH_POLY 03551
#define BCH_N    31
#define BCH_K    21

volatile int pocsag_state = POCSAG_NOSYNC;
static int bitcount;
static int digit_count;
static uint32_t shiftreg;
static int function;
static int frame_count=0;
static int cw_count;
static uint8_t c;
static int bc;
static uint32_t data;
static uint32_t add;
static int i;
static char header_buffer[128];
static int timeout;
static uint32_t nbits;
static uint32_t decode_bits;
int pocsag_debug_on=0;
static int init_pocsag=0;

//////////////////////////////////////////////////////////////////////////
// ripped from multimon
//////////////////////////////////////////////////////////////////////////
//static inline unsigned char even_parity(uint32_t data)
//{
//  unsigned int temp = data ^ (data >> 16);
//
//  temp = temp ^ (temp >> 8);
//  temp = temp ^ (temp >> 4);
//  temp = temp ^ (temp >> 2);
//  temp = temp ^ (temp >> 1);
//  return temp & 1;
//}
//////////////////////////////////////////////////////////////////////////
// ripped from multimon
//////////////////////////////////////////////////////////////////////////
static unsigned int pocsag_syndrome(uint32_t data)
{
  uint32_t shreg = data >> 1;
  uint32_t mask = 1L << (BCH_N-1), coeff = BCH_POLY << (BCH_K-1);
  int n = BCH_K;

  for(; n > 0; mask >>= 1, coeff >>= 1, n--)
    if (shreg & mask)
      shreg ^= coeff;
  if (even_parity(data)) shreg |= (1 << (BCH_N - BCH_K));
  return shreg;
}

//////////////////////////////////////////////////////////////////////////
// single-bit, brute force bit repair
//////////////////////////////////////////////////////////////////////////
uint32_t try_repair(uint32_t u)
{
  uint32_t ret = u;

  int i;
  uint32_t tmp;
  for(i=0; i<32; i++) {
    tmp = u;
    tmp ^= (1u<<i);
    if( !pocsag_syndrome(tmp) ) {
      return tmp;
    }
  }

  return ret;
}


//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
void pocsag_12_decode(int16_t *buffer, int len) {
int8_t bit_val;

    if(!init_pocsag) {
      fsk12_set_loop_filter_bw(1.0/10.0);
      fsk12_set_srate(48000.0f, 1200.0f);
      init_pocsag=1;
    }

    nbits = fsk_12_decode_nrz( buffer, len);

    if(nbits>0) decode_bits = fsk_12_get_decode_bits();

    while(nbits>0) {

      bit_val = (decode_bits & (1<<nbits-1));
      nbits--;

      shiftreg<<=1;
      if( bit_val ) {
        shiftreg |= 1;
      }
        if(pocsag_state < POCSAG_ADDRESS) frame_count=0; 

    handle_sync:
      if(shiftreg==POCSAG_SYNC_WORD) {

        if(pocsag_state < POCSAG_ADDRESS) pocsag_state=POCSAG_ADDRESS;

        if(pocsag_debug_on) printf(" <SYNC> ");
        timeout=0;
        bitcount=0;
        frame_count=0;
        function=2;
        add=0;
        shiftreg=0;
        continue;
      }

    handle_idle:
      if(shiftreg==POCSAG_IDLE) {

        if(pocsag_state < POCSAG_ADDRESS) pocsag_state=POCSAG_ADDRESS;  //not completely sure should do this since we 
                                                                          //don't know what the frame count would be

        if(pocsag_debug_on) printf(" <IDLE: %d> ", frame_count);
        bitcount=0;
        goto inc_fcount;
      }

      if(pocsag_state>=POCSAG_ADDRESS &&  ++bitcount==32) {
        bitcount=0;

    handle_data:
        //is this message data
        if( (shiftreg>>31) == 1 ) {
          //if( pocsag_state != POCSAG_MESSAGE ) {
          //  if(pocsag_debug_on) goto handle_message;  //handle message anyway if pocsag_debugging 
          //  goto inc_fcount;
         // }
          pocsag_state = POCSAG_MESSAGE;
        }

        //is this address data
        if( (shiftreg>>31) == 0 ) {
          pocsag_state = POCSAG_ADDRESS;
        }

        data=shiftreg;

        if(pocsag_syndrome(data)) {
          data = try_repair(data);
          if(pocsag_syndrome(data)) {
            if(pocsag_debug_on) printf(" <BIT ERROR> ");
            goto inc_fcount;
          } else {
            if(pocsag_debug_on) printf(" <BIT REPAIR> ");
            shiftreg = data;
            if( shiftreg==POCSAG_SYNC_WORD ) goto handle_sync;
            else if( shiftreg==POCSAG_IDLE ) goto handle_idle;
            else goto handle_data;
          }
        }

        switch(pocsag_state) {

        case  POCSAG_MESSAGE :
    handle_message:

          data = shiftreg;
          data<<=1;

          for(bc=0; bc<20; bc++) {

            if( function==0x00 ) {

              c >>=1;
              if(data&0x80000000) {
                c |= 0x10;
              }
              //c &= 0x0f;
              if(++cw_count==4) {
                printf("%02X,", c);
                if( (c&0x10)==0x10) {
                  printf("<EOT>");
                  pocsag_state=POCSAG_NOSYNC;
                }
                cw_count=0x00;
                c = 0x00;
              }

              /*
              if(++cw_count==4) {
                if( c <= 9 ) c+='0';
                else if( c == 0x0a ) c = ' ';  
                else if( c== 0x0b ) c = 'U';  
                else if( c== 0x0c ) c = ' ';  
                else if( c== 0x0d ) c = '-';  
                else if( c== 0x0e ) c = ']';  
                else if( c== 0x0f ) c = '[';  
                //else {
                //  function = 2;
               // }

                c&=0x3f;

                //if( isalnum(c) || isspace(c) || c=='-' || c==':' || c=='@') {
                if( isprint(c) ) { 
                  printf("%s\0", &c);
                  timeout=0;
                } else {
                  if(c==0x17) printf("<EOT>");
                  if(pocsag_debug_on) printf(" <0x%02x> ", c);
                  //if(c==EOM) pocsag_state=POCSAG_NOSYNC;
                }
                cw_count=0x00;
                c = 0x00;

              }
              */
            } else if( function!=0  ) {

              if(data&0x80000000) {
                c |= 0x80;
              }
              c >>=1;


              if(++cw_count==7) {
                //if( isalnum(c) || isspace(c) || c=='-' || c==':' || c=='@') {
                if( isprint(c) ) { 
                  printf("%s\0", &c);
                  timeout=0;
                } else {
                  if(pocsag_debug_on) printf(" <0x%02x> ", c);
                  if(c==0x17) printf("EOT>");
                  //if(c==EOM) pocsag_state=POCSAG_NOSYNC;
                }
                c = 0x00;
                cw_count=0;
              }
            }
            data <<= 1;
          }
          break;

        case  POCSAG_ADDRESS :
          add = ((shiftreg>>13)&0x0003ffff);
          add <<= 3;
          add |= ((frame_count>>1)&0x07);


          function = (shiftreg>>11)&0x03;

          memset(header_buffer,0x00,sizeof(header_buffer));
          int ret=snprintf(header_buffer, sizeof(header_buffer)-1, "\r\n[POCSAG-12, Freq: %4.4f MHz] address  %7d dec, function %d, rssi: %-3.0f dBm,  Message:   \0", config->frequency, add, function, get_rssi());
          printf("%s", header_buffer);
          timeout=0;

          cw_count=0;
          c = 0x00;

          if( function==0x00 ) {
            digit_count=0;
          }

          pocsag_state=POCSAG_MESSAGE;
          break;

        }
    inc_fcount:
        if(++frame_count==16) {
         pocsag_state=POCSAG_NOSYNC;
        }

      }
  }
}
