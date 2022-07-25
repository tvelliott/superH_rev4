
/*
 * Copyright (C) 2010 DSD Author
 * GPG Key ID: 0x3F1D7FD0 (74EF 430D F7F2 0A48 FCE6  F630 FAA2 635D 3F1D 7FD0)
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND ISC DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS.  IN NO EVENT SHALL ISC BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE
 * OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */


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




#include <stdint.h>
#include <float.h>
#include <math.h>

#include "globals.h"
#include "dsd.h"
#include "dmr_const.h"

#include "Golay.h"

#include "mbelib/mbelib.h"
#include "p25_stats.h"
#include "p25_decode.h"

#include "mbelib/mbelib_test_main.h"
#include "dmr_state.h"

static int dmr_data_sync_count;
static int dmr_voice_sync_count;
static uint64_t dreg;

static int symsync_reset_mod;
static struct symsync_s *ss_q = NULL;
//extern struct resamp_cf_s *ch_resamp_q_narrow;
static int sym_count;

static int ambe_odd;
static uint8_t ambe_bits[36];
static uint8_t ambe_bits1[36+18];
static uint8_t ambe_bits2[36+18];
static int ambe_idx;
static int skip_bits;
static int cach_idx;

static char ambe_fr[4][24];
static char ambe_d[49];
static int errors=0;
static int errors2=0;
static int ambe_state;

void process_ambe_arm(uint8_t *ambe_dibits, int is_last);
void demodAmbe3600x24x0Data (int *errs2, char ambe_fr[4][24], char *ambe_d);

static int repeats;
static int frame_count;
static unsigned char cachbits[25];
static unsigned char cach_hdr;
static unsigned char cach_hdr_hamming;
static int currentslot;

#define DATA_SYNC 0
#define VOICE_SYNC 0
static int last_sync_type;
static int first_audio;

static int16_t umid;
static int16_t lmid;
static int16_t center;
static int16_t max;
static int16_t min;
static int is_last;
static uint8_t cval;
static int i;

int dmr_tick;

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void dmr_100ms_tick(void) {
  if(dmr_tick>0) {
    dmr_tick--;
    if(dmr_tick==0) {
      dmr_s->mode=DMR_MODE_SEEKVOICE;
      if(config->is_control) {
        dmr_current_freq=454.34375;
        set_freq_mhz( config->if_frequency + dmr_control_freq); 
        printf("  -> back to control channel freq: %4.6f", config->frequency);
      }
      dmr_data_reset();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void process_ambe_arm(uint8_t *ambe_dibits, int is_last) {
  unsigned int i, dibit;

  errors=0;
  errors2=0;

  if(first_audio) {
    first_audio=0;
    //repeat_last_audio_frame(1); //1 signals silence
  }

  //de-interleave
    for (i = 0; i < 36; i++) {
      dibit = ambe_dibits[i];
      ambe_fr[rW[2*i+0]][rX[2*i+0]] = (1 & (dibit >> 1));        // bit 1
      ambe_fr[rW[2*i+1]][rX[2*i+1]] = (1 & dibit);               // bit 0
    }

  //demod and use Golay to correct errors
    demodAmbe3600x24x0Data(&errors2, ambe_fr, ambe_d);

  //if(errors2) printf("\bE ");

  process_mbe( ambe_d, errors, errors2, 0, is_last, 0);

}


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void dmr_lost_sync() {
  //lost rf carrier
  if(dmr_s->dmr_is_synced>0) printf("\r\n\r\nno rf carrier.  lost sync.");
  dmr_s->dmr_is_synced=0;
  dmr_s->mode=DMR_MODE_SEEKVOICE;

  ambe_state=0;
  //first_audio=1;
  //dmr_tick=5;
  //dmr_data_reset();
}


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void dmr_voice_reset(void) {
  ambe_state=0;
  dmr_s->dmr_is_synced=0;
  frame_count=0;
}
////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void process_dmr_symbols_voice(int16_t sample16, struct symsync_s *symsync_q_4fsk ) {


  sample16 += 256;  //fixed offset

  ss_q = symsync_q_4fsk;

  max = config->dmr_level_max;
  min = -max;
  center = 0;
  umid = (int16_t) (0.5 * max);
  lmid = -umid; 

  //umid+=100;
  //center+=100;

  dreg <<= 2;

  if( sample16 > umid ) {
    dreg |= 0x01;       //+3
    cval = 0x01;
  } else if( sample16 > center ) {
    dreg |= 0x00;       //+1
    cval = 0x00;
  } else if( sample16 < lmid ) {
    dreg |= 0x03;      //-3
    cval = 0x03;
  } else {
    dreg |= 0x02;      //-1
    cval = 0x02;
  }

  if(skip_bits>0) {
    skip_bits--;
    goto check_sync;
    //return; 
  }

  if(dmr_s->dmr_is_synced) {

      if(ambe_state<4) ambe_bits[ambe_idx++] = cval;

      if(ambe_idx==36 && ambe_state==0) { //bits1
        ambe_state=1;
        ambe_idx=0;
        #if 0
          if(last_sync_type==VOICE_SYNC) process_ambe_arm(ambe_bits, 0); 
            else dmr_s->dmr_is_synced=0;
        #else 
          process_ambe_arm(ambe_bits, 0); 
          //printf("\r\nbits1,%d\r\n", last_sync_type);
        #endif
      }

    /////////////////////////////////////////////
    //  split bits  2a/2b
    /////////////////////////////////////////////
      else if(ambe_idx==18 && ambe_state==1) {   //bits2a
        ambe_state=2;
        skip_bits=24;     //skip over 48-bit sync or signaling
        goto check_sync;
      }

      else if(ambe_idx==36 && ambe_state==2) {  //bits 2b
        //if(last_sync_type==DATA_SYNC) printf("\r\nINFO DATA SYNC");
        //if(last_sync_type==VOICE_SYNC) printf("\r\nINFO VOICE SYNC");
        ambe_state=3;
        ambe_idx=0;
        #if 0
          if(last_sync_type==VOICE_SYNC) process_ambe_arm(ambe_bits, 0);
            else dmr_s->dmr_is_synced=0;
        #else
          process_ambe_arm(ambe_bits, 0);
          //printf("\r\nbits2,%d\r\n", last_sync_type);
        #endif
      }
    /////////////////////////////////////////////


      else if(ambe_idx==36 && ambe_state==3) {  //bits3
        #if 0
          if(last_sync_type==VOICE_SYNC) process_ambe_arm(ambe_bits, 0);
            else dmr_s->dmr_is_synced=0;
        #else
          process_ambe_arm(ambe_bits, 0);
          //printf("\r\nbits3,%d\r\n", last_sync_type);
        #endif

        ambe_idx=0;
        cach_hdr=0;
        cach_hdr_hamming=0;

        skip_bits=12+54+24+54+12;  //skip over CACH(12)  and next slot  (54+24+54)
        //skip_bits=12;

        //skip_bits=0;
        //ambe_state=4;
        ambe_state=0;
        cach_idx=0;

        //if(dmr_tick < 15) dmr_tick=15;

        if(frame_count>0) frame_count--;
        //if(frame_count==1) dmr_s->dmr_is_synced=skip_bits+54+48;
      }

    #if 0
      else if(ambe_state==4 || ambe_state==5) {  

        // CACH
        cachbits[2*cach_idx] = (1 & (cval >> 1));    // bit 1
        cachbits[2*cach_idx+1] = (1 & cval);   // bit 0
        cach_idx++;
        if(cach_idx==12) {
          cach_idx=0;
          if(ambe_state==4) {
            skip_bits=54+24+54;
            ambe_state=5;
          }
          else {
            ambe_state=0;
            skip_bits=0;
            ambe_idx=0;
          }

          cach_hdr  = ((cachbits[0] << 6) | (cachbits[4] << 5) | (cachbits[8] << 4) | (cachbits[12] << 3));
          cach_hdr |= ((cachbits[14] << 2) | (cachbits[18] << 1) | (cachbits[22] << 0));
          cach_hdr_hamming = Hamming7_4_Correct(cach_hdr);

          currentslot = ((cach_hdr_hamming >> 2) & 1);      // bit 2 

          if(currentslot == 0) {
            //printf("\r\nSLOT[0]");
          } else {
            //printf("\r\nSLOT[1]");
          }
        }

     }
   #endif
  }


check_sync:

  //48-bit sync
  if( ( dreg & 0xffffffffffff ) == DMR_TUNING_BS_DATA_SYNC ) {
    last_sync_type=DATA_SYNC;
    if(dmr_s->dmr_is_synced==0) skip_bits=54+12+54;
  } else if( ( dreg & 0xffffffffffff ) == DMR_TUNING_MS_DATA_SYNC ) {
    //if(!dmr_s->dmr_is_synced) printf( "\r\nfound DMR_MS_DATA_SYNC %d", dmr_data_sync_count++);
  } else if( ( dreg & 0xffffffffffff ) == DMR_TUNING_BS_VOICE_SYNC ) {
    last_sync_type=VOICE_SYNC;


    printf( "\r\nfound DMR_BS_VOICE_SYNC %d, freq %4.4f     ", dmr_voice_sync_count++, dmr_current_freq);

    if(dmr_s->dmr_is_synced==0 && ambe_state==0) {
      #if 0
      printf("\r\ncurrent synced pos %d\r\n", dmr_s->dmr_is_synced);
      #endif
      skip_bits=18;   //right after sync skip bits2b and jump to state 3 since we already missed bits1,bits2a
      ambe_state=3;
      ambe_idx=0;
      frame_count=6;
      dmr_s->dmr_is_synced=1738;  
      dmr_tick = 40;  //4 seconds
    }
    else if(frame_count==0 && dmr_s->dmr_is_synced==24) {
      #if 0
      printf("\r\ncurrent synced pos %d\r\n", dmr_s->dmr_is_synced);
      #endif

      //seems to be in the right place to continue with next voice super frame
      //
      dmr_s->dmr_is_synced=1738;  //just a little bit longer than a BS/CACH voice super-frame
                           //need to check the other 48-bit syncs within the super-frame
                           //to determine what is going on.  Also need to add CACH decoding
      dmr_tick = 40;  //4 seconds
      frame_count=6;
    }
    else {

      //printf("\r\nframe_count %d   ", frame_count);
      //if(frame_count>0 && dmr_s->dmr_is_synced) {
       // if(dmr_tick==0) dmr_tick=4;
        //return;
      //}


      //this must be a new conversation 
      skip_bits=18;   //right after sync skip bits2b and jump to state 3 since we already missed bits1,bits2a
      ambe_state=3;
      ambe_idx=0;
      frame_count=6;
      dmr_s->dmr_is_synced=1738;
      dmr_tick = 40;  //4 seconds
    }
                         
    p25_status_mod=1;
  } else if( ( dreg & 0xffffffffffff ) == DMR_TUNING_MS_VOICE_SYNC ) {
    printf( "\r\nfound DMR_MS_VOICE_SYNC, %d  ", dmr_voice_sync_count++);
  } else if( ( dreg & 0xffffffffffff ) == DMR_TUNING_RC_SYNC ) {
    printf( "\r\nfound DMR_RC_SYNC" );
  } 

  if(dmr_s->dmr_is_synced) {
    dmr_s->dmr_is_synced--;
    if(dmr_s->dmr_is_synced==0) {

      //if(dmr_tick>2) {
       // dmr_s->dmr_is_synced=1738-24;
        //return;
      //}
      //sym_count=0;
      printf("\r\n\r\nlost sync.");
      //first_audio=1;
      //dmr_voice_sync_count=0;
      //do_audio_tone = 1; //send out short sine-wave audio tone for user feedback
      //tone_timeout = 6;
      dmr_tick=25;
      ambe_state=0;
      dmr_data_reset();
    }
  }

  #if 1
    if(config->logging>10) printf("\r\n%d", sample16);
  #endif

  if( !dmr_s->dmr_is_synced && symsync_reset_mod++ % 48000 == 0 ) {
    //printf("\r\ndmr reset symbolsync %d", sample16);
    symsync_set_lf_bw( symsync_q_4fsk, 0.01f );
    symsync_reset( ss_q );
    //resamp_cf_reset( ch_resamp_q_narrow );
  } else if( dmr_s->dmr_is_synced ) {
    symsync_reset_mod = 1;
  }
}

////////////////////////////////////////////////////////////////////////////////////////
//  this demod function is taken from dsd-dmr fork of dsd
////////////////////////////////////////////////////////////////////////////////////////
void demodAmbe3600x24x0Data (int *errs2, char ambe_fr[4][24], char *ambe_d)
{
  int i, j, k;
  unsigned int block = 0, errs = 0;
  unsigned short pr[115], foo;
  char *ambe = ambe_d;

  for (j = 22; j >= 0; j--) {
      block <<= 1;
      block |= ambe_fr[0][j+1];
  }

  errs = Golay23_CorrectAndGetErrCount(&block);

  // create pseudo-random modulator
  foo = block;
  pr[0] = (16 * foo);
  for (i = 1; i < 24; i++) {
      pr[i] = (173 * pr[i - 1]) + 13849 - (65536 * (((173 * pr[i - 1]) + 13849) >> 16));
  }
  for (i = 1; i < 24; i++) {
      pr[i] >>= 15;
  }

  // just copy C0
  for (j = 11; j >= 0; j--) {
      *ambe++ = ((block >> j) & 1);
  }

  // demodulate C1 with pr
  // then, ecc and copy C1
  k = 1;
  for (j = 22; j >= 0; j--) {
      block <<= 1;
      block |= (ambe_fr[1][j] ^ pr[k++]);
  }

  errs += Golay23_CorrectAndGetErrCount(&block);


  for (j = 11; j >= 0; j--) {
      *ambe++ = ((block >> j) & 1);
  }

  // just copy C2
  for (j = 10; j >= 0; j--) {
      *ambe++ = ambe_fr[2][j];
  }

  // just copy C3
  for (j = 13; j >= 0; j--) {
      *ambe++ = ambe_fr[3][j];
  }

  *errs2 = errs;
}


