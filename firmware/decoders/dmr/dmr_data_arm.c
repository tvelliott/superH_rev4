
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

#include "Golay.h"

#include "dmr_state.h"
#include "fec.h"

double dmr_control_freq = 454.34375; 
double dmr_current_freq;

static int dmr_data_sync_count;
static int dmr_voice_sync_count;
static uint64_t dreg;

static int symsync_reset_mod;
static struct symsync_s *ss_q = NULL;
//extern struct resamp_cf_s *ch_resamp_q_narrow;
static int sym_count;

static int skip_bits;
static int data_idx;

static int frame_count;
static unsigned char cach_hdr;
static unsigned char cach_hdr_hamming;
static int currentslot;

static dmr_state dmrstate;
dmr_state *dmr_s;

static int data_state;

static int16_t umid;
static int16_t lmid;
static int16_t center;
static int16_t max;
static int16_t min;
static int is_last;
static uint8_t cval;
static int i;
static uint8_t data_bits[64];

static uint8_t *dibit_p;
static int last_sync;

static unsigned int dibit;
static unsigned char infodata[196];
static unsigned char payload[12];
static unsigned int golay_codeword = 0;
static unsigned int bursttype = 0;
static unsigned char fid = 0;
static unsigned char csbk_id;
static int lcn=0;

extern int dmr_tick;

static int sync_mod;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static inline unsigned int get_uint(unsigned char *pay_load, unsigned int bits)
{
  unsigned int i, v = 0;
  for(i = 0; i < bits; i++) {
            v <<= 1;
            v |= pay_load[i];
        }
  return v;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void ProcessBPTC(unsigned char *infodata, unsigned char *payload)
{
  unsigned int i, j, k;
  unsigned char payload_[97];
  unsigned char data_fr[196];
  for(i = 1; i < 197; i++) {
    data_fr[i-1] = infodata[((i*181)%196)];
  }

  for (i = 0; i < 3; i++)
     data_fr[0 * 15 + i] = 0; // Zero reserved bits
  for (i = 0; i < 15; i++) {
    unsigned int codeword = 0;
    for(j = 0; j < 13; j++) {
      codeword <<= 1;
      codeword |= data_fr[j * 15 + i];
    }
    Hamming15_11_3_Correct(&codeword);
    codeword &= 0x1ff;
    for (j = 0; j < 9; j++) {
      data_fr[j * 15 + i] = ((codeword >> (8 - j)) & 1);
    }
  }
  for (j = 0; j < 9; j++) {
    unsigned int codeword = 0;
    for (i = 0; i < 15; i++) {
        codeword <<= 1;
        codeword |= data_fr[j * 15 + i];
    }
    Hamming15_11_3_Correct(&codeword);
    for (i = 0; i < 11; i++) {
        data_fr[j * 15 + 10 - i] = ((codeword >> i) & 1);
    }
  }
  for (i = 3, k = 0; i < 11; i++, k++) {
     payload_[k] = data_fr[0 * 15 + i];
  }
  for (j = 1; j < 9; j++) {
    for (i = 0; i < 11; i++, k++) {
      payload_[k] = data_fr[j * 15 + i];
    }
  }
  for (j = 0; j < 12; j++) {
    payload[j] = get_uint(payload_+8*j, 8);
  }
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void init_dmr(void) {
  dmr_s = &dmrstate;
  dmr_s->mode = DMR_MODE_SEEKVOICE;
  dmr_current_freq=dmr_control_freq;
}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
void process_dmr_symbols_data(int16_t sample16, struct symsync_s *symsync_q_4fsk ) {




  sample16 += 256;  //fixed offset

  ss_q = symsync_q_4fsk;

  max = config->dmr_level_max;
  min = -max;
  center = 0;
  umid = (int16_t) (0.5 * max);
  lmid = -umid; 

  //umid+=100;
  //center+=100;

  if(config->is_control==3) printf("\r\n%d, %d, %d", sample16, umid, lmid);

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
    goto  check_sync; 
  }


  if(data_state==1) {

      data_idx++;
      if(data_idx==12) {  //CACH
        data_idx=0;
        data_state=2;
     }

  }
  else if(data_state==2) {
    infodata[2*data_idx] = (1 & (cval >> 1)); // bit 1
    infodata[2*data_idx+1] = (1 & cval);        // bit 0

    data_idx++;
    if(data_idx==49) {
      data_idx=0;
      data_state=3;
    }
  }
  else if(data_state==3) {
      data_bits[data_idx] = cval;

      data_idx++;
      if(data_idx==6+24+4) {
        data_idx=0;
        data_state=4;


        if(last_sync == 1) {
           data_state=0;
        }

        dibit_p = (uint8_t *) &data_bits[0];
      
        // slot type
        golay_codeword  = *dibit_p++;   //0
        golay_codeword <<= 2;
        golay_codeword |= *dibit_p++;   //1

        dibit = *dibit_p++; //2
        bursttype = dibit;

        dibit = *dibit_p++; //3
        bursttype = ((bursttype << 2) | dibit);
        golay_codeword = ((golay_codeword << 4)|bursttype);

        // parity bit
        golay_codeword <<= 2;
        golay_codeword |= *dibit_p++; //4


        dibit_p += 24;  //skip over the sync 

        // repeat of slottype
        golay_codeword <<= 2;
        golay_codeword |= *dibit_p++;   //5
        golay_codeword <<= 2;



        golay_codeword |= *dibit_p++;   //6
        golay_codeword <<= 2;
        golay_codeword |= *dibit_p++;   //7
        golay_codeword <<= 2;
        golay_codeword |= *dibit_p++;   //8
        golay_codeword <<= 2;
        golay_codeword |= *dibit_p++;   //9
        golay_codeword >>= 1;
        Golay23_Correct(&golay_codeword);

        golay_codeword &= 0x0f;
        bursttype ^= golay_codeword;

        bursttype = golay_codeword;
      }
  }
  else if(data_state==4) {

    infodata[2*(data_idx+49)] = (1 & (cval >> 1)); // bit 1
    infodata[2*(data_idx+1+49)] = (1 & cval);        // bit 0
    data_idx++;
    if(data_idx==49) {
      data_idx=0;


        //if(bursttype == 3 || bursttype == 4) {
        if(bursttype == 3) {

          memset(payload,0x00,12);
          ProcessBPTC(infodata, payload);

          csbk_id = (payload[0] & 0x3f);
          //csbk_id = (payload[0] & 0x03);

          fid = payload[1];


          if(config->is_control==1 && fid == 6 && csbk_id==3) {
          //if(csbk_id==3) {
            if(config->is_control==1 && config->logging>1) printf("\r\nConn+:   BURST TYPE %d, FID %d, CSBK_ID %d", bursttype, fid, csbk_id);


            ///////////////////////////////////////////////////////////////
            //
            //p[8] is VOICE logical channel number
            //
            //What is the logical carrier number?
            //The all important ConnectPlus LCN #


            ///////////////////////////////////////////////////////////////

            //TODO:  lcn maybe interpreted differently based on repeater # reporting it? 
            //if(lcn==2) lcn=4;

            lcn = (payload[8] >> 4)&0x0f;


            if(lcn==2) { //ignore LCN=2
              data_state=0;
              return;
            }

            printf("   GOTO VOICE LCN (0x%02x) %d   ", payload[8], lcn);

            dmr_s->mode = DMR_MODE_VOICE_PLAYBACK;
            dmr_s->dmr_is_synced=1;
            dmr_tick=4000;

            if( lcn==0 ) lcn=1;
            if(lcn==1 || lcn==2 || lcn==3 || lcn==4 || lcn==8) {


              //TODO:  fix.  2 really means 4 ...sometimes.  must be a missing bit somewhere
              //uint8_t dice = (rand()%255)&0x01;
              //if(lcn==2 && dice) lcn=2;  
              //if(lcn==2 && !dice) lcn=4;

              double freq = 0.0;
              switch(lcn) {
                case  1 :
                  //set_freq_mhz( config->if_frequency + 454.34375); 
                  //dmr_current_freq=454.34375;
                break;

                case  2 :
                  dmr_current_freq=454.5687;
                  set_freq_mhz( config->if_frequency + 454.5687); 
                break;

                case  3 :
                  set_freq_mhz( config->if_frequency + 461.5750); 
                  dmr_current_freq=461.5750;
                break;

                case  4 :
                  set_freq_mhz( config->if_frequency + 462.1375); 
                  dmr_current_freq=462.1375;
                break;

                case  8 :
                  set_freq_mhz( config->if_frequency + 452.2500); 
                  dmr_current_freq=452.2500;
                break;

                default :
                  set_freq_mhz( config->if_frequency + 454.34375); 
                  dmr_current_freq=454.34375;
                  dmr_s->mode = DMR_MODE_SEEKVOICE;
                  dmr_tick=4;
                  data_state=0;
                  dmr_voice_reset();
                break;
              }
              printf("  freq: %4.4f", dmr_current_freq);
              data_state=0;
            }
            else {
              //set_freq_mhz( config->if_frequency + 454.34375); 
              dmr_s->mode = DMR_MODE_SEEKVOICE;
              dmr_s->dmr_is_synced=0;
              data_state=0;
              //dmr_tick=5;
              dmr_data_reset();
            }
          } 
          else {
            if(config->is_control==1 && config->logging>1) printf("\r\n    BURST TYPE %d, FID %d, CSBK_ID %d", bursttype, fid, csbk_id);

              data_state=0;
              return;
          }

          //skip_bits=12+54+24+54;
        }
      data_state=1;
    }
  }
  else {
    data_state=0;
  }
  //else if(data_state==5) {
    //data_idx++;
   // if(data_idx==54+24+54) {
     // data_idx=0;
      // data_state=0;
    //}
  //}

check_sync:

  //48-bit sync
  if( ( dreg & 0xffffffffffff ) == DMR_TUNING_BS_DATA_SYNC ) {
    last_sync=0;


    if(data_state==0) {
      dmr_tick=4;

      data_idx=0;
      last_sync=-1;
      skip_bits=54;
      data_state=1;
    }
    if(config->is_control==1 && config->logging>1) printf("\r\nCONTROL CHANNEL DATA_SYNC: %d, freq  %4.4f", dmr_data_sync_count++, config->frequency);
      else if(config->logging>1) printf("\r\nDMR DATA SYNC %d", dmr_data_sync_count++);
  }
  else if( ( dreg & 0xffffffffffff ) == DMR_TUNING_BS_VOICE_SYNC ) {
    //printf("\r\nVOICE_SYNC  ");
    last_sync=1;
                dmr_s->mode = DMR_MODE_VOICE_PLAYBACK;
                dmr_tick=4000;
    data_state=0;
  }

  if(dmr_tick==0) {
    if(symsync_reset_mod++ % 48000 == 0 ) {
      symsync_reset( ss_q );
    }
  }
}

/////////////////////////////////////////////////////
/////////////////////////////////////////////////////
void dmr_data_reset(void) {
  data_state=0;
  data_idx=0;
  last_sync=-1;
  symsync_reset( ss_q );
}
