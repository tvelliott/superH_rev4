
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




#ifndef __P25_DECODE_H__
#define __P25_DECODE_H__

#include "symsync_rf_arm32.h"

#define DEFAULT_CENTER 0.0f
#define DEFAULT_MIN -4000.0f
#define DEFAULT_MAX 4000.0f
#define DEFAULT_STD_DEV 2000.0f;

#define DEFAULT_STD_DEV_FACTOR 1.7f
#define DEFAULT_STATS_ALPHA 0.005f
#define DEFAULT_ABS_MAX_ERROR 4500
#define DEFAULT_HISTORY_SIZE 150

#define OUT_BUFFER_SIZE 16384 //must be power of 2



void Golay23_Correct( unsigned int *block );
inline unsigned int Golay23_CorrectAndGetErrCount( unsigned int *block )
{
  unsigned int i, errs = 0, in = *block;

  Golay23_Correct( block );

  in >>= 11;
  in ^= *block;
  for( i = 0; i < 12; i++ ) {
    if( ( in >> i ) & 1 ) {
      errs++;
    }
  }

  return ( errs );
};


typedef struct {
  uint32_t enabled;
  int32_t sys_id;
  int32_t  priority;
  int32_t talkgroup;
  char    desc[32];
  char    location[16];
} tgrecord;

typedef struct {
  int talkgroup;
  int timeout;
} enc_timeout_rec;

typedef struct {
  int iden;
  uint32_t base_freq;
  int bw;
  uint32_t tx_off_vu;
  int spacing;
} base_freq_iden;

unsigned int hdu_correct_hex_word( unsigned char *hex_and_parity, unsigned int codeword_bits );
void pack_data( char *in, char *out, int errors );
int p25_mbe_eccImbe7200x4400Data( char imbe_fr[8][23], char *imbe_d );
void p25_mbe_demodulateImbe7200x4400Data( char imbe[8][23] );
int p25_mbe_eccImbe7200x4400C0( char imbe_fr[8][23] );
void deinterleave_frame( int dibit_n, uint8_t dibit_val );
void p25_processHDU( char *hex_and_parity );
void p25_decode( float sample_f, struct symsync_s *symsync_q_4fsk );
char *get_talkgroup_str( int talkgroup );
int p25_is_synced( void );
int p25_working_state( void );
void set_channel_timeout_zero( void );
unsigned short CRCCCITT(uint8_t *data, int16_t length, uint16_t seed, uint16_t final);

extern volatile short out_buffer[OUT_BUFFER_SIZE];
extern volatile short upsampled[160 * 6];
extern volatile int tdu_count;
extern int p25_hfreq_count;
extern int p25_status_mod;

#endif
