

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




#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <complex.h>
#include <limits.h>

#include "command.h"
#include "globals.h"

#include "mbelib/mbelib.h"
#include "dsd.h"
#include "std_io.h"
#include "resamp_rf_arm32.h"
#include "resamp_cf_arm32.h"

#include "syncmon.h"

static int sync_count;

static const int16_t  max = DEFAULT_MAX;
static const int16_t  min = DEFAULT_MIN;
static const int16_t  center = DEFAULT_CENTER;

  //use max/min to define the center and slicer levels
static const int16_t  umid = DEFAULT_CENTER + (int16_t)( fabs( DEFAULT_MAX - DEFAULT_CENTER ) * 0.55f );
static const int16_t  lmid = DEFAULT_CENTER - (int16_t)( fabs( DEFAULT_CENTER - DEFAULT_MIN ) * 0.55f );

static uint8_t cval; 
static uint64_t dreg;
static struct symsync_s *ss_q = NULL;
static int sym_mon;

////////////////////////////////////////////////////////////////////////
//Frame sync patterns
/*
#define P25P1_TUNING_SYNC 0x5575F5FF77FFULL

#define DMR_TUNING_BS_DATA_SYNC  0xDFF57D75DF5DULL
#define DMR_TUNING_MS_DATA_SYNC  0xD5D7F77FD757ULL
#define DMR_TUNING_BS_VOICE_SYNC 0x755FD7DF75F7ULL
#define DMR_TUNING_MS_VOICE_SYNC 0x7F7D5DD57DFDULL
#define DMR_TUNING_RC_SYNC       0x77D55F7DFD77ULL


#define NXDN_TUNING_BS_DATA_SYNC  0xDDF5DD577
#define NXDN_TUNING_MS_DATA_SYNC  0xDDF5DD57F
#define NXDN_TUNING_TC_CC_SYNC    0xDDF5D5D77
#define NXDN_TUNING_TD_CC_SYNC    0xDDF5DFD57
#define NXDN_TUNING_BS_VOICE_SYNC 0xDDF5DD5D7
#define NXDN_TUNING_MS_VOICE_SYNC 0xDDF5DD5DF
#define NXDN_TUNING_TC_VOICE_SYNC 0xDDF5D75D7
#define NXDN_TUNING_TD_VOICE_SYNC 0xDDF5DF5D5

#define DSTAR_TUNING_HD_SYNC 0x77777F7D7755ULL
#define DSTAR_TUNING_SYNC 0xDDDDDF757DD5ULL


#define P25P1_TUNING_SYNC 0x5575F5FF77FFULL

#define X2TDMA_TUNING_BS_DATA_SYNC 0xF77557757F5FULL
#define X2TDMA_TUNING_MS_DATA_SYNC 0xDD7FD555FFF7ULL
#define X2TDMA_TUNING_BS_VOICE_SYNC 0x5DDFFDDFD5F5ULL
#define X2TDMA_TUNING_MS_VOICE_SYNC 0x77D57FFF555DULL
*/
////////////////////////////////////////////////////////////////////////
void process_symbols_mon(int16_t sample16, struct symsync_s *symsync_q_4fsk) {

  ss_q = symsync_q_4fsk;

  ////////////////////////////////////////////////////////////////
  dreg <<= 2;

  //slice 4 levels
  if( sample16 > umid ) {
    dreg |= 0x01;
    cval = 0x01; 
  } else if( sample16 > center ) {
    dreg |= 0x00;
    cval = 0x00; 
  } else if( sample16 < lmid ) {
    dreg |= 0x03;
    cval = 0x03; 
  } else {
    dreg |= 0x02;
    cval = 0x02; 
  }

  if( ( dreg & 0xffffffffffff ) == DMR_TUNING_BS_DATA_SYNC ) {
    printf( "\r\nfound DMR_TUNING_BS_DATA_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xffffffffffff ) == DMR_TUNING_MS_DATA_SYNC ) {
    printf( "\r\nfound DMR_TUNING_MS_DATA_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xffffffffffff ) == DMR_TUNING_BS_VOICE_SYNC ) {
    printf( "\r\nfound DMR_TUNING_BS_VOICE_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xffffffffffff ) == DMR_TUNING_MS_VOICE_SYNC ) {
    printf( "\r\nfound DMR_TUNING_MS_VOICE_SYNC, %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xffffffffffff ) == DMR_TUNING_RC_SYNC ) {
    printf( "\r\nfound DMR_TUNING_RC_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xfffffffff ) == 0xDDF5DD577 ) {
    printf( "\r\nfound NXDN_TUNING_BS_DATA_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xfffffffff ) == 0xDDF5DD57F ) {
    printf( "\r\nfound NXDN_TUNING_MS_DATA_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xfffffffff ) == 0xDDF5D5D77 ) {
    printf( "\r\nfound NXDN_TUNING_TC_CC_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xfffffffff ) == 0xDDF5DFD57 ) {
    printf( "\r\nfound NXDN_TUNING_TD_CC_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xfffffffff ) == 0xDDF5DD5D7 ) {
    printf( "\r\nfound NXDN_TUNING_BS_VOICE_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xfffffffff ) == 0xDDF5DD5DF ) {
    printf( "\r\nfound NXDN_TUNING_MS_VOICE_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xfffffffff ) == 0xDDF5D75D7 ) {
    printf( "\r\nfound NXDN_TUNING_TC_VOICE_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xfffffffff ) == 0xDDF5DF5D5 ) {
    printf( "\r\nfound NXDN_TUNING_TD_VOICE_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xffffffffffff ) == 0x77777F7D7755ULL ) {
    printf( "\r\nfound DSTAR_TUNING_HD_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xffffffffffff ) == 0xDDDDDF757DD5ULL ) {
    printf( "\r\nfound DSTAR_TUNING_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xffffffffffff ) == 0x5575F5FF77FFULL ) {
    printf( "\r\nfound P25P1_TUNING_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xffffffffffff ) == 0xF77557757F5FULL ) {
    printf( "\r\nfound X2TDMA_TUNING_BS_DATA_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xffffffffffff ) == 0xDD7FD555FFF7ULL ) {
    printf( "\r\nfound X2TDMA_TUNING_MS_DATA_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xffffffffffff ) == 0x5DDFFDDFD5F5ULL ) {
    printf( "\r\nfound X2TDMA_TUNING_BS_VOICE_SYNC %d", sync_count++ );
    sym_mon=1;
  } else if( ( dreg & 0xffffffffffff ) == 0x77D57FFF555DULL ) {
    printf( "\r\nfound X2TDMA_TUNING_MS_VOICE_SYNC %d", sync_count++ );
    sym_mon=1;
  }

  if(sym_mon++%4800==0) {
    symsync_reset( ss_q );
  }
}
