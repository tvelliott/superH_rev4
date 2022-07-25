#ifndef DSD_H
#define DSD_H
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

//#include "config.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <math.h>
#include <mbelib.h>


#define SAMPLE_RATE_IN 48000
#define SAMPLE_RATE_OUT 8000

typedef struct
{
  int onesymbol;
  FILE *mbe_in_f;
  int errorbars;
  int datascope;
  int symboltiming;
  int verbose;
  int p25enc;
  int p25lc;
  int p25status;
  int p25tg;
  int scoperate;
  int audio_in_fd;
  void *audio_in_file;
  void *audio_in_file_info;
  int split;
  int playoffset;
  FILE *mbe_out_f;
  float audio_gain;
  int serial_baud;
  int serial_fd;
  int resume;
  int frame_dstar;
  int frame_x2tdma;
  int frame_p25p1;
  int frame_nxdn48;
  int frame_nxdn96;
  int frame_dmr;
  int frame_provoice;
  int mod_c4fm;
  int mod_qpsk;
  int mod_gfsk;
  int uvquality;
  int inverted_x2tdma;
  int inverted_dmr;
  int mod_threshold;
  int ssize;
  int msize;
  int playfiles;
  int delay;
  int use_cosine_filter;
  int unmute_encrypted_p25;
} dsd_opts;

typedef struct
{
  int *dibit_buf;
  int *dibit_buf_p;
  int repeat;
  short *audio_out_buf;
  short *audio_out_buf_p;
  float *audio_out_float_buf;
  float *audio_out_float_buf_p;
  float *audio_out_temp_buf_p;
  int audio_out_idx;
  int audio_out_idx2;
  //int wav_out_bytes;
  int center;
  int jitter;
  int synctype;
  int min;
  int max;
  int lmid;
  int umid;
  int minref;
  int maxref;
  int lastsample;
  int sidx;
  int midx;
  char err_str[64];
  int symbolcnt;
  int rf_mod;
  int numflips;
  int lastsynctype;
  int lastp25type;
  int offset;
  int carrier;
  int tgcount;
  int lasttg;
  int lastsrc;
  int nac;
  int errs;
  int errs2;
  int mbe_file_type;
  int optind;
  int numtdulc;
  int firstframe;
  float aout_gain;
  float *aout_max_buf_p;
  int aout_max_buf_idx;
  int samplesPerSymbol;
  int symbolCenter;
  char algid[9];
  char keyid[17];
  int currentslot;
  mbe_parms *cur_mp;
  mbe_parms *prev_mp;
  mbe_parms *prev_mp_enhanced;
  int p25kid;

  unsigned int debug_audio_errors;
  unsigned int debug_header_errors;
  unsigned int debug_header_critical_errors;

  // Last dibit read
  int last_dibit;

} dsd_state;


#include "symsync_rf_arm32.h"

#define SAMPLE_RATE_IN 48000
#define SAMPLE_RATE_OUT 8000
#define FSK4_NTAPS  8
#define FSK4_NSTEPS 128
#define RRC_NZEROS 80
#define RRC_NXZEROS 134

#define DEFAULT_CENTER 0
#define DEFAULT_MIN -5000
#define DEFAULT_MAX 5000

//Frame sync patterns
#define P25P1_TUNING_SYNC 0x5575F5FF77FFULL

#define DMR_TUNING_BS_DATA_SYNC  0xDFF57D75DF5DULL
#define DMR_TUNING_MS_DATA_SYNC  0xD5D7F77FD757ULL
#define DMR_TUNING_BS_VOICE_SYNC 0x755FD7DF75F7ULL
#define DMR_TUNING_MS_VOICE_SYNC 0x7F7D5DD57DFDULL
#define DMR_TUNING_RC_SYNC       0x77D55F7DFD77ULL

#define X2TDMA_TUNING_BS_DATA_SYNC 0xF77557757F5FULL
#define X2TDMA_TUNING_MS_DATA_SYNC 0xDD7FD555FFF7ULL
#define X2TDMA_TUNING_BS_VOICE_SYNC 0x5DDFFDDFD5F5ULL
#define X2TDMA_TUNING_MS_VOICE_SYNC 0x77D57FFF555DULL

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

/*
 * function prototypes
 */
void processDMRdata (dsd_opts * opts, dsd_state * state);
void processDMRvoice (dsd_opts * opts, dsd_state * state);
int processAudio (float *, short *);
void writeSynthesizedVoice (dsd_opts * opts, dsd_state * state);
void playSynthesizedVoice (dsd_opts * opts, dsd_state * state);
void openAudioOutDevice (dsd_opts * opts, int speed);
void openAudioInDevice (dsd_opts * opts);

int getDibit (dsd_opts * opts, dsd_state * state);
int get_dibit_and_analog_signal (dsd_opts * opts, dsd_state * state, int * out_analog_signal);

void skipDibit (dsd_opts * opts, dsd_state * state, int count);
void saveImbe4400Data (dsd_opts * opts, dsd_state * state, char *imbe_d);
void saveAmbe2450Data (dsd_opts * opts, dsd_state * state, char *ambe_d);
int readImbe4400Data (dsd_opts * opts, dsd_state * state, char *imbe_d);
int readAmbe2450Data (dsd_opts * opts, dsd_state * state, char *ambe_d);
void openMbeInFile (dsd_opts * opts, dsd_state * state);
void closeMbeOutFile (dsd_opts * opts, dsd_state * state);
void openMbeOutFile (dsd_opts * opts, dsd_state * state);
void openWavOutFile (dsd_opts * opts, dsd_state * state);
void closeWavOutFile (dsd_opts * opts, dsd_state * state);
void printFrameInfo (dsd_opts * opts, dsd_state * state);
void processFrame (dsd_opts * opts, dsd_state * state);
void printFrameSync (dsd_opts * opts, dsd_state * state, char *frametype, int offset, char *modulation);
int getFrameSync (dsd_opts * opts, dsd_state * state);
int comp (const void *a, const void *b);
void noCarrier (dsd_opts * opts, dsd_state * state);
void initOpts (dsd_opts * opts);
void initState (dsd_state * state);
void usage ();
void liveScanner (dsd_opts * opts, dsd_state * state);
void cleanupAndExit (dsd_opts * opts, dsd_state * state);
void sigfun (int sig);
void playMbeFiles (dsd_opts * opts, dsd_state * state, int argc, char **argv);
void processMbeFrame (dsd_opts * opts, dsd_state * state, char imbe_fr[8][23], char ambe_fr[4][24], char imbe7100_fr[7][24]);
void openSerial (dsd_opts * opts, dsd_state * state);
void resumeScan (dsd_opts * opts, dsd_state * state);
int getSymbol (dsd_opts * opts, dsd_state * state, int have_sync);
void upsample (dsd_state * state, float invalue);
void processDSTAR (dsd_opts * opts, dsd_state * state);
void processNXDNVoice (dsd_opts * opts, dsd_state * state);
void processNXDNData (dsd_opts * opts, dsd_state * state);
void processP25lcw (dsd_opts * opts, dsd_state * state, char *lcformat, char *mfid, char *lcinfo);
void processHDU (dsd_opts * opts, dsd_state * state);
void processLDU1 (dsd_opts * opts, dsd_state * state);
void processLDU2 (dsd_opts * opts, dsd_state * state);
void processTDU (dsd_opts * opts, dsd_state * state);
void processTDULC (dsd_opts * opts, dsd_state * state);
void processProVoice (dsd_opts * opts, dsd_state * state);
void processX2TDMAdata (dsd_opts * opts, dsd_state * state);
void processX2TDMAvoice (dsd_opts * opts, dsd_state * state);
void processDSTAR_HD (dsd_opts * opts, dsd_state * state);
short dmr_filter(short sample);
short nxdn_filter(short sample);

#endif // DSD_H
