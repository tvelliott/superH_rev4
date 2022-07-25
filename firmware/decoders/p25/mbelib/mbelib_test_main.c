#include "mbelib_test_main.h"

int imb_idx;
dsd_state dsdstate;
dsd_opts dsdopts;
dsd_state *t_state;
dsd_opts *t_opts;
char t_imbe_d[88];
float dsd_fbuf[160+1];
short dsd_sbuf[160+1];
int as_count;
int dsd_sample_count;

static mbe_parms mbe1;
static mbe_parms mbe2;
static mbe_parms mbe3;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mbelib_test_init( int frame_type )
{

  t_state = &dsdstate;
  t_opts = &dsdopts;

  t_opts->mbe_in_f = NULL;
  t_opts->errorbars = 1;
  t_opts->verbose = 2;
  t_opts->p25enc = 0;
  t_opts->p25lc = 0;
  t_opts->p25status = 0;
  t_opts->p25tg = 0;
  t_opts->split = 0;
  t_opts->playoffset = 0;
  t_opts->mbe_out_f = NULL;
  t_opts->frame_dstar = 0;
  t_opts->frame_x2tdma = 1;
  t_opts->frame_p25p1 = 1;
  t_opts->frame_nxdn48 = 0;
  t_opts->frame_nxdn96 = 1;
  t_opts->frame_dmr = 1;
  t_opts->frame_provoice = 0;
  t_opts->mod_c4fm = 1;
  t_opts->mod_qpsk = 1;
  t_opts->mod_gfsk = 1;
  t_opts->uvquality = 3;
  t_opts->mod_threshold = 26;
  t_opts->ssize = 36;


  t_state->cur_mp = &mbe1;
  t_state->prev_mp = &mbe2;
  t_state->prev_mp_enhanced = &mbe3;


  t_state->mbe_file_type = frame_type;  //imb ==0, ambe=1
  imb_idx = 4;
  mbe_initMbeParms( t_state->cur_mp, t_state->prev_mp, t_state->prev_mp_enhanced );


}


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
int mbe_processAudio( float *in_f, short *out_s )
{

  int i;

  for( i = 0; i < 160; i++ ) {
    *out_s++ = ( short )( ( float ) * in_f++ );
  }

  return 160;
}
