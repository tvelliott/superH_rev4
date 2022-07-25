
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



float fast_log10(float X);
void audio_adjust_rate( float rate );
void do_am_demod( int len, int decimate );
void do_fm_demod2( int len, int decimate );
void do_fm_demod( int len, int decimate );
void init_rfdemod( void );
void init_audio_dc_offset( double val );
void init_dc_offset( double val );
extern float i_stddev;
extern float q_stddev;
extern volatile uint8_t stats_idx;
extern volatile float stats_buffer[];
extern volatile float stats_buffer_I[];
extern volatile float stats_buffer_Q[];
extern volatile double d_avg_q;
extern volatile double d_beta_q;
extern volatile double d_alpha_q;
extern volatile double ALPHA_D_q;
extern volatile double d_avg_i;
extern volatile double d_beta_i;
extern volatile double d_alpha_i;
extern volatile double ALPHA_D_i;
extern volatile double a_avg;
extern volatile double a_beta;
extern volatile double a_alpha;
extern volatile double ALPHA_A;
extern volatile int udp_out_len;
extern float audio_out_level;
extern volatile int fm_cnt;
extern int eqlms_trained;
extern float eqlms_dhat[1];
extern float eqlms_d[1];
extern float eqlms_mu;
extern float eqlms_beta;
extern unsigned int eqlms_m;
extern unsigned int eqlms_k;
extern struct eqlms_rf_s *eqlms;
extern struct freqmod_s *freqmod;
extern struct nco_cf_s *nco;
extern volatile int upsample_ready;
extern volatile short upsampled[160 * 6];
extern struct resamp_rf_s *audio_resamp_q;
extern unsigned int audio_resamp_npfb;
extern float audio_resamp_bw;
extern float audio_resamp_As;
extern unsigned int audio_resamp_m;
extern float audio_resamp_r;
extern unsigned int ch_ny;
extern float complex ch_resamp_y[512];
extern struct resamp_cf_s *ch_resamp_q_wide;
extern struct resamp_cf_s *ch_resamp_q_narrow;
extern unsigned int ch_resamp_npfb;
extern float ch_resamp_bw_wide;
extern float ch_resamp_bw_narrow;
extern float ch_resamp_As;
extern unsigned int ch_resamp_m;
extern float ch_resamp_r_wide;
extern float ch_resamp_r_narrow;
extern struct symsync_s *symsync_q_4fsk;
extern struct symsync_s symsync_4fsk;
extern float symsync_buf_out[1];
extern float symsync_buf_in[1];
extern unsigned int symsync_npfb;
extern float symsync_beta;
extern unsigned int symsync_m_2fsk;
extern unsigned int symsync_k_2fsk;
extern unsigned int symsync_m_4fsk;
extern unsigned int symsync_k_4fsk;
extern int symsync_ftype;
extern volatile float audio_adj_rate;
