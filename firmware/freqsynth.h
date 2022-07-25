
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



void init_synth1( void );
void mixer_write_reg( uint8_t a, uint8_t val );
uint8_t _xmit_spi( uint8_t out_reg );
void mixer_deselect();
void mixer_select();
void synth2_deselect();
void synth2_select();
void synth1_deselect();
void synth1_select();
void synth2_write_reg( uint8_t a, uint8_t val );
void set_freq_mhz2( double freq );
void rf_unmute( void );
void synth_write_reg( uint8_t a, uint8_t val );
void rf_mute( void );
void set_freq_mhz( double freq );
uint8_t wait_for_lock2( void );
uint8_t wait_for_lock( void );
uint8_t mixer_read_reg( uint8_t a );
void init_mixer( void );
uint8_t synth2_read_reg( uint8_t a );
void init_synth2( void );
uint8_t synth_read_reg( uint8_t a );
void dump_regs();
