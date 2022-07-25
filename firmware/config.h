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



#define OVERCLOCK_MCU_480 1   //480 MHz regardless of stm32h743 version
//#define OVERCLOCK_MCU_500 1   //500 MHz regardless of stm32h743 version, audio is flaky
#define SYNTH1_LTC6948_V3 1 //first LO is version 3 of ltc6948-3
#define SYNTH2_LTC6948_V1 1 //second LO is version 1 of ltc6948-1
#define REF_ACTUAL 9.99999970 //measured ref freq in mhz
#define REF_DIV 1
#define CH_NARROW_RATE_DIV4 18752.0   //use show_srate with channel_bw = 0, divide by 4 to determine this value
#define CH_MED_RATE_DIV4 50000.0   //use show_srate with channel_bw = 0, divide by 4 to determine this value
#define CH_WIDE_RATE_DIV4 200000.0    //use show_srate with channel_bw = 1, divide by 4 to determine this value
#define DECODER_FS  (48000.00f/1.0)     //final re-sampled rate of demod/decoded audio before being fed to I2S for output
#define PLL_LOCK_RETRIES 99999   //keep trying until it is correct
#define CHANNEL_SETTLE_TIME 2   //time/ms for dc offset to settle after freq change
