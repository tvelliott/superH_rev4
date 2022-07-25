

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




#include <complex.h>
#include <math.h>

#include "globals.h"
#include "stm32h7xx_hal.h"
#include "lwip.h"
#include "mbelib_test_main.h"

#define ARM_MATH_CM7 1
#include "arm_common_tables.h"
#include "arm_math.h"



#include "p25_decode.h"
#include "p25_stats.h"

#include "channel_filter.h"

#include "fsk_decode_12.h"
#include "pocsag.h"

#include "telnet.h"

#include "lwip/udp.h"

#include "main.h"
#include "std_io.h"
#include "agc.h"
#include "freqsynth.h"
#include "rtl_tcp_driver.h"
#include "rfdemod.h"

#include "config.h"
#include "adsb_decoder.h"
#include "multimon.h"
#include "dmr_state.h"
#include "syncmon.h"

// These vectors need to be in address range that the DMA can access.
// See .ld file for definition of sections
// Also see MPU_Config().  Cache is disabled for these buffers so DMA works correctly
ALIGN_32BYTES( static uint32_t ADCBuffer[ADC_SIZE*2] __attribute__( ( section( ".ADCSection" ) ) ) ); //dual-mode interleaved ADC1 and ADC2, 8th-order BB filter, 2 channels, differential.
//ALIGN_32BYTES( static uint32_t ADCBuffer3[ADC_SIZE] __attribute__( ( section( ".ADC3Section" ) ) ) ); //regular conversion, 2.8 MHz BW, ADC3, 2-channels single-ended
ALIGN_32BYTES( volatile uint16_t   audio_tx[256] __attribute__( ( section( ".AudioSection" ) ) ) ); //I2S audio buffer


volatile int do_update_agc;
volatile int rssi_offset;
volatile int bb_gain;
static volatile int agc_update_mod;

uint32_t udp_send_ip_bak;

uint8_t *adc_ptr_half = ( ( uint8_t * ) &ADCBuffer[0] );
uint8_t *adc_ptr_full = ( ( uint8_t * ) &ADCBuffer[0] ) + ( ADC_SIZE * 4 ) / 2;

volatile int adc_idx = 0;
volatile float complex nco_buffer[1];

volatile float sweep[4096];
volatile double sweep_freq_start;
volatile double sweep_freq_stop;
volatile double sweep_step;
volatile int sweep_idx;
volatile float max;

static uint32_t telnet_tick_time;
extern volatile int8_t adc_ready;
volatile int adc_rate;
volatile int adc_count;
volatile int do_i2s;
volatile int is_playing;
volatile uint32_t out_s = 0;
volatile uint32_t out_e = 0;
volatile int alen;
volatile int do_sample_process = 1;

volatile int adc12_dma = 0;
volatile int adc3_dma = 0;

static volatile struct pbuf *p;
static volatile struct pbuf *pb;

static volatile int squelch_active = 1;
volatile int squelch_timer;

static volatile float angle1;
static volatile float angle2;

/* Private variables ---------------------------------------------------------*/
volatile struct ip4_addr udp_saddr;

volatile int do_freq_offset = 1;

volatile int do_48khz_iq_only = 0; //1==do RF IQ resamp from 100khz to 48khz only and send that over udp


volatile int channel_time;
volatile int channel_timeout;
volatile int tdu_count;
volatile int channel_settle;
volatile int stats_buffer_ready;
volatile int fft_buffer_ready;

volatile int did_print_channel;
static volatile float real_amp = 0.0f;

void p25_net_tick( void );
void process_incoming_samples( int16_t val );
void udp_send_data( uint8_t *buffer, int len );
void p25_udp_init( void );
void udp_data_rx( void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip4_addr *addr, uint16_t port );
int _mem_free( void );
void timer_tick( void );
void main_tick( void );

volatile uint32_t second_tick;
volatile uint32_t uptime_sec;
volatile int sample_cnt;
int found_signal;

volatile char in_buffer[1500 * IN_BUFFER_N];
volatile char in_buffer_tmp[1500 * IN_BUFFER_N];
volatile int in_buffer_len;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int64_t __errno;

volatile struct udp_pcb *udp_data_pcb;
int audio_frames_done;
int do_silent_frame;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
I2S_HandleTypeDef hi2s2;
SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_adc3;
ADC_HandleTypeDef hadc3;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

static void SystemClock_Config( void );
static void MX_GPIO_Init( void );
static void MX_ADC1_Init( int oversample_rate, int clockdiv, int rightshift );
static void MX_ADC2_Init( int oversample_rate, int clockdiv, int rightshift );
static void MX_ADC3_Init( void );
static void MX_SPI3_Init( void );
static void MX_DMA_Init( void );
static void MX_I2S2_Init( void );
static void MX_TIM2_Init( void );
static void MX_TIM3_Init( void );

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void MPU_Config( void );
void tcp_iperf_init( void );
static volatile uint32_t current_time;
static uint32_t led_time;
static uint32_t p25_time;
static uint8_t led_state;
static uint32_t clk_mhz;

// initialize frame generator properties

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

unsigned int did_sym = 0;

extern volatile int do_gqrx;
volatile int audio_out_rate;
volatile int audio_in_rate;

volatile struct pbuf *pb_adc; //not freed for speed when in ADSB mode
volatile struct pbuf *pb_audio; //not freed for audio_buffer output 
volatile int do_adsb_preamble;
volatile char adsb_buffer[ADC_SIZE*4*2];
volatile int  adsb_buffer_idx;
volatile int adsb_dc_offset;
int adsb_preamble_count;
volatile int adsb_overruns;

struct demod_state flexstate;

static volatile int16_t audio_buffer[512];
static volatile int ab_idx;

static volatile  uint32_t tmp_out_e;
static volatile  uint32_t tmp_out_s;
static volatile  float gain;
static volatile int do_vaudio;
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
int main( void )
{

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  SystemClock_Config();

  HAL_Delay( 100 );

  //HAL_RCC_MCOConfig( RCC_MCO2, RCC_MCO2SOURCE_SYSCLK, RCC_MCODIV_8 );
  //delay_ms_ni(50);

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  // activate SRAM1\2\3
  RCC->AHB2ENR |= ( 0x7 << 29 );

  MPU_Config();

  SCB_EnableICache();
  //SCB_DisableICache();

  /* USER CODE END 1 */

  /* Enable D-Cache-------------------------------------------------------------*/
  SCB_EnableDCache();
  //SCB_DisableDCache();

  /* MCU Configuration----------------------------------------------------------*/


  /* Initialize all configured peripherals */
  MX_SPI3_Init();
  MX_GPIO_Init();
  MX_DMA_Init();
  //MX_ADC1_Init(20,8,2);
  //MX_ADC2_Init(20,8,2);
  //MX_ADC3_Init();


  //TODO: FIX. disable ir remote related for now
  //MX_TIM2_Init();
  //MX_TIM3_Init();


  MX_GPIO_Init();
  HAL_GPIO_WritePin( synth1_cs_GPIO_Port, synth1_cs_Pin, GPIO_PIN_SET );
  HAL_GPIO_WritePin( synth2_cs_GPIO_Port, synth2_cs_Pin, GPIO_PIN_SET );
  HAL_GPIO_WritePin( mixer_cs_GPIO_Port, mixer_cs_Pin, GPIO_PIN_SET );

  leds_on();

  init_globals(); //this needs to be done before uart configuration  (baud rate) and networking configuration, etc


  rssi_offset = config->rssi_offset;
  bb_gain = config->gain1;

  MX_LWIP_Init();

  printf( "\r\n\r\nInstruction Cache Enabled" );
  printf( "\r\nData Cache Enabled" );
  printf( "\r\nHAL Initialized" );

  clk_mhz = SystemCoreClock / 1e6;
  printf( "\r\nSTM32H743 Running @ %lu MHz", clk_mhz );


  tcp_iperf_init();
  printf( "\r\niperf3 server started on port 5201" );



  adsb_init();
  p25_udp_init();
  init_dmr();
  telnet_init();
  rtl_tcp_driver_init();

  init_rfdemod();

  flex_init(&flexstate);

  int free = _mem_free();
  printf( "\r\nheap mem free %d bytes\r\n", free );

  init_dc_correction( ( double ) config->iq_dc_off_rate );


  HAL_GPIO_WritePin( rf_sw_GPIO_Port, rf_sw_Pin, GPIO_PIN_RESET );    //select RFB port while synths init
  set_atten( 63 ); //high front-end attenuation while synths init



  leds_on();
  HAL_Delay( 10 );
  init_synth2();
  HAL_Delay( 10 );
  init_synth1();
  HAL_Delay( 10 );
  init_mixer();
  HAL_Delay( 10 );
  leds_off();

  HAL_GPIO_WritePin( rf_sw_GPIO_Port, rf_sw_Pin, GPIO_PIN_SET );    //select RFA port

  MX_I2S2_Init();

  int i;
  for( i = 0; i < 256; i++ ) {
    //audio_tx[i] = (uint16_t) rand()%65535;
    audio_tx[i] = ( uint16_t ) 0;
  }


  sweep_freq_start = 900.0;
  sweep_freq_stop = 920.0;
  sweep_step = ( sweep_freq_stop - sweep_freq_start ) / ( float )( sizeof( sweep ) / 4 );
  //sweep_step= 0.020;

#if 0
  HAL_SYSCFG_VREFBUF_VoltageScalingConfig( SYSCFG_VREFBUF_VOLTAGE_SCALE0 );
  HAL_SYSCFG_EnableVREFBUF();
  HAL_SYSCFG_VREFBUF_HighImpedanceConfig( SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE );
#else
  HAL_SYSCFG_DisableVREFBUF();
  HAL_SYSCFG_VREFBUF_HighImpedanceConfig( SYSCFG_VREFBUF_HIGH_IMPEDANCE_ENABLE );
#endif

  __preset = 0;
  read_config();


  //HAL_TIM_Base_Start( &htim2 );

  start_eth_auto_neg();

  pb_adc = pbuf_alloc( PBUF_RAW, ADC_SIZE*4, PBUF_RAM );

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  main loop
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  while( 1 ) {


    timer_tick(); //timer related for P25

    int i;
    uint32_t prim;

    MX_LWIP_Process();

    main_tick();    //check various timers related to debug output, etc


    #if 1
    //move this to main loop outside of interrupt handler
    if(do_update_agc) {
      prim = __get_PRIMASK();
      __disable_irq();

      do_update_agc=0;

      if( !prim ) {
        __enable_irq();
      }

      //AGC function
      update_agc();
    }
    #endif
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void reconfig_adc( int channel_bw )
{
  uint32_t prim;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};



  if( adc12_dma ) {
    HAL_ADCEx_MultiModeStop_DMA( &hadc1 );
    delay_ms_ni( 1 );
    adc12_dma = 0;
    HAL_ADC_DeInit( &hadc1 );
    HAL_ADC_DeInit( &hadc2 );
  }
  if( adc3_dma ) {
    HAL_ADC_Stop_DMA( &hadc3 );
    delay_ms_ni( 1 );
    adc3_dma = 0;
    HAL_ADC_DeInit( &hadc3 );
  }

  delay_ms_ni( 1 );

  prim = __get_PRIMASK();
  __disable_irq();

  config->channel_bw = channel_bw;

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3|RCC_PERIPHCLK_SPI2
                                |RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M = 2;
    PeriphClkInitStruct.PLL2.PLL2N = 157;
    PeriphClkInitStruct.PLL2.PLL2P = 4;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 1840;
    PeriphClkInitStruct.PLL3.PLL3M = 1;
    PeriphClkInitStruct.PLL3.PLL3N = 19;
    PeriphClkInitStruct.PLL3.PLL3P = 2;
    PeriphClkInitStruct.PLL3.PLL3Q = 2;
    PeriphClkInitStruct.PLL3.PLL3R = 2;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;

  switch( config->sa_mode ) {

  case  MODE_ADSB :

    PeriphClkInitStruct.PLL3.PLL3N = 22;
    PeriphClkInitStruct.PLL3.PLL3FRACN = config->adc3_frac;    //2MSPS in sa_mode==5 mode  (ADSB)
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      _Error_Handler( __FILE__, __LINE__ );
    }
    MX_ADC3_Init();

    mixer_write_reg( 0x0e, 4 ); //I dc offset, leave at 1 for best sensitivity
    mixer_write_reg( 0x0f, 4 ); //Q dc offset, leave at 1 for best sensitivity

    HAL_ADC_Start_DMA( &hadc3, ( uint32_t * ) ADCBuffer, ADC_SIZE * 8 ); //ADC3  regular conversion 2 channels, single-ended
    adc3_dma = 1;
    break;

  case  MODE_FM :
  case  MODE_IQ :
  case  MODE_P25 :
  case  MODE_SYNCMON :
  case  MODE_AM :
  case  MODE_4 :
  case  MODE_6 :
  case  MODE_DMR :
  default :

    PeriphClkInitStruct.PLL3.PLL3N = 19;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 1484;  //800ksps in channel_bw=1 mode
    PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      _Error_Handler( __FILE__, __LINE__ );
    }
    switch( config->channel_bw ) {
    case  0 : //narrow
      MX_ADC1_Init( 16, 8, 2 ); //oversample=20, clockdiv=8, righshift=2
      MX_ADC2_Init( 16, 8, 2 );
      //set_bw( 1 ); // +/- 10kHz
      break;

    case  1 : //wide
      MX_ADC1_Init( 3, 4, 1 ); //oversample=2, clockdiv=4, righshift=1
      MX_ADC2_Init( 3, 4, 1 );
      //set_bw( 9 ); // +/- 90 kHz
      break;

    case  2 : //medium
      MX_ADC1_Init( 12, 4, 1 ); //oversample=12, clockdiv=4, righshift=1
      MX_ADC2_Init( 12, 4, 1 );
      //set_bw( 2 ); // +/- 20kHz
      break;
    }

    mixer_write_reg( 0x0e, (uint8_t) config->mixer_doi ); //I dc offset, leave at 1 for best sensitivity
    mixer_write_reg( 0x0f, (uint8_t) config->mixer_doq ); //Q dc offset, leave at 1 for best sensitivity

    HAL_ADCEx_MultiModeStart_DMA( &hadc1, ( uint32_t * ) ADCBuffer, ADC_SIZE ); //ADC1 and ADC2 interleaved, 2 channels, differential-ended
    adc12_dma = 1;
    break;
  }


  out_s = 0;
  out_e = 0;
  is_playing = 0;   //allow some audio to buffer back up again
  udp_out_len = 0;

  if( !prim ) {
    __enable_irq();
  }

  set_freq_mhz( config->frequency + config->if_frequency ); //reset freq because do_low_if_mix might have changed
}

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void check_udp_out()
{
  uint32_t prim;
  uint8_t buffer[1514];
  int len=0;

  MX_LWIP_Process();

  switch( config->sa_mode ) {
  case  MODE_P25 :
  case  MODE_DMR :
  case  MODE_SYNCMON :
  case  MODE_FM :
  case  MODE_AM :

    if( config->udp_mode==UDP_STREAM_DEMOD && udp_out_len%2==0 && udp_out_len>=64 ) { 
      prim = __get_PRIMASK();
      __disable_irq();
      len = udp_out_len;
      goto len_gtz;
    }
    else if( config->udp_mode==UDP_STREAM_IQ16_DECFLT && udp_out_len > 255 && udp_out_len%2==0) { 
      prim = __get_PRIMASK();
      __disable_irq();
      len = udp_out_len;
      goto len_gtz;
    }
      //else if( config->udp_mode==UDP_STREAM_IQ16_DECFLT && udp_out_len == ((ADC_SIZE/2)/config->decimate)*8 ) { 
       // len = udp_out_len;
      //}

    if(len>0) {
len_gtz:
      if(len>1460) len=1460;
      if(len<=0) goto len_isz;

      pb = pbuf_alloc( PBUF_RAW, len, PBUF_RAM );
      if( pb != NULL ) {
        memcpy( pb->payload, ( uint8_t * ) udp_out_buffer, len );
        memcpy( buffer, ( uint8_t * ) udp_out_buffer, len );

        udp_sendto( udp_data_pcb, pb, &udp_saddr, UDP_PORT );

        if(pb) pbuf_free( pb );
      }

len_isz:
      udp_out_len = 0;

      if( !prim ) {
        __enable_irq();
      }

      //run decoders if enabled
      if( !squelch_active && len > 0 && config->udp_mode==UDP_STREAM_DEMOD) {
        if( config->do_pagers ) {
          pocsag_12_decode( ( int16_t * ) &buffer[0], len / 2 );
          flex_demod(&flexstate, ( int16_t * ) &buffer[0], len / 2);
        }
        if(config->do_acars) {
          acars_decode(&buffer[0], len/2);
        }
      }

    }

    break;

  case MODE_IQ  :
    if( udp_out_len == ( ADC_SIZE * 4 ) / config->decimate ) {

      prim = __get_PRIMASK();
      __disable_irq();

      if(udp_out_len>1460) udp_out_len=1460;

      pb = pbuf_alloc( PBUF_RAW, udp_out_len, PBUF_RAM );
      if( pb != NULL ) {
        memcpy( pb->payload, ( uint8_t * ) udp_out_buffer, udp_out_len );
        udp_sendto( udp_data_pcb, pb, &udp_saddr, UDP_PORT );
        udp_out_len = 0;
        if(pb) pbuf_free( pb );
      }

      if( !prim ) {
        __enable_irq();
      }

    }

  case MODE_4  :
    if( udp_out_len == ( ADC_SIZE * 4 ) ) {

      prim = __get_PRIMASK();
      __disable_irq();
      if(udp_out_len>1460) udp_out_len=1460;

      pb = pbuf_alloc( PBUF_RAW, udp_out_len, PBUF_RAM );
      if( pb != NULL ) {
        memcpy( pb->payload, ( uint8_t * ) udp_out_buffer, udp_out_len );
        udp_sendto( udp_data_pcb, pb, &udp_saddr, UDP_PORT );
        udp_out_len = 0;
        if(pb) pbuf_free( pb );
      }

      if( !prim ) {
        __enable_irq();
      }

    }
    break;
  }


  if( config->sa_mode!=MODE_P25 && config->sa_mode!=MODE_DMR && 
                     ab_idx>256  && (config->udp_mode==UDP_STREAM_AUDIO_48K_S16 ||
                     config->udp_mode==UDP_STREAM_AUDIO_48K_S16_CONT) ) {
    prim = __get_PRIMASK();
    __disable_irq();

    len = ab_idx*2;
    if( config->udp_mode!=UDP_STREAM_AUDIO_48K_S16_CONT && squelch_active) {
      len = 0;
    }

    if(len>0) {
      if(len>1460) len=1460;
      //audio buffer pbuf 
      pb_audio = pbuf_alloc( PBUF_RAW, len , PBUF_RAM );

      if(pb_audio!=NULL) {
        memcpy( pb_audio->payload, ( uint8_t * ) audio_buffer , len );
        udp_sendto( udp_data_pcb, pb_audio, &udp_saddr, UDP_PORT );
        if(pb_audio ) pbuf_free(pb_audio);
      }
    }

    ab_idx=0;
    if( !prim ) {
      __enable_irq();
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void led1_on( void )
{
  HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET );
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void led1_off( void )
{
  HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET );
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void led2_on( void )
{
  HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET );
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void led2_off( void )
{
  HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET );
}



//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void leds_on( void )
{
  HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET );
  HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET );
}
//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void leds_off( void )
{
  HAL_GPIO_WritePin( LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET );
  HAL_GPIO_WritePin( LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET );
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void main_tick()
{
  uint32_t prim;
  int ii;
  int ii_len;
  static short *main_in_ptr;


  current_time = HAL_GetTick();
  if( ( current_time - telnet_tick_time ) > 50 ) {
    telnet_tick();
    telnet_tick_time = current_time;
  }

  current_time = HAL_GetTick();
  if( current_time - second_tick > 1000 ) {
    second_tick = current_time;
    uptime_sec++;
    if( uptime_sec < 5 ) {
      //if(uptime_sec&0x01) leds_on();
      // else leds_off();
    }

    if( config->show_srate && config->sa_mode  ) printf( "\r\nadc_rate %d", adc_rate );
    if( config->show_srate && config->sa_mode == MODE_FM ) printf( "\r\naudio_in_rate %d", audio_in_rate );
    if( config->show_srate && config->sa_mode == MODE_FM ) printf( "\r\naudio_out_rate %5.1f", audio_adj_rate );


    //DONT DO THIS IF USING GOOD 10MHZ REFERENCE
    //update the audio resample rate.  compensates for differences in the ADC clocks and I2S clocks
    //this is an attempt to avoid over-runs and under-runs in the audio output buffers for I2S dac.
    if( audio_in_rate > 0 && config->sa_mode != MODE_P25 && 
      config->sa_mode!=MODE_ADSB && config->sa_mode!=MODE_DMR && 
      config->sa_mode!=MODE_SYNCMON) {
        
        audio_adjust_rate( audio_in_rate );
    }

    //check if channel change glitched adc config and try to recover
    //if( adc_rate == 0 && audio_in_rate > 0 && config->sa_mode!=MODE_ADSB) reconfig_adc( config->channel_bw );


    prim = __get_PRIMASK();
    __disable_irq();

    sample_cnt = 0;
    adc_rate = 0;
    audio_out_rate = 0;
    audio_in_rate = 0;

    if( !prim ) {
      __enable_irq();
    }
  }

  MX_LWIP_Process();

  command_tick(); //timer based checks in command line processing
  check_udp_out();  //IQ / DEMOD data streaming over UDP

  //update RSSI / AGC
  if( channel_settle == 0 ) { //wait for settle time to allow DC offset to quiet down after changing frequencies.


    //done in update_agc()
    //current_rssi = (int) get_rssi();  //get_rssi also updates dBFS variable

    //squelch function
    if( current_rssi > config->squelch ) {

      led1_on();
      squelch_active = 0;
      squelch_timer = 50;

      found_signal = 1;
    }

  }

    switch( config->sa_mode ) {

    //P25 decode mode, process incoming samples from FM demod
    case MODE_P25 :
    case MODE_DMR :
    case MODE_SYNCMON :
      if( do_sample_process && in_buffer_len > 0 && in_buffer_len % 2 == 0 ) {


        prim = __get_PRIMASK();
        __disable_irq();
        do_sample_process = 0;
        ii_len = in_buffer_len;
        memcpy( in_buffer_tmp, in_buffer, ii_len );
        in_buffer_len = 0;
        if( !prim ) {
          __enable_irq();
        }

        main_in_ptr = ( int16_t * ) &in_buffer_tmp[0];

        for( ii = 0; ii < ii_len / 2; ii++ ) {
          process_incoming_samples( ( int16_t ) *main_in_ptr++ );
          //if( found_signal ) process_incoming_samples( ( int16_t ) *main_in_ptr++ );

          if( ii % 4 == 0 ) check_udp_out();

        }

      }
      break;

    }

  if(config->sa_mode==MODE_ADSB) {
    adsb_main_loop();
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////
void timer_tick()
{
  int i;

  current_time = HAL_GetTick();

  //p25 scanning
  if( ( current_time - channel_time ) > 1 ) {

    channel_time = current_time;

    ir_remote_ms_tick();

    //telnet_fastpoll();


  }

  if( ( current_time - p25_time ) > 1 ) {

    p25_time = current_time;

    if( config->sa_mode == MODE_P25 ) p25_net_tick();

    audio_frames_done = 0;

    if( do_silent_frame++ > 3 ) {
      do_silent_frame = 0;
      //flush_audio();
    }


    if( squelch_timer > 0 ) {
      squelch_timer--;
      if( squelch_timer == 0 ) {
        squelch_active = 1;
        led1_off();
      }
    }

    if( channel_settle > 0 ) channel_settle--;

      //annoying test of telnet output
    if(config->logging==9998) {
      int test_mod=0;
      uint8_t c = 'A';
      for(i=0;i<1460;i++) {
        putchar_stdio(c++);
        if(test_mod++%26==0) {
          c = 'A';
        }
      }
    }

  }


  if( ( current_time - led_time ) > 100 ) {
    led_time = current_time;

    //blink the on-board red/blue leds every 10ms so we know the code is running
    led_state ^= 1;
    if( led_state ) {
      //leds_off();
    } else {
      //leds_on();
    }

    uint8_t alc = synth_read_reg( 0x00 );


    if( config->show_rssi ) printf( "\r\nrssi: %d dBm, %3.1f dBFS, front-end atten: %d, bbatten %d, gain1 %d", current_rssi, dBFS, config->front_end_atten, config->bb_atten, config->gain1 );


    if(config->sa_mode==MODE_DMR) dmr_100ms_tick();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MPU_Config( void )
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disable the MPU */
  HAL_MPU_Disable();

  /* Configure the MPU attributes as Device not cacheable
     for ETH DMA descriptors */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30040000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512B;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion( &MPU_InitStruct );

  /* Configure the MPU attributes as Cacheable write through
     for LwIP RAM heap which contains the Tx buffers */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = 0x30044000; //also defined in lwipopts.h as #define LWIP_RAM_HEAP_POINTER    (0x30044000)
  MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion( &MPU_InitStruct );


  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = ADCBuffer;
  MPU_InitStruct.Size = MPU_REGION_SIZE_2KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion( &MPU_InitStruct );

  /*
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = ADCBuffer3;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion( &MPU_InitStruct );
  */


  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = audio_tx;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;

  HAL_MPU_ConfigRegion( &MPU_InitStruct );


  /* Enable the MPU */
  HAL_MPU_Enable( MPU_PRIVILEGED_DEFAULT );
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler( char *file, int line )
{
  leds_on();
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while( 1 ) {
    main_tick();
    printf( "\r\n_Error_Handler: file %s, line %d", file, line );
    HAL_Delay( 100 );
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed( uint8_t *file, uint32_t line )
{
  while( 1 ) {
    main_tick();
    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
    HAL_Delay( 100 );
  }
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

//////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////
void udp_data_rx( void *arg, struct udp_pcb *pcb, struct pbuf *p, struct ip4_addr *addr, uint16_t port )
{

  int i;
  uint8_t *ptr;
  int in_len=0;
  struct pbuf *q;
  int len;
  int max_loop=0;

  if( p != NULL ) {

    q = p;
    len = q->len;
    in_len=q->len;
    ptr = q->payload;

    if( p->tot_len > 10 && ( in_buffer_len + p->tot_len ) <  sizeof( in_buffer ) ) {

      while( q && in_len <= p->tot_len && max_loop++<10) {
        for( i = 0; i < len; i++ ) {
          if( in_buffer_len < sizeof( in_buffer ) ) {
            in_buffer[in_buffer_len++] = *ptr++;
          }
          else {
            goto exit_clean;
          }
        }

        q = q->next;
        if( q != NULL ) {
          len = q->len;
          in_len += len;
          ptr = q->payload;
        }
      }

    }
exit_clean:
    if(p) pbuf_free( p );

  }
}


////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
void p25_udp_init()
{
  IP4_ADDR( &udp_saddr, config->udp_ip_addr[0], config->udp_ip_addr[1], config->udp_ip_addr[2], config->udp_ip_addr[3] );

  udp_send_ip_bak = config->udp_ip_addr32;

  udp_data_pcb = udp_new();
  if( udp_data_pcb == NULL ) {
    return;
  }
  //}
  udp_bind( udp_data_pcb, IP_ADDR_ANY, UDP_PORT );
  udp_recv( udp_data_pcb, ( void * ) udp_data_rx, NULL );

}

////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
void udp_send_data( uint8_t *buffer, int len )
{
  uint32_t prim;

  prim = __get_PRIMASK();
  __disable_irq();

  if(len>1460) len=1460;

  p = pbuf_alloc( PBUF_RAW, len, PBUF_RAM );
  if( p != NULL ) {
    memcpy( p->payload, buffer, len );
    udp_sendto( udp_data_pcb, p, &udp_saddr, UDP_PORT );
    if(p) pbuf_free( p );
  }

  if( !prim ) {
    __enable_irq();
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////
void process_incoming_samples( int16_t val )
{
  sample_cnt++;

  symsync_buf_in[0] = ( float ) val;

  did_sym = 0;

#if 0
  eqlms_rf_push( eqlms, ( float ) symsync_buf_in[0] );
  eqlms_rf_execute( eqlms, &eqlms_dhat[0] );
#endif


  //sync to the 4-fsk
  symsync_execute( symsync_q_4fsk, symsync_buf_in, 1, symsync_buf_out, &did_sym );

  //decode the symbols
  if( did_sym ) {

    switch(config->sa_mode) {
      case  MODE_P25   :
        p25_decode( symsync_buf_out[0], symsync_q_4fsk );
      break;

      case  MODE_DMR   :
        if(dmr_s->mode==DMR_MODE_SEEKVOICE) {
          process_dmr_symbols_data((int16_t) symsync_buf_out[0], symsync_q_4fsk);
        }
        else if(dmr_s->mode==DMR_MODE_VOICE_PLAYBACK) {
          process_dmr_symbols_voice((int16_t) symsync_buf_out[0], symsync_q_4fsk);
        }
        else {
          dmr_s->mode==DMR_MODE_SEEKVOICE;
        }
      break;
      case  MODE_SYNCMON   :
          process_symbols_mon((int16_t) symsync_buf_out[0], symsync_q_4fsk);
      break;
    }

  }

}

/////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////
double get_current_freq()
{
  return config->frequency;
}


/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Supply configuration update enable 
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source 
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;


  uint32_t *mcu_ver_ptr = 0x5c001000;
  uint32_t ver = *mcu_ver_ptr >> 16;

//the following are for ref clock of 10 MHz
#if MCU_400
  RCC_OscInitStruct.PLL.PLLN = 80; //400 mhz
#elif OVERCLOCK_MCU_480
  RCC_OscInitStruct.PLL.PLLN = 96; //480 mhz
#elif OVERCLOCK_MCU_500
  RCC_OscInitStruct.PLL.PLLN = 100; //500 mhz
  //RCC_OscInitStruct.PLL.PLLN = 101; //505 mhz works
  //RCC_OscInitStruct.PLL.PLLN = 102; //works briefly, then dies
#else
  if( ver == 0x1003 ) {
    RCC_OscInitStruct.PLL.PLLN = 80; //Y version, 400MHz
  } else if( ver == 0x2003 ) {
    RCC_OscInitStruct.PLL.PLLN = 96; //V version, 480MHz
  } else {
    RCC_OscInitStruct.PLL.PLLN = 96; //unknown version, 480MHz
  }
#endif


  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler( __FILE__, __LINE__ );
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler( __FILE__, __LINE__ );
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 2;
  PeriphClkInitStruct.PLL2.PLL2N = 157;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 1840;
  PeriphClkInitStruct.PLL3.PLL3M = 1;
  PeriphClkInitStruct.PLL3.PLL3N = 19;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOMEDIUM;
  //PeriphClkInitStruct.PLL3.PLL3FRACN = 2150;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 2153; //THIS GETS REDONE IN reconfig_adc();
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler( __FILE__, __LINE__ );
  }

}

static void MX_ADC1_Init( int oversample_rate, int clockdiv, int rightshift )
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  //hadc1.Init.OversamplingMode = DISABLE;
  if( clockdiv == 8 ) {
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  } else if( clockdiv == 4 ) {
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  }
  hadc1.Init.OversamplingMode = ENABLE;
  hadc1.Init.Oversampling.Ratio = oversample_rate;   //24 is srate of 94795.85185 sps

  switch( rightshift ) {
  case  0 :
    hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
    break;
  case  1 :
    hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_1;
    break;
  case  2 :
    hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
    break;
  case  3 :
    hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
    break;
  case  4 :
    hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
    break;
  case  5 :
    hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_5;
    break;
  case  6 :
    hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_6;
    break;
  case  7 :
    hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_7;
    break;
  case  8 :
    hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
    break;
  case  9 :
    hadc1.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_9;
    break;
  }
  hadc1.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc1.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;


  if( HAL_ADC_Init( &hadc1 ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_INTERL;
  //multimode.Mode = ADC_DUALMODE_REGSIMULT;
  multimode.DualModeData = ADC_DUALMODEDATAFORMAT_32_10_BITS; //32bits down to 10bits
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_1CYCLE;
  if( HAL_ADCEx_MultiModeConfigChannel( &hadc1, &multimode ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if( HAL_ADC_ConfigChannel( &hadc1, &sConfig ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init( int oversample_rate, int clockdiv, int rightshift )
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION_16B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  //hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;

  if( clockdiv == 8 ) {
    hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  } else if( clockdiv == 4 ) {
    hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  }
  //hadc2.Init.OversamplingMode = DISABLE;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio = oversample_rate;   //24 is srate of 94795.85185 sps

  switch( rightshift ) {
  case  0 :
    hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_NONE;
    break;
  case  1 :
    hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_1;
    break;
  case  2 :
    hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
    break;
  case  3 :
    hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_3;
    break;
  case  4 :
    hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
    break;
  case  5 :
    hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_5;
    break;
  case  6 :
    hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_6;
    break;
  case  7 :
    hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_7;
    break;
  case  8 :
    hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_8;
    break;
  case  9 :
    hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_9;
    break;
  }

  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;


  if( HAL_ADC_Init( &hadc2 ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if( HAL_ADC_ConfigChannel( &hadc2, &sConfig ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
static void MX_ADC3_Init( void )
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV4;
  //hadc3.Init.Resolution = ADC_RESOLUTION_8B;
  hadc3.Init.Resolution = ADC_RESOLUTION_10B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  if( HAL_ADC_Init( &hadc3 ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }


  /** Configure Regular Channel
  *   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_1;
  sConfig.Offset = 0;
  //sConfig.OffsetRightShift = ADC_RIGHTBITSHIFT_2;
  sConfig.OffsetRightShift = ADC_RIGHTBITSHIFT_NONE;
  sConfig.OffsetSignedSaturation = DISABLE;


  if( HAL_ADC_ConfigChannel( &hadc3, &sConfig ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  //  HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  //HAL_ADCEx_Calibration_SetValue(&hadc3, ADC_SINGLE_ENDED, 20);

  /** Configure Regular Channel
  *   */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.OffsetNumber = ADC_OFFSET_2;
  sConfig.Offset = 0;
  if( HAL_ADC_ConfigChannel( &hadc3, &sConfig ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }

   // HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  //HAL_ADCEx_Calibration_SetValue(&hadc3, ADC_SINGLE_ENDED, 20);

}


////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
void MX_I2S2_Init( void )
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  //hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.FirstBit = I2S_FIRSTBIT_MSB;
  hi2s2.Init.WSInversion = I2S_WS_INVERSION_DISABLE;
  hi2s2.Init.Data24BitAlignment = I2S_DATA_24BIT_ALIGNMENT_RIGHT;
  hi2s2.Init.MasterKeepIOState = I2S_MASTER_KEEP_IO_STATE_ENABLE;
  if( HAL_I2S_Init( &hi2s2 ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
static void MX_SPI3_Init( void )
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_01CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;


  __HAL_RCC_SPI3_FORCE_RESET();
  __NOP();
  __HAL_RCC_SPI3_RELEASE_RESET();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();
  __NOP();

  if( HAL_SPI_Init( &hspi3 ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}


////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////
static void MX_GPIO_Init( void )
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin( GPIOE, LED1_Pin | LED2_Pin, GPIO_PIN_SET );

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin( GPIOB, data_4_Pin | FLT_CS_Pin | atten_le_Pin, GPIO_PIN_RESET );

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin( GPIOF, data_0_Pin, GPIO_PIN_RESET );

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin( GPIOG, data_1_Pin | data_2_Pin, GPIO_PIN_RESET );

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin( GPIOE, postmix_gain2_Pin | postmix_gain1_Pin | postmix_gain0_Pin | data_3_Pin
                     | data_5_Pin | rf_sw_Pin | mixer_cs_Pin | synth1_cs_Pin
                     | synth2_cs_Pin, GPIO_PIN_RESET );

  /*Configure GPIO pins : LED1_Pin LED2_Pin postmix_gain2_Pin postmix_gain1_Pin
                           postmix_gain0_Pin data_3_Pin data_5_Pin rf_sw_Pin
                           mixer_cs_Pin synth1_cs_Pin synth2_cs_Pin */
  GPIO_InitStruct.Pin = LED1_Pin | LED2_Pin | postmix_gain2_Pin | postmix_gain1_Pin
                        | postmix_gain0_Pin | data_3_Pin | data_5_Pin | rf_sw_Pin
                        | mixer_cs_Pin | synth1_cs_Pin | synth2_cs_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init( GPIOE, &GPIO_InitStruct );

  /*Configure GPIO pin : IR_INPUT_Pin */
  GPIO_InitStruct.Pin = IR_INPUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init( IR_INPUT_GPIO_Port, &GPIO_InitStruct );


  /*Configure GPIO pins : data_4_Pin FLT_CS_Pin atten_le_Pin */
  GPIO_InitStruct.Pin = data_4_Pin | FLT_CS_Pin | atten_le_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init( GPIOB, &GPIO_InitStruct );

  /*Configure GPIO pin : data_0_Pin */
  GPIO_InitStruct.Pin = data_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init( GPIOF, &GPIO_InitStruct );

  /*Configure GPIO pins : data_1_Pin data_2_Pin */
  GPIO_InitStruct.Pin = data_1_Pin | data_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init( GPIOG, &GPIO_InitStruct );

  // EXTI interrupt init
  // IR_Input
  HAL_NVIC_SetPriority( EXTI3_IRQn, 0, 0 );
  HAL_NVIC_EnableIRQ( EXTI3_IRQn );

}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
static void MX_DMA_Init( void )
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority( DMA1_Stream1_IRQn, 0, 0 );
  HAL_NVIC_EnableIRQ( DMA1_Stream1_IRQn );

  HAL_NVIC_SetPriority( DMA1_Stream0_IRQn, 0, 0 );
  HAL_NVIC_EnableIRQ( DMA1_Stream0_IRQn );

  HAL_NVIC_SetPriority( DMA2_Stream0_IRQn, 0, 0 );
  HAL_NVIC_EnableIRQ( DMA2_Stream0_IRQn );
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
static void handle_adc12_double_buffer( uint8_t *bufptr )
{
  int i;
  int8_t *ptr;

  if( config->sa_mode == MODE_ADSB ) {

    adc_rate+=512;

    if(do_adsb_preamble) {
      //adsb_overruns++;
      return;
    }


    ptr = bufptr;
    if(adsb_dc_offset++%2048==0) {
      for( i = 0; i < ADC_SIZE*2; i+=2 ) {
        d_avg_i = d_avg_i * d_beta_i + ((uint8_t)ptr[i]) * d_alpha_i;
        d_avg_q = d_avg_q * d_beta_q + ((uint8_t)ptr[i+1]) * d_alpha_q;
      }
    }

    ptr = bufptr;
    for( i = 0; i < ADC_SIZE*2; i++ ) {
        *ptr++ -= ( uint8_t ) d_avg_i + (int)config->i_off;
        *ptr++ -= ( uint8_t ) d_avg_q + (int)config->q_off;
    }

    if(adsb_buffer_idx==0) {
      memcpy( adsb_buffer, bufptr, ADC_SIZE*4 );
    }
    else {
      memcpy( &adsb_buffer[ADC_SIZE*4], bufptr, ADC_SIZE*4 );
      //if(!do_adsb_preamble && i<176 && ( *(ptr-1)>6 || *(ptr-1)<-6) ) do_adsb_preamble=1;
      do_adsb_preamble=1;
    }


    if( config->udp_mode==UDP_STREAM_IQ16_DECFLT && pb_adc != NULL ) {
      if(adsb_buffer_idx==0) {
        memcpy( pb_adc->payload, ( uint8_t * ) adsb_buffer, ( ADC_SIZE * 4 ) );
      }
      else {
        memcpy( pb_adc->payload, ( uint8_t * ) &adsb_buffer[ADC_SIZE*4], ( ADC_SIZE * 4 ) );
      }
      udp_sendto( udp_data_pcb, pb_adc, &udp_saddr, UDP_PORT );
      //udp_out_len = 0;
      //pbuf_free( pb_adc );  //we don't free this buffer
    }

    adsb_buffer_idx++;
    adsb_buffer_idx&=0x01;

  } else {
    int16_t *_adc_sptr = ( int16_t * ) bufptr;
    downconvert_and_demod( _adc_sptr, ( ADC_SIZE / 2 ), config->decimate );

    #if 1
      do_update_agc=1; //don't do this in interrupt handler
    #else
      //AGC function
      update_agc();
    #endif
  }
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void HAL_ADC_ConvHalfCpltCallback( ADC_HandleTypeDef *hadc )
{
  uint8_t *adc_ptr_half = ( ( uint8_t * ) &ADCBuffer[0] );
  handle_adc12_double_buffer( adc_ptr_half );
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void HAL_ADC_ConvCpltCallback( ADC_HandleTypeDef *hadc )
{
  uint8_t *adc_ptr_full;
  if(config->sa_mode!=MODE_ADSB) {
    adc_ptr_full = ( ( uint8_t * ) &ADCBuffer[0] ) + ( ADC_SIZE * 4 ) / 2;
  }
  else {
    adc_ptr_full = ( ( uint8_t * ) &ADCBuffer[0] ) + ( ADC_SIZE * 8 ) / 2;
  }
  handle_adc12_double_buffer( adc_ptr_full );
}


//////////////////////////////////////////////////////////////////////////
// Note: the audio interrupt handler is sensitve to too much going on.
// Be sure to re-test audio quality with sig gen and FM wide channel
// if changes are made.
//////////////////////////////////////////////////////////////////////////
static void handle_i2s_double_buffer( int16_t *audio_tx_ptr )
{

  int vi;

  gain = 1.0f;


  if( config->sa_mode == MODE_FM && squelch_active ) gain =0.0f;

  if( (int)( out_e ^ out_s ) >= 8192 ) is_playing = 1; //allow some audio to buffer up to get rid of glitches


  for( vi = 0; vi < 64; vi++ ) {

    if( ( out_s!=out_e && is_playing && config->audio_on ) || do_audio_tone ) {

      if( do_audio_tone ) {
        if(config->sa_mode!=MODE_P25 && config->sa_mode!=MODE_DMR &&
          ab_idx<512) audio_buffer[ab_idx++] = ( int16_t )( config->audio_volume_f * angle1  * 32767.0f );

        *audio_tx_ptr++ = ( int16_t )( config->audio_volume_f * angle1  * 32767.0f );
        *audio_tx_ptr++ = ( int16_t )( config->audio_volume_f * angle2  * 32767.0f );
        angle1 += 0.15;
        angle2 += 0.30;
        if( angle1 > 1.0 ) angle1 = 0.0f;
        if( angle2 > 1.0 ) angle2 = 0.0f;
      } else {
        if(config->sa_mode!=MODE_P25 && config->sa_mode!=MODE_DMR &&
          ab_idx<512) audio_buffer[ab_idx++] = ( int16_t ) out_buffer[out_s];

        *audio_tx_ptr++ = ( int16_t )( out_buffer[out_s] * config->audio_volume_f * gain);
        *audio_tx_ptr++ = ( int16_t )( out_buffer[out_s++] * config->audio_volume_f * gain);
        out_s &= (uint32_t) (OUT_BUFFER_SIZE - 1);
      }
    } else {
      *audio_tx_ptr++ = 0x0000;   //silence
      *audio_tx_ptr++ = 0x0000;
      is_playing=0;

      if(config->sa_mode!=MODE_P25 && config->sa_mode!=MODE_DMR &&
        ab_idx<512 && config->udp_mode==UDP_STREAM_AUDIO_48K_S16_CONT) {
        audio_buffer[ab_idx++] = 0x00; 
      }
    }

  }
  audio_in_rate += 64;

}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void HAL_I2S_TxHalfCpltCallback( I2S_HandleTypeDef *hi2s )
{
  handle_i2s_double_buffer( &audio_tx[0] );
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void HAL_I2S_TxCpltCallback( I2S_HandleTypeDef *hi2s )
{
  handle_i2s_double_buffer( &audio_tx[128] );
}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
static void MX_TIM2_Init( void )
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0x3ff;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if( HAL_TIM_Base_Init( &htim2 ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if( HAL_TIM_ConfigClockSource( &htim2, &sClockSourceConfig ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if( HAL_TIMEx_MasterConfigSynchronization( &htim2, &sMasterConfig ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
static void MX_TIM3_Init( void )
{


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0x3ff;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 416 / 2; //1.778ms   (208=887uS bit period)
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  htim3.Init.RepetitionCounter = 0;
  if( HAL_TIM_Base_Init( &htim3 ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if( HAL_TIM_ConfigClockSource( &htim3, &sClockSourceConfig ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if( HAL_TIMEx_MasterConfigSynchronization( &htim3, &sMasterConfig ) != HAL_OK ) {
    _Error_Handler( __FILE__, __LINE__ );
  }
}
