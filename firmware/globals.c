

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






#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <float.h>
#include <complex.h>

#include "main.h"
#include "globals.h"
#include "crc32.h"
#include "rfdemod.h"
#include "std_io.h"
#include "nco_cf_arm32.h"
#include "freqdem_cfrf_arm32.h"
#include "mbelib_test_main.h"

extern void MX_SPI3_Init( void ); //must be after fpga bit-bang init
uint32_t GetSector( uint32_t Address );

volatile int do_audio_tone;
int do_test_mem;
int do_test_flash;
int do_write_config;
int do_read_config;
int do_delete_config;
int do_system_reset;
int do_iq_mode;
int do_rf_tx_test;
int do_ls_regs;
int do_ls_presets;
int do_ls_talkgroups;
int ls_talkgroups_parm;

static config_t configuration;
config_t *_config;
config_t1 *config;

uint32_t FirstSector = 0, NbOfSectors = 0;
uint32_t Address = 0, SECTORError = 0, Index = 0;

volatile int __preset = 0;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void flash_read( uint32_t addr, uint8_t *config_data )
{
  uint32_t prim = __get_PRIMASK();
  __disable_irq();

  memcpy( config_data, ( uint8_t * ) addr, sizeof( config_t ) );

  if( !prim ) {
    __enable_irq();
  }

}
////////////////////////////////////////////////////////////////////////////////////////////////////////
// make backup copy of configuration flash area from bank2, sector 7  to sector 6
////////////////////////////////////////////////////////////////////////////////////////////////////////
void flash_backup()
{
  uint32_t prim = __get_PRIMASK();
  __disable_irq();

  flash_write_blocking( ( uint8_t * ) CONFIG_ADDRESS_BACKUP, ( uint8_t * ) CONFIG_ADDRESS, ( 128 * 1024 ) );

  if( !prim ) {
    __enable_irq();
  }
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void list_presets( void )
{
  config_t config_tmp;
  config_t *_c = &config_tmp;
  config_t1 *_ctmp = &( _c->configt1 );

  int i;
  int linesout = 0;

  printf( " \r\n\r\n" );

  for( i = 0; i < 256; i++ ) {
    flash_read( ( uint8_t * ) CONFIG_ADDRESS + ( i * 512 ), &config_tmp );


    crc32_val = 0x0000;
    uint32_t crc = crc32_range( ( uint8_t * ) &config_tmp, sizeof( config_t ) - 4 );

    if( crc == _c->crc32 || _ctmp->frequency >= 0.5 || _ctmp->frequency <= 1300.0 && isprint( _ctmp->preset_desc_str[0] ) ) {

      //override
      if(_ctmp->if_low_high==0) {
        _ctmp->if_frequency=1552.0;
      }
      else if(_ctmp->if_low_high==1) {
        _ctmp->if_frequency=1583.0;
      }
      else {
        _ctmp->if_low_high=0;
        _ctmp->if_frequency=1552.0;
      }



      if(_ctmp->fm_deviation==0.0f) _ctmp->fm_deviation = 0.5f;
      if(_ctmp->fm_deviation < 0.1) _ctmp->fm_deviation = 0.1f;
      if(_ctmp->fm_deviation > 0.9) _ctmp->fm_deviation = 0.9f;

      if( i == __preset ) printf( " -> " );
      printf( "p# %d, des: [%s], f:%4.6f MHz, f_off:%4.6f,ch_bw %d,adc_flt %d, DEM: %s", i, _ctmp->preset_desc_str, _ctmp->frequency, _ctmp->freq_offset_mhz, _ctmp->channel_bw, _ctmp->adc_flt_bw, sa_mode_to_str(_ctmp->sa_mode) );


      printf(", AUD_FLT: ");
      switch( _ctmp->audio_filter ) {
        case  0 :
          printf("NONE");
        break;
        case  1 :
          printf("P25-P1");
        break;
        case  4 :
          printf("+/-1.8kHz");
        break;
        case  3 :
          printf("+/-2.7kHz");
        break;
        case  2 :
          printf("+/-4.5kHz");
        break;
        case  6 :
          printf("+/-7.2kHz");
        break;
        case  5 :
          printf("+/-12kHz");
        break;
      }

      printf("\r\n ");
      linesout++;
    }

    if( linesout % 16 == 0 ) {  //every 16 records, wait for console buffer to flush
      while( netbuf_s!=netbuf_e ) {
        main_tick();
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void flash_read_preset( int preset_index )
{

  leds_off();

  if( preset_index < 0 || preset_index > 255 ) {
    printf( "\r\npreset index must be between 0 and 255" );
    return;
  }

  float audio_tmp = config->audio_volume_f;

  flash_read( ( uint8_t * ) CONFIG_ADDRESS + ( preset_index * 512 ), &configuration );


  crc32_val = 0x0000;
  uint32_t crc = crc32_range( ( uint8_t * ) &configuration, sizeof( config_t ) - 4 );


  if( preset_index == 0 ) {
    rssi_offset = config->rssi_offset;
  }


  config->audio_volume_f = audio_tmp;

    if(config->adc3_frac < 1) config->adc3_frac=1;
    if(config->adc3_frac > 8000) config->adc3_frac=8000;
    if(config->dmr_level_max<3000) config->dmr_level_max=6400;
    if(config->dmr_level_max>7000) config->dmr_level_max=6400;


  if( config->p25_audio_gain < 0.0f || config->p25_audio_gain > 10.0f ) config->p25_audio_gain = 1.0f;

#if 1
  if( crc != _config->crc32 || config->frequency < 0.5 || config->frequency > 1300.0 ) {
    printf( "\r\ncorrupt preset, loading default." );
    reset_config_to_defaults();
    __preset = 0;
    config->audio_volume_f = audio_tmp;
    do_read_config = 1;
    return;
  }
#else
  if( crc != _config->crc32 ) {
    config->aud_dc_off_rate = 0.0f;
    config->freq_offset_mhz = 0.0f;
  }
#endif

  if( config->agc_mode != 0 ) {
    if( config->agc_mode == 5 ) {
      set_atten( 15 );
    } else {
      set_atten( 0 );
    }
    set_bbatten( 0 );
  } else {
    set_bbatten( config->bb_atten );
    set_atten( config->front_end_atten );
  }
  set_gain1( config->gain1 );
  set_bw( config->adc_flt_bw );
  set_bbampg( 7 );

  if( config->iq_dc_off_rate < 64 || config->iq_dc_off_rate > 32768 ) {
    config->iq_dc_off_rate = 800.0;
  }
  init_dc_correction( ( double ) config->iq_dc_off_rate ); //rf dc offset correction rate, 800 is good defaults  (1/800)
  init_audio_dc_offset( config->aud_dc_off_rate ); //fm demod dc offset  correction rate  4800 is good default (1/4800)

    //override
    if(config->if_low_high==0) {
      config->if_frequency=1552.0;
    }
    else if(config->if_low_high==1) {
      config->if_frequency=1583.0;
    }
    else {
      config->if_low_high=0;
      config->if_frequency=1552.0;
    }

      if(config->fm_deviation==0.0f) config->fm_deviation = 0.5f;
      if(config->fm_deviation < 0.1f) config->fm_deviation = 0.1f;
      if(config->fm_deviation > 0.9f) config->fm_deviation = 0.9f;
      freqdem_init( config->fm_deviation);

  if(config->if_frequency < 1550.0) {
    config->if_frequency = 1550.0;
  }
  if(config->if_frequency > 1587.0) {
    config->if_frequency = 1587.0;
  }

  if(config->if_frequency > 1583.0) printf("\r\nwarning: IF Freq is %3.1f, best if <=1583.0", config->if_frequency);
  if(config->if_frequency < 1552.0) printf("\r\nwarning: IF Freq is %3.1f, best if >=1554.0", config->if_frequency);

  set_freq_mhz2( config->if_frequency);

  if(config->sa_mode==MODE_P25) {
    mbelib_test_init(0); //imbe
  }
  if(config->sa_mode==MODE_DMR) {
    mbelib_test_init(1); //ambe
  }

  if(config->aud_agc < 0.01f) config->aud_agc=0.01f;
  if(config->aud_agc > 0.99f) config->aud_agc=0.1f;

  if(config->mixer_doi==0) config->mixer_doi=1;
  if(config->mixer_doq==0) config->mixer_doq=1;

  int adcflt = config->adc_flt_bw;  //reconfig over-writes adc filter bw
  reconfig_adc( config->channel_bw );
  set_bw( adcflt );

  nco_set_frequency( nco, config->nco_val );

  p25_reset_base_iden();

  uint32_t *ptr = 0x5c001000;
  config->mcu_ver = *ptr;
  ptr = 0x1ff1e800;
  config->mcu_unique_id = *ptr;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void delete_configuration_from_flash( void )
{

  uint8_t zeros[ sizeof(config_t) +1];
  memset(zeros, 0xff, sizeof(zeros));

  memset( config->padding, 0x00, sizeof( config->padding ) );

  crc32_val = 0x0000;
  uint32_t crc = crc32_range( ( uint8_t * ) &configuration, sizeof( config_t ) - 4 );
  _config->crc32 = crc;

  if( __preset < 0 || __preset > 255 ) {
    printf( "\r\npreset index must be between 0 and 255" );
    return;
  }

  int preset_addr = CONFIG_ADDRESS + ( __preset * 512 );

  flash_backup();

  flash_write_blocking( ( uint8_t * ) preset_addr, ( uint8_t * ) &zeros, sizeof( config_t ), 1 );


  int i;
  //copy the rest of them back in
  for( i = 0; i < 256; i++ ) {
    if( __preset != i ) {
      flash_write_blocking( ( uint8_t * )( CONFIG_ADDRESS + ( i * 512 ) ), ( uint8_t * )( CONFIG_ADDRESS_BACKUP + ( i * 512 ) ), sizeof( config_t ), 0 );
    }
  }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void flash_write_preset( int preset_index )
{


  memset( config->padding, 0x00, sizeof( config->padding ) );

  crc32_val = 0x0000;
  uint32_t crc = crc32_range( ( uint8_t * ) &configuration, sizeof( config_t ) - 4 );
  _config->crc32 = crc;

  if( preset_index < 0 || preset_index > 255 ) {
    printf( "\r\npreset index must be between 0 and 255" );
    return;
  }

  int preset_addr = CONFIG_ADDRESS + ( preset_index * 512 );

  flash_backup();

  flash_write_blocking( ( uint8_t * ) preset_addr, ( uint8_t * ) &configuration, sizeof( config_t ), 1 );


  int i;
  //copy the rest of them back in
  for( i = 0; i < 256; i++ ) {
    if( preset_index != i ) {
      flash_write_blocking( ( uint8_t * )( CONFIG_ADDRESS + ( i * 512 ) ), ( uint8_t * )( CONFIG_ADDRESS_BACKUP + ( i * 512 ) ), sizeof( config_t ), 0 );
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void flash_write_blocking( uint32_t addr, uint8_t *configuration, int len, int do_erase )
{
  static FLASH_EraseInitTypeDef EraseInitStruct;

  HAL_FLASH_Unlock();

  FirstSector = GetSector( addr );
  NbOfSectors = 1;

  EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Banks         = FLASH_BANK_2;
  EraseInitStruct.Sector        = FirstSector;
  EraseInitStruct.NbSectors     = NbOfSectors;

  if( do_erase && HAL_FLASHEx_Erase( &EraseInitStruct, &SECTORError ) != HAL_OK ) {
    printf( "\r\nerror erasing" );
    HAL_FLASH_Lock();
    return;
  }

  Address = addr;
  uint32_t idx = configuration;

  while( Address < addr + len ) {
    if( HAL_FLASH_Program( FLASH_TYPEPROGRAM_FLASHWORD, Address, idx ) == HAL_OK ) {
      Address = Address + 32; //32 byte flash pages
      idx += 32;
    } else {
      printf( "\r\nerror writing" );
      HAL_FLASH_Lock();
      return;
    }
  }

  HAL_FLASH_Lock();

}
////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
int read_configuration_from_flash( void )
{
  flash_read( CONFIG_ADDRESS, ( uint8_t * ) &configuration );


  crc32_val = 0x0000;
  uint32_t crc = crc32_range( ( uint8_t * ) &configuration, sizeof( config_t ) - 4 );

  if( crc != _config->crc32 ) return 1;
  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void write_configuration_to_flash( void )
{

#if 0
  //wipe out entire list of presets, and write to preset 0
  crc32_val = 0x0000;
  uint32_t crc = crc32_range( ( uint8_t * ) &configuration, sizeof( config_t ) - 4 );
  _config->crc32 = crc;

  flash_write_blocking( CONFIG_ADDRESS, ( uint8_t * ) &configuration, sizeof( config_t ), 1 );
#else
  flash_write_preset( __preset );
#endif
  return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////
void reset_config_to_defaults( void )
{
  do_test_mem = 0;
  do_test_flash = 0;
  do_iq_mode = 0;
  do_rf_tx_test = 0;
  do_read_config = 0;
  do_write_config = 0;
  do_system_reset = 0;
  do_ls_regs = 0;
  do_ls_presets = 0;

  config->audio_on = 1;
  config->audio_volume_f = 1.0f;
  config->logging = 1; //1=print headers, 2+ == more details

  config->p25_audio_gain = 1.0f;

  config->uart3_baudrate = 2000000;

  config->ip_addr[0] = 192;
  config->ip_addr[1] = 168;
  config->ip_addr[2] = 1;
  config->ip_addr[3] = 150;

  config->udp_ip_addr[0] = 192;
  config->udp_ip_addr[1] = 168;
  config->udp_ip_addr[2] = 1;
  config->udp_ip_addr[3] = 3;

  config->squelch = -105;
  config->rcut = 0;
  config->bw = 0;
  config->rfgain = 0; //0=auto

  config->i_off = -0.0;
  config->q_off = -0.0;

  config->i_gain = -1.0;
  config->q_gain = -1.0;

  config->frequency = 915.0;
  config->if_frequency = 1587.0;

  config->dc_off = 1;

  config->gain1 = 5;
  config->gain2 = 0;

  config->show_srate = 0;
  config->show_rssi = 0;

  config->audio_filter = 3;

  config->iq_dc_off_rate = 1.0 / 800.0;

  config->adc_flt_bw = 1; //10khz
  config->do_adsb = 0;
  config->do_pagers = 0;
  config->do_acars = 0;
  config->am_gain = 1.0;
  config->am_offset = 0;

  config->front_end_atten = 0;
  config->bb_atten = 0;
  config->bb_ampg = 6;

  config->sa_mode = MODE_IQ;
  config->channel_bw = 0; //narrow
  config->decimate = 4;

  config->agc_auto = 1; //auto switch between mode 1 and 2
  config->agc_mode = 1; //prefer front-end_atten first
  config->agc_target = -35;
  config->agc_max_gain1 = 3;
  config->agc_attack = 16;
  config->agc_decay = 32;

  config->do_low_if = 0;
  config->nco_val = 0.1;
  config->nco_offset_freq = -0.00018; //not used.  use freq_offset_mhz instead

  config->aud_dc_off_rate = 0.0f;
  memset( config->preset_desc_str, 0x00, 65 );

  config->freq_offset_mhz = 0.0f;

  config->adc3_frac = 3086;

  config->udp_mode=UDP_STREAM_AUDIO_48K_S16;
  config->fm_deviation=0.5f;
  config->dmr_level_max=6400;
  config->is_control=0;
  config->p25_sys_id = 0;
  config->if_low_high=0;
  config->aud_agc=0.5f;

  config->mixer_doi=1;
  config->mixer_doq=1;
  config->aud_agc_en=1;
  config->p25_grant=2;  //e.g. a value of allows 0x00 and 0x02 grants, set to 2  (conversation already in progress)
                        // a value of 0x00 allows only 0x00 grants
  config->am_mode=0;  //0=double side band, 1=USB, 2=LSB
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// call in main.c during init
// init variables that can be changed via the command interface /scripts / non-volatile config, etc
////////////////////////////////////////////////////////////////////////////////////////////////////////
void init_globals( void )
{

  _config = &configuration;
  _config->config = &( _config->configt1 );
  config = _config->config;

#if 0
  reset_config_to_defaults();
#else
  int flash_err = read_configuration_from_flash();
  if( flash_err || config->frequency < 0.5 || config->frequency > 1300.0 || config->channel_bw > 2 ) {
    reset_config_to_defaults();
    //write_configuration_to_flash();
  }
#endif

  uint32_t *ptr = 0x5c001000;
  config->mcu_ver = *ptr;
  ptr = 0x1ff1e800;
  config->mcu_unique_id = *ptr;
}

/////////////////////////////////////////////
// delay for 3 clock cycles * ulCount
/////////////////////////////////////////////
void __attribute__( ( naked ) ) DelayClk3( unsigned long ulCount )
{
  __asm( "    subs    r0, #1\n"
         "    bne     DelayClk3\n"
         "    bx      lr" );
}

/////////////////////////////////////////////
/////////////////////////////////////////////
void delay_ms_ni( int delay )
{
  delay_us( 1500 * delay );
}
/////////////////////////////////////////////
/////////////////////////////////////////////
void delay_ms( int delay )
{
  HAL_Delay( delay );
}
/////////////////////////////////////////////
/////////////////////////////////////////////
void delay_us( int delay )
{
  DelayClk3( delay * 134 );
}

/**
  * @brief  Gets the sector of a given address
  * @param  Address Address of the FLASH Memory
  * @retval The sector of a given address
  */
uint32_t GetSector( uint32_t Address )
{
  uint32_t sector = 0;

  if( ( ( Address < ADDR_FLASH_SECTOR_1_BANK1 ) && ( Address >= ADDR_FLASH_SECTOR_0_BANK1 ) ) || \
      ( ( Address < ADDR_FLASH_SECTOR_1_BANK2 ) && ( Address >= ADDR_FLASH_SECTOR_0_BANK2 ) ) ) {
    sector = FLASH_SECTOR_0;
  } else if( ( ( Address < ADDR_FLASH_SECTOR_2_BANK1 ) && ( Address >= ADDR_FLASH_SECTOR_1_BANK1 ) ) || \
             ( ( Address < ADDR_FLASH_SECTOR_2_BANK2 ) && ( Address >= ADDR_FLASH_SECTOR_1_BANK2 ) ) ) {
    sector = FLASH_SECTOR_1;
  } else if( ( ( Address < ADDR_FLASH_SECTOR_3_BANK1 ) && ( Address >= ADDR_FLASH_SECTOR_2_BANK1 ) ) || \
             ( ( Address < ADDR_FLASH_SECTOR_3_BANK2 ) && ( Address >= ADDR_FLASH_SECTOR_2_BANK2 ) ) ) {
    sector = FLASH_SECTOR_2;
  } else if( ( ( Address < ADDR_FLASH_SECTOR_4_BANK1 ) && ( Address >= ADDR_FLASH_SECTOR_3_BANK1 ) ) || \
             ( ( Address < ADDR_FLASH_SECTOR_4_BANK2 ) && ( Address >= ADDR_FLASH_SECTOR_3_BANK2 ) ) ) {
    sector = FLASH_SECTOR_3;
  } else if( ( ( Address < ADDR_FLASH_SECTOR_5_BANK1 ) && ( Address >= ADDR_FLASH_SECTOR_4_BANK1 ) ) || \
             ( ( Address < ADDR_FLASH_SECTOR_5_BANK2 ) && ( Address >= ADDR_FLASH_SECTOR_4_BANK2 ) ) ) {
    sector = FLASH_SECTOR_4;
  } else if( ( ( Address < ADDR_FLASH_SECTOR_6_BANK1 ) && ( Address >= ADDR_FLASH_SECTOR_5_BANK1 ) ) || \
             ( ( Address < ADDR_FLASH_SECTOR_6_BANK2 ) && ( Address >= ADDR_FLASH_SECTOR_5_BANK2 ) ) ) {
    sector = FLASH_SECTOR_5;
  } else if( ( ( Address < ADDR_FLASH_SECTOR_7_BANK1 ) && ( Address >= ADDR_FLASH_SECTOR_6_BANK1 ) ) || \
             ( ( Address < ADDR_FLASH_SECTOR_7_BANK2 ) && ( Address >= ADDR_FLASH_SECTOR_6_BANK2 ) ) ) {
    sector = FLASH_SECTOR_6;
  } else if( ( ( Address < ADDR_FLASH_SECTOR_0_BANK2 ) && ( Address >= ADDR_FLASH_SECTOR_7_BANK1 ) ) || \
             ( ( Address < FLASH_END_ADDR ) && ( Address >= ADDR_FLASH_SECTOR_7_BANK2 ) ) ) {
    sector = FLASH_SECTOR_7;
  } else {
    sector = FLASH_SECTOR_7;
  }

  return sector;
}
