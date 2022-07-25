/* This file was automatically generated.  Do not edit! */
void pocsag_12_decode(int16_t *buffer,int len);
uint32_t try_repair(uint32_t u);
inline unsigned char even_parity(uint32_t data){
  unsigned int temp = data ^ (data >> 16);

  temp = temp ^ (temp >> 8);
  temp = temp ^ (temp >> 4);
  temp = temp ^ (temp >> 2);
  temp = temp ^ (temp >> 1);
  return temp & 1;
};
extern enum {
  POCSAG_NOSYNC,
  POCSAG_ADDRESS,
  POCSAG_MESSAGE,
}pocsag_state_enum;
extern volatile int pocsag_state;
uint32_t fsk_12_get_decode_bits();
int fsk_12_decode_nrz(int16_t *sample,int len);
