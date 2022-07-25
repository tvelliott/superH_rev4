
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

#ifndef __DSD_FEC_H__
#define __DSD_FEC_H__

/* RS code over GF(2**mm) */
/* nn=2**mm-1 -> length of codeword */
#define MAX_NN 255
#define MAX_TT 8
#define NN ((1<<MM)-1)

typedef struct _ReedSolomon {
    int alpha_to[MAX_NN + 1];
    int index_of[MAX_NN + 1];
    int gg[17];
    /* tt -> number of errors that can be corrected */
    unsigned char tt;
    /* distance = nn-kk+1 = 2*tt+1 */
    unsigned int n;
} ReedSolomon;

unsigned char Hamming7_4_Correct(unsigned char value);
void Hamming15_11_3_Correct(unsigned int *block);
void p25_Hamming10_6_4_Correct(unsigned int *codeword);
void p25_Hamming15_11_3_Correct(unsigned int *codeword);
void p25_lsd_cyclic1685_Correct(unsigned int *codeword);
unsigned int p25_trellis_1_2_decode(uint8_t *in, uint32_t in_sz, uint8_t *out);
void Golay23_Correct(unsigned int *block);

unsigned char Hamming7_4_Encode(unsigned char value);
unsigned int Hamming15_11_3_Encode(unsigned int input);
unsigned int p25_Hamming10_6_4_Encode(unsigned int input);
unsigned int p25_Hamming15_11_3_Encode(unsigned int input);
unsigned int p25_lsd_cyclic1685_Encode(unsigned int input);
void p25_trellis_1_2_encode(unsigned char *data_in, unsigned int data_len, unsigned char *out);
unsigned int Golay23_Encode(unsigned int cw);

void rs8_init(ReedSolomon *rs, unsigned int generator_polinomial, unsigned char tt);
void rs8_encode(ReedSolomon *rs, const unsigned char *data, unsigned char *bb);
int rs8_decode(ReedSolomon *rs, unsigned char *input, unsigned char *output);
void rs6_init(ReedSolomon *rs, unsigned int generator_polinomial, unsigned char tt);
void rs6_encode(ReedSolomon *rs, const unsigned char *data, unsigned char *bb);
int rs6_decode(ReedSolomon *rs, unsigned char *input, unsigned char *output);

#endif
