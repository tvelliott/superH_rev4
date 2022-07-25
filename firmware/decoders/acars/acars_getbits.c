/*
 *  Copyright (c) 2007 by Thierry Leconte (F4DWV)
 *
 *      $Id: getbits.c,v 1.4 2007/04/15 15:06:54 f4dwv Exp $
 *
 *   This code is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU Library General Public License version 2
 *   published by the Free Software Foundation.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Library General Public License for more details.
 *
 *   You should have received a copy of the GNU Library General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "globals.h"
#include "acarsdec.h"


#define Fe 48000.0
#define Freqh 4800.0/Fe*2.0*M_PI
#define Freql 2400.0/Fe*2.0*M_PI
#define BITLEN ((int)Fe/1200)

static float h[BITLEN];

static struct bstat_s {
	float hsample[BITLEN];
	float lsample[BITLEN];
	float isample[BITLEN];
	float qsample[BITLEN];
	float csample[BITLEN];
	int is;
	int clock;
	float lin;
	float phih,phil;
	float dfh,dfl;
	float pC,ppC;
	int sgI, sgQ;
	float ea;
} bstat[1];


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void init_bits(void)
{
	int i;
	for (i = 0; i < BITLEN; i++)
		h[i] = sin(2.0 * M_PI * (float) i / (float) BITLEN);

	for (i = 0; i < BITLEN; i++) {
		bstat[0].hsample[i] = bstat[0].lsample[i] =
		    bstat[0].isample[i] = bstat[0].qsample[i] =
		    bstat[0].csample[i] = 0.0;
	}
	bstat[0].is = bstat[0].clock = bstat[0].sgI = bstat[0].sgQ = 0;
	bstat[0].phih = bstat[0].phil = bstat[0].dfh = bstat[0].dfl =
	    bstat[0].pC = bstat[0].ppC = bstat[0].ea = 0.0;
	bstat[0].lin=1.0;

}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void resetbits(int ch)
{
	bstat[ch].sgI = bstat[ch].sgQ = 0;
}

#define VFOPLL 0.7e-3
#define BITPLL 0.2

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
int getbit(short sample, unsigned char *outbits, int ch)
{
	int i, bt;
	float in, in2;
	float C;
	float I, Q;
	float oscl, osch;
	struct bstat_s *st;

  if(ch>0) return;  //1 channel

	bt = 0;
	st = &bstat[ch];

	in = (float) sample;
	st->lin = 0.003 * fabs(in) + 0.997 * st->lin;
	in /= st->lin;
	in2 = in * in;

	st->is--;
	if (st->is < 0)
		st->is = BITLEN - 1;

	/* VFOs */
	st->phih += Freqh - VFOPLL * st->dfh;
	if (st->phih >= 4.0 * M_PI)
		st->phih -= 4.0 * M_PI;
	st->hsample[st->is] = in2 * sin(st->phih);
	for (i = 0, st->dfh = 0.0; i < BITLEN / 2; i++) {
		st->dfh += st->hsample[(st->is + i) % BITLEN];
	}
	osch = cos(st->phih / 2.0);

	st->phil += Freql - VFOPLL * st->dfl;
	if (st->phil >= 4.0 * M_PI)
		st->phil -= 4.0 * M_PI;
	st->lsample[st->is] = in2 * sin(st->phil);
	for (i = 0, st->dfl = 0.0; i < BITLEN / 2; i++) {
		st->dfl += st->lsample[(st->is + i) % BITLEN];
	}
	oscl = cos(st->phil / 2.0);

	/* mix */
	st->isample[st->is] = in * (oscl + osch);
	st->qsample[st->is] = in * (oscl - osch);
	st->csample[st->is] = oscl * osch;


	/* bit clock */
	st->clock++;
	if (st->clock >= BITLEN/4 + st->ea) {
		st->clock = 0;

		/*  clock filter  */
		for (i = 0, C = 0.0; i < BITLEN; i++) {
			C += h[i] * st->csample[(st->is + i) % BITLEN];
		}

		if (st->pC < C && st->pC < st->ppC) {

			/* integrator */
			for (i = 0, Q = 0.0; i < BITLEN; i++) {
				Q += st->qsample[(st->is + i) % BITLEN];
			}

			if (st->sgQ == 0) {
				if (Q < 0)
					st->sgQ = -1;
				else
					st->sgQ = 1;
			}

			*outbits =
			    ((*outbits) >> 1) | (unsigned
						 char) ((Q * st->sgQ >
							 0) ? 0x80 : 0);
			bt = 1;

			st->ea = -BITPLL * (C - st->ppC);
			if(st->ea > 2.0) st->ea=2.0;
			if(st->ea < -2.0) st->ea=-2.0;
		}
		if (st->pC > C && st->pC > st->ppC) {

			/* integrator */
			for (i = 0, I = 0.0; i < BITLEN; i++) {
				I += st->isample[(st->is + i) % BITLEN];
			}

			if (st->sgI == 0) {
				if (I < 0)
					st->sgI = -1;
				else
					st->sgI = 1;
			}

			*outbits =
			    ((*outbits) >> 1) | (unsigned
						 char) ((I * st->sgI >
							 0) ? 0x80 : 0);
			bt = 1;

			st->ea = BITPLL * (C - st->ppC);
			if(st->ea > 2.0) st->ea=2.0;
			if(st->ea < -2.0) st->ea=-2.0;
		}
		st->ppC = st->pC;
		st->pC = C;
	}
	return bt;
}
