/*
 *  Copyright (c) 2007 by Thierry Leconte (F4DWV)
 *
 *      $Id: getmesg.c,v 1.3 2007/03/28 06:26:05 f4dwv Exp $
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
#include <string.h>

#include "acarsdec.h"

#define SYN 0x16
#define SOH 0x01

static struct mstat_s {
	enum { HEADL, HEADF, BSYNC1, BSYNC2, SYN1, SYN2, SOH1, TXT, CRC1,
		    CRC2, END } state;
	int ind;
	unsigned short crc;
	char txt[320];
} mstat[1];


//////////////////////////////////////////////////////////////////////////
/* CCITT 16 CRC */
//////////////////////////////////////////////////////////////////////////
#define POLY 0x1021
static void update_crc(unsigned short *crc, unsigned char ch)
{
	unsigned char v;
	unsigned int i;
	unsigned short flag;

	v = 1;
	for (i = 0; i < 8; i++) {
		flag = (*crc & 0x8000);
		*crc = *crc << 1;

		if (ch & v)
			*crc = *crc + 1;

		if (flag != 0)
			*crc = *crc ^ POLY;

		v = v << 1;
	}
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
static int build_mesg(char *txt, int len, msg_t * msg)
{
	int i, k;
	char r;

  if(len > 319) len = 319;

	/* remove special chars */
	for (i = 0; i < len; i++) {
		r = txt[i];
		if (r < ' ' && r != 0x0d && r != 0x0a) {
      r = '_'; 
    }
		txt[i] = r;
	}
	if(i>0) txt[i-1] = '\0';

	/* fill msg struct */
	k = 0;
	msg->mode = txt[k];
	k++;

	for (i = 0; i < 7; i++, k++) {
		msg->addr[i] = txt[k];
	}
	msg->addr[7] = '\0';

	/* ACK/NAK */
	msg->ack = txt[k];
	k++;

	msg->label[0] = txt[k];
	k++;
	msg->label[1] = txt[k];
	k++;
	msg->label[2] = '\0';

	msg->bid = txt[k];
	k++;

	k++;

	for (i = 0; i < 4; i++, k++) {
		msg->no[i] = txt[k];
	}
	msg->no[4] = '\0';

	for (i = 0; i < 6; i++, k++) {
		msg->fid[i] = txt[k];
	}
	msg->fid[6] = '\0';

  if(len==23) txt[k]=0x00;
	strncpy(msg->txt, &(txt[k]),275);

	return 1;
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void init_mesg(void)
{
	mstat[0].state = HEADL;
}

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
int getmesg(unsigned char r, msg_t * msg, int ch)
{
	struct mstat_s *st;
  int i;
  unsigned char m;
  int loopcnt=0;

  if(ch>0) return 8;  //1 channel

	st = &(mstat[ch]);


  for(loopcnt=0;loopcnt<730;loopcnt++) {
		switch (st->state) {
		case HEADL:

			if (r == 0xff) {
				st->state = HEADF;
				return 8;
			}
			resetbits(ch);
			return 8;
			break;
		case HEADF:
			if (r != 0xff) {

				for (i = 0, m = 1; i < 7; i++, m = m << 1) {
					if (!(r & m))
						break;
				}
				if (i < 2) {
					st->state = HEADL;
					break;
				}
				st->state = BSYNC1;
				st->ind = 0;
				if (i != 2)
					return (i - 2);
				break;
			}
			return 6;
		case BSYNC1:
			if (r != 0x80 + '+')
				st->ind++;
			st->state = BSYNC2;
			return 8;
		case BSYNC2:
			if (r != '*')
				st->ind++;
			st->state = SYN1;
			return 8;
		case SYN1:
			if (r != SYN)
				st->ind++;
			st->state = SYN2;
			return 8;
		case SYN2:
			if (r != SYN)
				st->ind++;
			st->state = SOH1;
			return 8;
		case SOH1:
			if (r != SOH)
				st->ind++;
			if (st->ind > 2) {
				st->state = HEADL;
				break;
			}
			st->state = TXT;
			st->ind = 0;
			st->crc = 0;
      print_header();
			return 8;
		case TXT:
			update_crc(&st->crc, r);
			r = r & 0x7f;
			if (r == 0x03 || r == 0x17) {
				st->state = CRC1;
				return 8;
			}
			st->txt[st->ind] = r;
			st->ind++;
			if (st->ind > 318) {
				st->state = HEADL;
				break;
			}
			return 8;
		case CRC1:
			update_crc(&st->crc, r);
			st->state = CRC2;
			return 8;
		case CRC2:
			update_crc(&st->crc, r);
			st->state = END;
			return 8;
		case END:
			st->state = HEADL;

      //check length to see if it falls within valid range or maybe passed crc, but still bad decode
			if (st->crc == 0 && st->ind<234 && st->ind>22) {
        msg->tot_len = st->ind;
				build_mesg(st->txt, st->ind, msg);
				return 0;
			}
      else {
        init_mesg();
        resetbits(ch);
				return 8;
      }
		}
  } 
  return 8;
}
