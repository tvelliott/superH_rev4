
//
//t.elliott strip down code to essentials and mods to use it on an embedded system
//

/*
 *  Copyright (c) 2007 by Thierry Leconte (F4DWV)
 *
 *      $Id: main.c,v 1.5 2007/04/22 16:14:41 f4dwv Exp $
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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>

#include "acarsdec.h"
#include "agc.h"

static long rx_idx;
static int c;
static msg_t msgl;
static unsigned char rl;
static int nbitl;
static int nrbitl;
static int nbch=0;
static short port=0;
static int acars_init = 0;
static int ind;

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
void print_header(void) {
	printf("\r\n\r\nRX_IDX: %ld, rssi %3.0f dBm", rx_idx, get_rssi());
}

/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
void print_mesg(msg_t * msg)
{
	printf("\r\nACARS mesg len: %d, ", msg->tot_len);
	printf("\r\nACARS mode: %c, ", msg->mode);
	printf("\r\nmessage label: %s", msg->label);

	printf("\r\nAircraft reg: %s, ", msg->addr);
	printf("\r\nflight id: %s", msg->fid);

	printf("\r\nBlock id: %d, ", (int) msg->bid);
	printf("\r\nmsg. no: %s", msg->no);
	printf("\r\nMessage content:-\r\n%s", msg->txt);

  if( msg->tot_len==23 ) printf(" (ping. no message.)");

	rx_idx++;

}

////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
int acars_decode(int16_t *sample, int len) {

  if(len>730) return 0; //something is wrong; 1460 max / 2

  if(!acars_init) {
    init_bits();
    init_mesg();

    nbitl = 0;
    nrbitl = 8;
    acars_init=1;
  }

  for (ind = 0; ind < len;) {
    nbitl += getbit(sample[ind++], &rl, 0);
    if (nbitl >= nrbitl) {
      nrbitl = getmesg(rl, &msgl, 0);
      nbitl = 0;
      if (nrbitl == 0) {
        print_mesg(&msgl);
        nrbitl = 8;
        init_mesg();
        resetbits(0);
      }
    }
  }

}
