/*
 * Copyright (c) 2003 EISLAB, Lulea University of Technology.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwBT Bluetooth stack.
 * 
 * Author: Conny Ohult <conny@sm.luth.se>
 *
 */


#include "lwbt/phybusif.h"
#include "lwbt/lwbt_memp.h"
#include "lwbt/hci.h"
#include "lwbt/l2cap.h"
#include "lwbt/sdp.h"
#include "lwbt/rfcomm.h"
#include "lwip/memp.h"
#include "lwip/mem.h"
#include "lwip/sys.h"
#include "lwip/stats.h"

#include <sys/time.h>
#include <stdlib.h>
#include <unistd.h>


extern void bt_spp_start();
extern void bt_spp_tmr();

int main(int argc, char **argv)
{
	struct phybusif_cb *cb;
	struct timeval tcptv, bttv, now;
	struct timezone tz;
	u8_t btiptmr = 0;

	sys_init();
#ifdef PERF
	perf_init("/tmp/minimal.perf");
#endif /* PERF */
#ifdef STATS
	stats_init();
#endif /* STATS */

	mem_init();
	memp_init();
	pbuf_init();
	printf("mem mgmt initialized\n");


	lwbt_memp_init();
	phybusif_init(argv[1]);
	if(hci_init() != ERR_OK) {
		printf("HCI initialization failed!\n");
		exit(-1);
	}
	l2cap_init();
	sdp_init();
	rfcomm_init();
	printf("Bluetooth initialized.\n");

	bt_spp_start();
	printf("Applications started.\n");

	cb = malloc(sizeof(struct phybusif_cb));
	phybusif_reset(cb);

	gettimeofday(&bttv, &tz); /* Initialize Bluetooth timer (1s) */
	gettimeofday(&tcptv, &tz); /* Initialize TCP timer (TCP_TMR_INTERVAL) */

	while(1) {
		if( phybusif_input(cb) != ERR_OK) /* Check for input */
		{
			exit(-1);
		}

		gettimeofday(&now, &tz); /* Get current time */

		/* Check if TCP timer  should be called */
#if 0
		if((now.tv_sec - tcptv.tv_sec) * 1000000 + (now.tv_usec - tcptv.tv_usec) >= TCP_TMR_INTERVAL * 1000) {
			gettimeofday(&tcptv, &tz); /* Reset TCP timer */
			tcp_tmr();
		}
#endif

		/* Check if Bluetooth and NAT timers should be called */ 
		if((now.tv_sec - bttv.tv_sec) * 1000000 + (now.tv_usec - bttv.tv_usec) >= 1000000) {
			gettimeofday(&bttv, &tz); /* Restart Bluetooth timer */
			l2cap_tmr();
			rfcomm_tmr();
			bt_spp_tmr();
			if(++btiptmr == 5/*sec*/) {

	
			//	bt_ip_tmr();
				btiptmr = 0;
			}
		}
	}

	return 0;
}
