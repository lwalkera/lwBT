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

#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/sys.h"

#include "lwip/stats.h"

#include "lwip/ip.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"

#include "phybusif.h"
#include "netif/lwbt/lwbt_memp.h"
#include "netif/lwbt/hci.h"
#include "netif/lwbt/l2cap.h"
#include "netif/lwbt/sdp.h"
#include "netif/lwbt/rfcomm.h"
#include "netif/lwbt/ppp.h"
#include "netif/lwbt/nat.h"

#include "apps/httpd.h"

#include <sys/time.h>
#include <unistd.h>



void bt_ip_start(void *state);
void bt_ip_tmr(void);
/*-----------------------------------------------------------------------------------*/
int
main(int argc, char **argv)
{
  struct phybusif_cb *cb;
  struct timeval tcptv, bttv, now;
  struct timezone tz;
  u8_t btiptmr = 0;
  
#ifdef PERF
  perf_init("/tmp/minimal.perf");
#endif /* PERF */
#ifdef STATS
  stats_init();
#endif /* STATS */

  mem_init();
  memp_init();
  pbuf_init(); 
  netif_init();
  ip_init();
  //udp_init();
  tcp_init();
  printf("TCP/IP initialized.\n");
  lwbt_memp_init();
  phybusif_init();
  if(hci_init() != ERR_OK) {
    printf("HCI initialization failed!");
    exit(-1);
  }
  l2cap_init();
  sdp_init();
  rfcomm_init();
  ppp_init();
  printf("Bluetooth initialized.\n");

  //echo_init();
  httpd_init();
  printf("Applications started.\n");

  cb = malloc(sizeof(struct phybusif_cb));
  phybusif_reset(cb);

  gettimeofday(&bttv, &tz); /* Initialize Bluetooth timer (1s) */
  gettimeofday(&tcptv, &tz); /* Initialize TCP timer (TCP_TMR_INTERVAL) */

  /* Host controller initialization for DTs according to LAN access point (LAP) and dial up networking (DUN) profile */
  bt_ip_start(NULL);

  while(1) {
    phybusif_input(cb); /* Check for input */

    gettimeofday(&now, &tz); /* Get current time */

    /* Check if TCP timer  should be called */
    if((now.tv_sec - tcptv.tv_sec) * 1000000 + (now.tv_usec - tcptv.tv_usec) >= TCP_TMR_INTERVAL * 1000) {
      gettimeofday(&tcptv, &tz); /* Reset TCP timer */
      tcp_tmr();
    }

    /* Check if Bluetooth and NAT timers should be called */ 
    if((now.tv_sec - bttv.tv_sec) * 1000000 + (now.tv_usec - bttv.tv_usec) >= 1000000) {
      gettimeofday(&bttv, &tz); /* Restart Bluetooth timer */
      l2cap_tmr();
      rfcomm_tmr();
      ppp_tmr();
      nat_tmr();
      if(++btiptmr == 240) { /* Akes server special */
	bt_ip_tmr();
	btiptmr = 0;
      }
    }
  }
  
  return 0;
}
/*-----------------------------------------------------------------------------------*/








