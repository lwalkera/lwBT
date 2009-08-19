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

#include "apps/httpd.h"

#include "rtxcapi.h"
#include "cclock.h"
#include "csema.h"

#define SELFTASK ((TASK)0)
#include <iom16c60.h> /* Change this to represent the processor you are using with your RTXC OS */

/* Forward declarations */
u8_t *get_dm0ichars(void);
void bt_ip_start(void);
void bt_ip_tmr(void);
/*-----------------------------------------------------------------------------------*/
void 
dma_input(struct phybusif_cb *cb, u16_t dma0bsz) {
  if(KS_inqsema(DM0SEM) == SEMA_DONE) {
    KS_wait(DM0SEM);
    cb->bsize = dma0bsz;
    phybusif_input(cb);
    cb->bcount = 0;
    return;
  }
  
  cb->bsize = dma0bsz - (TCR0 + 1);
  if(cb->bsize != cb->bcount && KS_inqsema(DM0SEM) != SEMA_DONE) { /* Check DMA buffer for any new data */  
    phybusif_input(cb);
    return;
  }
}
/*-----------------------------------------------------------------------------------*/
void 
maintask(void)
{
  struct phybusif_cb *cb;
  CLKBLK ks_clk *tmr;
  u8_t bt_timer = 0;
  u16_t http_timer = 0;
  u16_t dma0bsz = TCR0+1; /* DMA 0 Buf size */
  u8_t bt_ip_timer = 0;

  mem_init();
  memp_init();
  pbuf_init(); 
  netif_init();
  ip_init();
  tcp_init();
  sio_print("TCP/IP initialized.\n");
  
  lwbt_memp_init();
  phybusif_init();
  if(hci_init() != ERR_OK) {
    sio_print("HCI initialization failed!");
  }
  l2cap_init();
  sdp_init();
  rfcomm_init();
  ppp_init();
  
  sio_print("Bluetooth initialized.\n");

  httpd_init();
  sio_print("Applications started.\n");

  cb = mem_malloc(sizeof(struct phybusif_cb));
  cb->dmabuf = get_dm0ichars();
  phybusif_reset(cb);
  
  tmr = KS_alloc_timer();
  if(tmr == 0) {
    sio_print("tmr==0!\n");
  }
  KS_start_timer(tmr, (TICKS)0, (TICKS)100/CLKTICK, TIMERSEM); /* TCP timer ticks every 100ms */

  /* Reset Bluetooth module */
  PD7.0 = 1; /* Enable output */
  sio_print("Reseting BT module\n");
  P7.0 = 1; /* Stop reset */
  
  KS_delay(SELFTASK,(TICKS)4000/CLKTICK); /* Wait for bluetooth module to init */

  /* Control application initialisation */
  bt_ip_start();

  while(1) {
    dma_input(cb, dma0bsz); /* Check for input */

    /* Handle timers */
    if(KS_inqsema(TIMERSEM) == SEMA_DONE) {
      KS_wait(TIMERSEM);
      tcp_tmr();
      ++bt_timer;
      if(bt_timer == 10) {
        l2cap_tmr();
        rfcomm_tmr();
        ppp_tmr();
        bt_timer = 0;
        ++bt_ip_timer;
        if(bt_ip_timer == 240) {
          bt_ip_tmr();
          bt_ip_timer = 0;
        } 
      }
      
    }
  }
}
/*-----------------------------------------------------------------------------------*/








