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

/*-----------------------------------------------------------------------------------*/
/* uartif.c
 *
 * Implementation of the HCI UART transport layer for RTXC
 */
/*-----------------------------------------------------------------------------------*/

#include "lwbtopts.h"
#include "phybusif.h"
#include "netif/lwbt/hci.h"
#include "lwip/debug.h"

#include "rtxcapi.h"
#include "csema.h"

#include <iom16c60.h> /* For TX of data. Change this to represent the processor you are using with your RTXC OS */

/* Define this to send one byte on the UART */
#define UART_SEND(x) do {U0TB = x; KS_wait(SIO0OSEM);} while(0)

/*-----------------------------------------------------------------------------------*/
/* 
 * phybusif_init():
 *
 * Initializes the physical bus interface
 */
/*-----------------------------------------------------------------------------------*/
void
phybusif_init(void)
{
}
/*-----------------------------------------------------------------------------------*/
/* 
 * phybusif_reset():
 *
 * Resets the physical bus interface control block to its initial state.
 */
/*-----------------------------------------------------------------------------------*/
err_t
phybusif_reset(struct phybusif_cb *cb) 
{
  /* Init new ctrl block */
  /* Alloc new pbuf. lwIP will handle dealloc */
  if((cb->p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL)) == NULL) {
    DEBUGF(PHYBUSIF_DEBUG, ("phybusif_reset: Could not allocate memory for pbuf\n"));
    return ERR_MEM; /* Could not allocate memory for pbuf */
  }
  cb->q = cb->p; /* Make p the pointer to the head of the pbuf chain and q to the tail */
  
  cb->tot_recvd = 0;
  cb->recvd = 0; 

  cb->bcount = 0; 
  cb->bsize = 0;

  cb->state = W4_PACKET_TYPE;
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * phybusif_reset():
 *
 * Handle incoming DMA data by parsing the HCI header and forwarding it to the HCI.
 */
/*-----------------------------------------------------------------------------------*/
err_t
phybusif_input(struct phybusif_cb *cb) 
{
  u16_t i = 0;
  u8_t n;

  for(i = cb->bcount; i < cb->bsize; i++) {
    switch(cb->state) {
    case W4_PACKET_TYPE:
      switch(cb->dmabuf[i]) {
      case HCI_ACL_DATA_PACKET:
	cb->state = W4_ACL_HDR;
	break;
      case HCI_EVENT_PACKET:
	cb->state = W4_EVENT_HDR;
	break;
      default:
        DEBUGF(PHYBUSIF_DEBUG, ("phybusif_input: Unknown packet type 0x%x\n", cb->dmabuf[i]));
	break;
      }
      break;
    case W4_EVENT_HDR:
      ((u8_t *)cb->q->payload)[cb->recvd] = cb->dmabuf[i];
      cb->tot_recvd++;
      cb->recvd++;
      if(cb->recvd == HCI_EVENT_HDR_LEN) {
	cb->evhdr = cb->p->payload;
	pbuf_header(cb->p, -HCI_EVENT_HDR_LEN);
	cb->recvd = cb->tot_recvd = 0;
	if(cb->evhdr->len > 0) {
	  cb->state = W4_EVENT_PARAM;
	} else {
	  hci_event_input(cb->p); /* Handle incoming event */
	  pbuf_free(cb->p);
	  phybusif_reset(cb);
	  cb->bcount = i + 1;
	  return ERR_OK; /* Since there most likley won't be any more data in the input buffer */
	}
      }
      break;
    case W4_EVENT_PARAM:
      ((u8_t *)cb->q->payload)[cb->recvd] = cb->dmabuf[i];
      cb->tot_recvd++;
      cb->recvd++;
      if(cb->recvd == cb->q->len) { /* Pbuf full. alloc and add new tail to chain */
        cb->recvd = 0;
        if((cb->q = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL)) == NULL) {
	  DEBUGF(PHYBUSIF_DEBUG, ("phybusif_input: Could not allocate memory for event parameter pbuf\n"));
	  return ERR_MEM; /* Could not allocate memory for pbuf */
	}
	pbuf_chain(cb->p, cb->q);
	pbuf_free(cb->q);
      }
      if(cb->tot_recvd == cb->evhdr->len) {
	hci_event_input(cb->p); /* Handle incoming event */
        pbuf_free(cb->p);
	phybusif_reset(cb);
	cb->bcount = i + 1;
        return ERR_OK; /* Since there most likley won't be any more data in the input buffer */
      }
      break;
    case W4_ACL_HDR:
      ((u8_t *)cb->q->payload)[cb->recvd] = cb->dmabuf[i];
      cb->tot_recvd++;
      cb->recvd++;
      if(cb->recvd == HCI_ACL_HDR_LEN) {
	cb->aclhdr = cb->p->payload;
	pbuf_header(cb->p, -HCI_ACL_HDR_LEN);
	cb->recvd = cb->tot_recvd = 0;
	if(cb->aclhdr->len > 0) {
	  cb->state = W4_ACL_DATA;
	} else {
	  DEBUGF(PHYBUSIF_DEBUG, ("phybusif_reset: Forward Empty ACL packet to higher layer\n"));
	  hci_acl_input(cb->p); /* Handle incoming ACL data */
	  phybusif_reset(cb);
	  cb->bcount = i + 1;
	  return ERR_OK; /* Since there most likley won't be any more data in the input buffer */
	}
      }
      break;
    case W4_ACL_DATA:
      ((u8_t *)cb->q->payload)[cb->recvd] = cb->dmabuf[i];
      cb->tot_recvd++;
      cb->recvd++;
      if(cb->recvd == cb->q->len) { /* Pbuf full. alloc and add new tail to chain */
        cb->recvd = 0;
        if((cb->q = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL)) == NULL) {
	  DEBUGF(PHYBUSIF_DEBUG, ("phybusif_input: Could not allocate memory for ACL data pbuf\n"));
	  return ERR_MEM; /* Could not allocate memory for pbuf */
	}
        pbuf_chain(cb->p, cb->q);
	pbuf_free(cb->q);
      }
      if(cb->tot_recvd == cb->aclhdr->len) {
	DEBUGF(PHYBUSIF_DEBUG, ("phybusif_input: Forward ACL packet to higher layer\n"));
	hci_acl_input(cb->p); /* Handle incoming ACL data */
	phybusif_reset(cb);
	cb->bcount = i + 1;
        return ERR_OK; /* Since there most likley won't be any more data in the input buffer */
      }
      break;
    default:
      DEBUGF(PHYBUSIF_DEBUG, ("phybusif_input: Unknown state\n\n"));
      break;
    }
  } /* for */
  /* All data from dma input buffer handled, go wait for more */
  cb->bcount = i;
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * phybusif_output():
 *
 * Called by HCI to output data on the physical bus (UART).
 */
/*-----------------------------------------------------------------------------------*/
void
phybusif_output(struct pbuf *p, unsigned int len) 
{
  static struct pbuf *q;
  static int i;
  static unsigned char *ptr;
  unsigned char c;
 
  /* Send pbuf on UART */
  for(q = p; q != NULL; q = q->next) {
    ptr = q->payload;
    for(i = 0; i < q->len && len; i++) {
      c = *ptr++;
      UART_SEND(c);
      --len;
    }
  }
}
/*-----------------------------------------------------------------------------------*/
