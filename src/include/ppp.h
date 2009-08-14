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

#ifndef __LWBT_PPP_H__
#define __LWBT_PPP_H__

#include "netif/lwbt/rfcomm.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"

struct ppp_pcb;

/* Lower layer interface to PPP: */
void ppp_init(void); /* Must be called first to initialize PPP */
void ppp_tmr(void); /* Must be called every 1s interval */

/* Application program's interface: */
struct ppp_pcb *ppp_new(struct rfcomm_pcb *pcb);
err_t ppp_close(struct ppp_pcb *pcb);
void ppp_reset_all(void);
void ppp_arg(struct ppp_pcb *pcb, void *arg);
void ppp_disconnected(struct ppp_pcb *pcb, err_t (* disconnected)(void *arg, 
								  struct ppp_pcb *pcb,
								  u16_t proto,
								  err_t err));
err_t ppp_listen(struct ppp_pcb *pcb, err_t (* accept)(void *arg, 
						       struct ppp_pcb *pcb, 
						       err_t err));
void ppp_netif(struct ppp_pcb *pcb, struct netif *netif);
err_t ppp_connect(struct ppp_pcb *pcb, err_t (* connected)(void *arg,
							   struct ppp_pcb *tpcb,
							   err_t err));
err_t ppp_disconnect(struct ppp_pcb *pcb);
err_t ppp_echo(struct ppp_pcb *pcb, err_t (* echo_rsp)(void *arg,
						       struct ppp_pcb *tpcb,
						       err_t err));
err_t ppp_netif_output(struct netif *netif, struct pbuf *p, struct ip_addr *ipaddr);
err_t ppp_data_output(struct ppp_pcb *pcb, struct pbuf *q);
err_t ppp_input(void *arg, struct rfcomm_pcb *rfcommpcb, struct pbuf *p, err_t err);

err_t ppp_lp_disconnected(struct rfcomm_pcb *rfcommpcb);

/* Framing flags */
#define PPP_END 0x7E
#define PPP_ESC 0x7D
#define PPP_ESC_END 0x5E
#define PPP_ESC_ESC 0x5D

#define PPP_ADDRCTRL 0xFF03

/* Protocol */
#define PPP_LCP 0xC021
#define PPP_PAP 0xC023
#define PPP_IPCP 0x8021
#define PPP_IP_DATA 0x0021

/* Link control protocol fields */
#define LCP_CFG_REQ 1
#define LCP_CFG_ACK 2
#define LCP_CFG_NAK 3
#define LCP_CFG_REJ 4
#define LCP_TERM_REQ 5
#define LCP_TERM_ACK 6
#define LCP_CODE_REJ 7
#define LCP_PROTO_REJ 8
#define LCP_ECHO_REQ 9
#define LCP_ECHO_RSP 10
#define LCP_DISCARD_REQ 11

/* Link control protocol option types */
#define LCP_CFG_MRU 1
#define LCP_CFG_ACCM 2
#define LCP_CFG_AUTH 3
#define LCP_CFG_QUAL 4
#define LCP_CFG_MAGIC 5
#define LCP_CFG_P_COMP 7
#define LCP_CFG_AC_COMP 8

/* IP control protocol fields */
#define IPCP_CFG_REQ 1
#define IPCP_CFG_ACK 2
#define IPCP_CFG_NAK 3
#define IPCP_CFG_REJ 4
#define IPCP_TERM_REQ 5
#define IPCP_TERM_ACK 6
#define IPCP_CODE_REJ 7

/* IP control protocol option types */
#define IPCP_CFG_COMP 2
#define IPCP_CFG_IPADDR 3
#define IPCP_CFG_PDNS 129
#define IPCP_CFG_PNBNS 130
#define IPCP_CFG_SDNS 131
#define IPCP_CFG_SNBNS 132

/* PPP Configuration parameter masks */
#define PPP_IR 0x01

/* LCP Configuration option masks */
#define LCP_CFG_OPT_MRU 0x01
#define LCP_CFG_OPT_PAP 0x02
#define LCP_CFG_OPT_PFC 0x04
#define LCP_CFG_OPT_ACFC 0x08
#define LCP_CFG_OPT_ACCM 0x10

/* LCP Configuration parameter masks */
#define LCP_CFG_IN_ACK 0x01
#define LCP_CFG_OUT_ACK 0x02
#define LCP_CFG_PAP 0x04
#define LCP_CFG_OUT_PCOMP 0x08
#define LCP_CFG_IN_PCOMP 0x10
#define LCP_CFG_OUT_ACCOMP 0x20
#define LCP_CFG_IN_ACCOMP 0x40
#define LCP_CFG_OUT_ACCM 0x80

/* LCP Configuration parameter 2 masks */
#define LCP_CFG_OUT_REQ 0x01

/* IPCP Configuration option masks */
#define IPCP_CFG_OPT_VJ 0x01
#define IPCP_CFG_OPT_IP 0x02
#define IPCP_CFG_OPT_PDNS 0x04
#define IPCP_CFG_OPT_NBNS 0x08
#define IPCP_CFG_OPT_SDNS 0x10
#define IPCP_CFG_OPT_SNBNS 0x20

/* IPCP Configuration parameter masks */
#define IPCP_CFG_IN_ACK 0x01
#define IPCP_CFG_OUT_ACK 0x02
#define IPCP_CFG_IN_VJ 0x04
#define IPCP_CFG_OUT_VJ 0x08
#define IPCP_CFG_OUT_REQ 0x10

/* Header lengths */
#define PPP_HDR_LEN 4
#define PPP_CPHDR_LEN 4
#define PPP_CFGHDR_LEN 2

/* Default LCP options */
#define LCP_DEFAULT_MRU 1500
#define LCP_DEFAULT_ACCM 0xFFFFFFFF

enum ppp_state {
  PPP_LCP_CLOSED, PPP_LCP_LISTEN, PPP_LCP_CFG, PPP_LCP_OPEN, PPP_IPCP_CFG, PPP_IPCP_OPEN, 
  PPP_LCP_CLOSING
};

struct ppp_cp_hdr {
  u8_t code;
  u8_t id;
  u16_t len;
};

struct ppp_cfg_hdr {
  u8_t type;
  u8_t len;
};

/* This structure is used to represent Configure and Terminate requests for LCP and IPCP. */
struct ppp_req {
  struct ppp_req *next;    /* for the linked list, used when putting signals
			      on a queue */
  struct pbuf *p;          /* buffer containing data + CP header */
  u8_t id; /* Identification */
  u8_t rto;  /* response timeout */
  u8_t nrtx; /* number of retransmissions */
  u16_t proto; /* Control protocol */
};

struct ppp_pcb {
  struct ppp_pcb *next; /* For the linked list */

  struct rfcomm_pcb *rfcommpcb; /* RFCOMM connection */

  enum ppp_state state; /* PPP state */
  
  struct pbuf *p; /* Incoming PPP packet chain */
  struct pbuf *q; /* Last packet in incoming PPP packet chain */
  u16_t psize; /* Incoming PPP packet size */
  u16_t qsize; /* Size of last packet in chain */

  struct ppp_req *reqs;  /* List of sent but unacked configure or terminate requests */
  u8_t naks; /* Number of received NAKs */
  /* Link control protocol configuration parameters */
  u16_t mru; /* Maximum receive unit */
  u32_t outaccm; /* The Asynchronous Character Control Map of outgoing data */

  /* PPP configuration parameters */
  u8_t pppcfg; /* Bit 0 indicates if we are the initiator of this connection
		*/
		   
  /* LCP configuration options */
  u8_t lcpopt; /* Bit 0 indicates Maximum Receive Unit (MRU) option
		* Bit 1 indicates Password Authentication Protocol (PAP) option
		* Bit 2 indicates Protocol Field Compression (PFC) option
		* Bit 3 indicates Address and Control Field Compression (ACFC) option
		* Bit 4 indicates Asynchronous Character Control Map (ACCM) option
		*/

  /* LCP configuration parameters */
  u8_t lcpcfg; /* Bit 0 indicates that a LCP config ack has been received
		* Bit 1 indicates that a LCP config ack has been sent
		* Bit 2 indicates if PAP must be used for authentication
		* Bit 3 indicates if the protocol field must be compressed in outgoing 
		* packets
		* Bit 4 indicates if the protocol field is compressed in incoming packets
		* Bit 5 indicates if the address and control field must be compressed in 
		* outgoing packets
		* Bit 6 indicates if the address and control field is compressed in incoming 
		* packets
		* Bit 7 indicates if the ACCM for outgoing information has been configured
		*/
  u8_t lcpcfg2; /* Bit 0 indicates that an initial LCP config request has been sent */

  /* IPCP configuration options */
  u8_t ipcpopt; /* Bit 1 indicates VJ IP address compression protocol option
		 * Bit 2 indicates IP address option
		 */

  /* IPCP configuration parameters */
  u8_t ipcpcfg; /* Bit 1 indicates if an IPCP config ack has been received
		 * Bit 2 indicates if an IPCP config ack has been sent
		 * Bit 3 indicates if VJ compressed TCP/IP is used on outgoing packets
		 * Bit 4 indicates if VJ compressed TCP/IP is used on incoming packets
		 * Bit 5 indicates if an initial IPCP config request has been sent
		 */

  struct netif *bluetoothif;

  void *callback_arg;

  /* Function to be called when a connection has been set up or when a listener has been
     connected */
  err_t (* connected)(void *arg, struct ppp_pcb *pcb, err_t err);
  /* Function to be called when a connection has been terminated */
  err_t (* disconnected)(void *arg, struct ppp_pcb *pcb, u16_t proto, err_t err);
  /* Function to be called when an echo response has been received */
  err_t (* echo_rsp)(void *arg, struct ppp_pcb *tpcb, err_t err);
};

#define PPP_EVENT_CONNECTED(pcb,err,ret) \
                           if((pcb)->connected != NULL) \
                           (ret = (pcb)->connected((pcb)->callback_arg,(pcb),(err)))
#define PPP_EVENT_DISCONNECTED(pcb,err,proto,ret) \
                               if((pcb)->disconnected != NULL) { \
                               (ret = (pcb)->disconnected((pcb)->callback_arg,(pcb),(proto),(err))); \
                               } else { \
                               netif_remove(pcb->bluetoothif); \
                               ppp_close(pcb); \
                               }
#define PPP_EVENT_ECHO_RSP(pcb,err,ret) \
                               if((pcb)->echo_rsp != NULL) \
                               (ret = (pcb)->echo_rsp((pcb)->callback_arg,(pcb),(err)))
#define PPP_EVENT_RECV(pcb,ret) \
                       if((pcb)->bluetoothif != NULL) { \
                       (ret = (pcb)->bluetoothif->input((pcb)->p,(pcb)->bluetoothif)); \
		       } else { \
		       (pbuf_free((pcb)->p)); \
		       }

extern struct ppp_pcb *ppp_listen_pcbs; /* List of all PPP PCBs in listening for a 
					   connection */
extern struct ppp_pcb *ppp_active_pcbs; /* List of all active PPP PCBs */

extern struct ppp_pcb *ppp_tmp_pcb;     /* Only used for temporary storage. */

/* Define two macros, PPP_REG and PPP_RMV that registers a PPP PCB
   with a PCB list or removes a PCB from a list, respectively. */

#define PPP_REG(pcbs, npcb) do { \
                            npcb->next = *pcbs; \
                            *pcbs = npcb; \
                            } while(0)
#define PPP_RMV(pcbs, npcb) do { \
                            if(*pcbs == npcb) { \
                               *pcbs = (*pcbs)->next; \
                            } else for(ppp_tmp_pcb = *pcbs; ppp_tmp_pcb != NULL; ppp_tmp_pcb = ppp_tmp_pcb->next) { \
                               if(ppp_tmp_pcb->next != NULL && ppp_tmp_pcb->next == npcb) { \
                                  ppp_tmp_pcb->next = npcb->next; \
                                  break; \
                               } \
                            } \
                            npcb->next = NULL; \
                            } while(0)

/* The PPP req list macros */
extern struct ppp_req *ppp_tmp_req;     /* Only used for temporary storage. */

#define PPP_REQ_REG(reqs, nreq) do { \
                                nreq->next = *reqs; \
                                *reqs = nreq; \
                                } while(0)
#define PPP_REQ_RMV(reqs, nreq) do { \
                                if(*reqs == nreq) { \
                                   *reqs = (*reqs)->next; \
                                } else for(ppp_tmp_req = *reqs; ppp_tmp_req != NULL; ppp_tmp_req = ppp_tmp_req->next) { \
                                   if(ppp_tmp_req->next != NULL && ppp_tmp_req->next == nreq) { \
                                      ppp_tmp_req->next = nreq->next; \
                                      break; \
                                   } \
                                } \
                                nreq->next = NULL; \
                                } while(0)
#endif /* __LWBT_PPP_H__ */
