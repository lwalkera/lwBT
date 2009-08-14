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
/* ppp.c
 *
 * Implementation of the PPP protocol.
 * Used to carry packets from the higher IP layer across the RFCOMM serial port 
 * emulation layer.
 */
/*-----------------------------------------------------------------------------------*/

#include "netif/lwbt/rfcomm.h"
#include "netif/lwbt/ppp.h"
#include "netif/lwbt/lwbt_memp.h"
#include "netif/lwbt/fcs.h"
#include "lwbtopts.h"
#include "lwip/debug.h"

struct ppp_pcb *ppp_listen_pcbs;  /* List of all PPP PCBs listening for a connection */
struct ppp_pcb *ppp_active_pcbs;  /* List of all active PPP PCBs */
struct ppp_pcb *ppp_tmp_pcb;
struct ppp_req *ppp_tmp_req;

u8_t id_nxt; /* Next Identifier to be sent */

/* Forward declarations */
err_t ppp_output(struct ppp_pcb *pcb, struct pbuf *p);
err_t ppp_cp_output(struct ppp_pcb *pcb, u16_t proto, u8_t code, u8_t id, struct pbuf *q);
u8_t ppp_next_id(void);
struct pbuf *lcp_cfg_req(struct ppp_pcb *pcb, struct pbuf *options);
err_t ipcp_cfg_req(struct ppp_pcb *pcb);
void ppp_pbuf_ref_chain(struct pbuf *p);

/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_init():
 *
 * Initializes the PPP layer.
 */
/*-----------------------------------------------------------------------------------*/
void
ppp_init(void)
{
  /* Clear globals */
  ppp_active_pcbs = NULL;
  ppp_tmp_pcb = NULL;

  /* Initialize the lcp and ipcp identifier */
  id_nxt = 0;
}
/*-----------------------------------------------------------------------------------*/
/*
 * ppp_tmr():
 *
 * Called every 1s and implements the command timer that
 * removes a DLC if it has been waiting for a response enough
 * time.
 */
/*-----------------------------------------------------------------------------------*/
void
ppp_tmr(void)
{
  struct ppp_pcb *pcb;
  struct ppp_req *req;
  err_t ret;

  /* Step through all of the active pcbs */
  for(pcb = ppp_active_pcbs; pcb != NULL; pcb = pcb->next) {
    /* Step through any unresponded requests */
    for(req = pcb->reqs; req != NULL; req = req->next) {
      --req->rto; /* Adjust rto timer */
      LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_tmr: req->rto = %d\n", req->rto));
      /* Check if restart timer has expired */
      if(req->rto == 0) {
	/* Check if max number of retransmissions have been reached */
	if(req->nrtx == 0) {
	  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_tmr: Max number of retransmissions have been reached\n"));
	  PPP_REQ_RMV(&(pcb->reqs), req);
	  pbuf_free(req->p);
	  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_tmr: Free memory for request with ID 0x%x***********\n", req->id));
	  lwbt_memp_free(MEMP_PPP_REQ, req);
	  pcb->state = PPP_LCP_CLOSED;
	  PPP_EVENT_DISCONNECTED(pcb,ERR_CLSD,req->proto,ret);
	} else {
	  --req->nrtx;
	  /* Retransmitt request with timeout doubled. It may not exceed the configured 
	     timeout value */
	  req->rto = PPP_RTO/(req->nrtx + 1);

	  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_tmr: Retransmitt\n"));
	  ppp_output(pcb, req->p);
	}
      }
    }
  }
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_lp_disconnected():
 *
 * Called by the application to indicate that the lower protocol disconnected.
 */
/*-----------------------------------------------------------------------------------*/
err_t
ppp_lp_disconnected(struct rfcomm_pcb *rfcommpcb)
{
  struct ppp_pcb *pcb;
  err_t ret = ERR_OK;

  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_lp_disconnected\n"));

  for(pcb = ppp_active_pcbs; pcb != NULL; pcb = pcb->next) {
    if(pcb->rfcommpcb == rfcommpcb) {
      pcb->state = PPP_LCP_CLOSED;
      PPP_EVENT_DISCONNECTED(pcb, ERR_OK, PPP_LCP, ret);
      return ret; /* Since there should be only one PPP connection */
    }
  }

  for(pcb = ppp_listen_pcbs; pcb != NULL; pcb = pcb->next) {
    if(pcb->rfcommpcb == rfcommpcb) {
      pcb->state = PPP_LCP_CLOSED;
      PPP_EVENT_DISCONNECTED(pcb, ERR_OK, PPP_LCP, ret);
      return ret; /* Since there should be only one PPP connection */
    }
  }

  return ret;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_process_lcp():
 *
 * Parses the received LCP packet and handles it.
 */
/*-----------------------------------------------------------------------------------*/
void
ppp_process_lcp(struct ppp_pcb *pcb, struct pbuf *p)
{
  struct ppp_cp_hdr *cphdr;
  struct ppp_cfg_hdr *cfghdr;
  struct ppp_req *req;
  struct pbuf *r, *s;
  u8_t rspstate = LCP_CFG_ACK;
  u16_t len;
  err_t ret;

  cphdr = p->payload;
  pbuf_header(p, -PPP_CPHDR_LEN);
  cphdr->len = ntohs(cphdr->len);
  len = cphdr->len - PPP_CPHDR_LEN;
  
  if(cphdr->code == LCP_CFG_ACK || cphdr->code == LCP_CFG_NAK || 
     cphdr->code == LCP_CFG_REJ || cphdr->code == LCP_CODE_REJ || 
     cphdr->code == LCP_TERM_ACK) {
    for(req = pcb->reqs; req != NULL; req = req->next) {
      /* Remove any matching request */
      if(cphdr->id == req->id) {
	PPP_REQ_RMV(&(pcb->reqs), req);
	pbuf_free(req->p);
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: Free memory for request with id 0x%x***********\n", req->id));
	lwbt_memp_free(MEMP_PPP_REQ, req);
      }
    }
  }

 LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: cphdr->code = 0x%x\n", cphdr->code));
 /*
   {
   struct pbuf *q;
   for(q = p; q != NULL; q = q->next) {
   u16_t i;
    for(i = 0; i < q->len; ++i) {
      LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: 0x%x\n", ((u8_t *)q->payload)[i]));
    }
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: q->len == %d q->tot_len == %d\n", q->len, q->tot_len));
  }
}
 */
  r = NULL;  
  switch(cphdr->code) {
  case LCP_CFG_REQ:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_CFG_REQ\n"));
    if(pcb->state == PPP_LCP_CLOSED) {
      /* A terminate-ack is sent to indicate that we are in a closed state */
      ppp_cp_output(pcb, PPP_LCP, LCP_TERM_ACK, ppp_next_id(), NULL);
      break;
    } else if(pcb->state == PPP_LCP_LISTEN) {
      /* If pcb in LISTEN state we move it to the active list */
      PPP_RMV(&(ppp_listen_pcbs), pcb);
      PPP_REG(&ppp_active_pcbs, pcb);
    }
    pcb->state = PPP_LCP_CFG;

    while(len > 0) { 
      cfghdr = p->payload;

      switch(cfghdr->type) {
      case LCP_CFG_MRU:
	/* ACK - Parameter accepted */
	pcb->mru = ntohs(((u16_t *)p->payload)[1]);
	if(rspstate == LCP_CFG_ACK) {
	  s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	  memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	  if(r == NULL) {
	    r = s;
	  } else {
	    pbuf_chain(r, s);
	    pbuf_free(s);
	  }
	} /* if rspstate == LCP_CFG_NAK or LCP_CFG_REJ do not add packet to outgoing pbuf */
      case LCP_CFG_ACCM:
	/* ACK - Parameter accepted */
	/* Since we will handle all characters that are escaped we do not need to save the 
	   ACCM for incoming data */
	if(rspstate == LCP_CFG_ACK) {
	  s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	  memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	  if(r == NULL) {
	    r = s;
	  } else {
	    pbuf_chain(r, s);
	    pbuf_free(s);
	  }
	} /* if rspstate == LCP_CFG_NAK or LCP_CFG_REJ do not add packet to outgoing pbuf */
	break;
#if PPP_AUTH
      case LCP_CFG_AUTH:
	//TODO: NOT IMPLEMENTDED
	break;
#endif
#if PPP_PHDR_COMP
      case LCP_CFG_P_COMP:
	pcb->lcpcfg |= LCP_CFG_IN_PCOMP;
	if(rspstate == LCP_CFG_ACK) {
	  s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	  memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	  if(r == NULL) {
	    r = s;
	  } else {
	    pbuf_chain(r, s);
	    pbuf_free(s);
	  }
	}
	break;
#endif /* PPP_PHDR_COMP */
#if PPP_ACHDR_COMP
      case LCP_CFG_AC_COMP:
	pcb->lcpcfg |= LCP_CFG_IN_ACCOMP;
	if(rspstate == LCP_CFG_ACK) {
	  s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	  memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	  if(r == NULL) {
	    r = s;
	  } else {
	    pbuf_chain(r, s);
	    pbuf_free(s);
	  }
	}
	break;
#endif /* PPP_ACHDR_COMP */
      default:
	/* Reject parameter */
	if(rspstate != LCP_CFG_REJ) {
	  rspstate = LCP_CFG_REJ;
	  if(r != NULL) {
	    pbuf_free(r);
	    r = NULL;
	  }
	}
	s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	if(r == NULL) {
	  r = s;
	} else {
	  pbuf_chain(r, s);
	  pbuf_free(s);
	}
	break;
      } /* switch */
      //pbuf_header(p, -(cfghdr->len - PPP_CFGHDR_LEN));
      //len -= cfghdr->len - PPP_CFGHDR_LEN;
      pbuf_header(p, -(cfghdr->len));
      len -= cfghdr->len;
    } /* while */

    if(!(pcb->pppcfg & PPP_IR) && !(pcb->lcpcfg2 & LCP_CFG_OUT_REQ)) {
      /* Send a LCP configure request for outgoing link if it hasnt been configured */
      p = lcp_cfg_req(pcb, NULL);
      ppp_cp_output(pcb, PPP_LCP, LCP_CFG_REQ, ppp_next_id(), p);
      pcb->lcpcfg2 |= LCP_CFG_OUT_REQ;
    }
    /* Send response to configuration request */
    ppp_cp_output(pcb, PPP_LCP, rspstate, cphdr->id, r);

    if(rspstate == LCP_CFG_ACK) {
      pcb->lcpcfg |= LCP_CFG_OUT_ACK;
      /* LCP connection established if a configuration a ack has been received */
      if(pcb->lcpcfg & LCP_CFG_IN_ACK) {
	if(pcb->lcpcfg & LCP_CFG_PAP && pcb->pppcfg & PPP_IR) {
	  //TODO: CALL AUTHENTICATE FUNCTION IN UPPER LAYER 
	  /* Authenticate if we are the initiator */
	} else {
	  pcb->state = PPP_IPCP_CFG;
	  if(pcb->pppcfg & PPP_IR) { /* Configure IPCP connection if we are the initiator */
	    if(ipcp_cfg_req(pcb) != ERR_OK) {
	      LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: ipcp config request failed\n"));
	      ppp_disconnect(pcb);
	    }  
	  }
	}
      }
    }
    break;
  case LCP_CFG_ACK:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_CFG_ACK\n"));
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN) {
      /* A terminate-ack is sent to indicate that we are in a closed state */
      ppp_cp_output(pcb, PPP_LCP, LCP_TERM_ACK, ppp_next_id(), NULL);
      break;
    }

    pcb->lcpcfg |= LCP_CFG_IN_ACK;
    pcb->naks = 0;
    while(len > 0) {
      cfghdr = p->payload;
      //pbuf_header(p, -PPP_CFGHDR_LEN);
      //len -= PPP_CFGHDR_LEN;
      switch(cfghdr->type) {
      case LCP_CFG_MRU:
	/* Maximum receive unit that the implementation can receive. Doesnt need to be stored */
	break;
      case LCP_CFG_ACCM:
	pcb->outaccm = ntohl(*((u32_t *)(((u16_t *)p->payload) + 1)));
	break;
#if PPP_PHDR_COMP
      case LCP_CFG_P_COMP: 
	pcb->lcpcfg |= LCP_CFG_OUT_PCOMP;
	break;
#endif /* PPP_PHDR_COMP */
#if PPP_ACHDR_COMP
      case LCP_CFG_AC_COMP:
	pcb->lcpcfg |= LCP_CFG_OUT_ACCOMP;
	break;
#endif /* PPP_ACHDR_COMP */
      default:
	/* Silently discard configuration option */
	break;
      } /* switch */
      pbuf_header(p, -cfghdr->len);
      len -= cfghdr->len;
    } /* while */
    /* LCP connection established if a configuration a ack has been sent */
    if(pcb->lcpcfg & LCP_CFG_OUT_ACK) {
      /* LCP connection established */
      pcb->state = PPP_IPCP_CFG;
      if(pcb->pppcfg & PPP_IR) { /* Configure IPCP connection if we are the initiator */
	if(ipcp_cfg_req(pcb) != ERR_OK) {
	  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: ipcp config request failed\n"));
	  ppp_disconnect(pcb);
	} 
      } 
    }
    break;
  case LCP_CFG_NAK:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_CFG_NAK\n"));
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN) {
      /* A terminate-ack is sent to indicate that we are in a closed state */
      ppp_cp_output(pcb, PPP_LCP, LCP_TERM_ACK, ppp_next_id(), NULL);
      break;
    }
    ++pcb->naks;
    if(pcb->naks == PPP_MAX_FAILURE) {
      pcb->state = PPP_LCP_CLOSED;
      PPP_EVENT_CONNECTED(pcb, ERR_CONN, ret);
      break;
    }
    while(len > 0) {
      cfghdr = p->payload;
      //pbuf_header(p, -PPP_CFGHDR_LEN);
      //len -= PPP_CFGHDR_LEN;
      switch(cfghdr->type) {
      case LCP_CFG_MRU:
	/* Maximum receive unit that the implementation can receive is not accepted */
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: NAK, MRU = %d\n", ntohs(((u16_t *)p->payload)[1])));
	if(PPP_IN_MRU > ntohs(((u16_t *)p->payload)[1])) {
	  pcb->mru = ntohs(((u16_t *)p->payload)[1]);
	} else {
	  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: MRU negotiation failed\n"));
	  ppp_disconnect(pcb); /* MRU negotiation failed */
	  return;
	}
	break;
      case LCP_CFG_ACCM:
	pcb->outaccm = ntohl(*((u32_t *)(((u16_t *)p->payload) + 1)));
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: NAK, ACCM\n"));
	break;
      case LCP_CFG_AUTH:
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: NAK, AUTH\n"));
	/* Check if authentication option was in request */
	if(pcb->lcpopt & LCP_CFG_OPT_PAP) {
	  //TODO: Remote PPP uses an authentication protocol that we do not understand. What to do???
	} else { /* Configuration option is requested to be in next request */
	  pcb->lcpopt |= LCP_CFG_OPT_PAP;
	}
	break;
#if PPP_PHDR_COMP
      case LCP_CFG_P_COMP:
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: NAK, P_COMP\n"));
	pcb->lcpopt &= ~LCP_CFG_OPT_PFC;
	break;
#endif /* PPP_PHDR_COMP */
#if PPP_ACHDR_COMP
      case LCP_CFG_AC_COMP:
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: NAK, AC_COMP\n"));
	pcb->lcpopt &= ~LCP_CFG_OPT_ACFC;
	break;
#endif /* PPP_ACHDR_COMP */
      default:
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: NAK, ?\n"));
	/* Silently discard configuration option */
	break;
      } /* switch */
      pbuf_header(p, -cfghdr->len);
      len -= cfghdr->len;
    } /* while */
    s = lcp_cfg_req(pcb, r);
    ppp_cp_output(pcb, PPP_LCP, LCP_CFG_REQ, ppp_next_id(), s);
    break;
  case LCP_CFG_REJ:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_CFG_REJ\n"));
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN) {
      /* A terminate-ack is sent to indicate that we are in a closed state */
      ppp_cp_output(pcb, PPP_LCP, LCP_TERM_ACK, ppp_next_id(), NULL);
      break;
    }
    while(len > 0) {
      cfghdr = p->payload;
      //pbuf_header(p, -PPP_CFGHDR_LEN);
      //len -= PPP_CFGHDR_LEN;
      switch(cfghdr->type) {
      case LCP_CFG_MRU:
	pcb->lcpopt &= ~LCP_CFG_OPT_MRU;
	break;
      case LCP_CFG_ACCM:
	pcb->lcpopt &= ~LCP_CFG_OPT_ACCM;
	break;
      case LCP_CFG_AUTH:
	pcb->lcpopt &= ~LCP_CFG_OPT_PAP;
	break;
#if PPP_PHDR_COMP
      case LCP_CFG_P_COMP:  
	pcb->lcpopt &= ~LCP_CFG_OPT_PFC;
	break;
#endif /* PPP_PHDR_COMP */
#if PPP_ACHDR_COMP
      case LCP_CFG_AC_COMP:
	pcb->lcpopt &= ~LCP_CFG_OPT_ACFC;
	break;
#endif /* PPP_ACHDR_COMP */
      default:
	break;
      } /* switch */
      pbuf_header(p, -cfghdr->len);
      len -= cfghdr->len;
    } /* while */
    s = lcp_cfg_req(pcb, NULL);
    ppp_cp_output(pcb, PPP_LCP, LCP_CFG_REQ, ppp_next_id(), s);
    break;
  case LCP_TERM_REQ:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_TERM_REQ\n"));
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: Connection terminate request\n"));
    ppp_pbuf_ref_chain(p); /* Increase reference count so that ppp_cp_output don't free it */
    ppp_cp_output(pcb, PPP_LCP, LCP_TERM_ACK, cphdr->id, p);   
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN) {
      /* We are already closed */
      break;
    }
    pcb->state = PPP_LCP_CLOSED;
    PPP_EVENT_DISCONNECTED(pcb, ERR_OK, PPP_LCP, ret);
    break;
  case LCP_TERM_ACK:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_TERM_ACK\n"));
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN) {
      break; /* Term-acks are silently discarded in this state to avoid creating a loop */
    }
    pcb->state = PPP_LCP_CLOSED;
    PPP_EVENT_DISCONNECTED(pcb, ERR_OK, PPP_LCP, ret);
    break;
  case LCP_CODE_REJ:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_CODE_REJ\n"));
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN) {
      break; /* Silently discarded */
    }
    /* Handle a code reject depending on what code was rejected */
    switch(*((u8_t *)p->payload)) {
    case LCP_CFG_REQ:
    case LCP_CFG_ACK:
    case LCP_CFG_NAK:
    case LCP_CFG_REJ:
      pcb->state = PPP_LCP_CLOSED;
      PPP_EVENT_CONNECTED(pcb, ERR_VAL, ret);
      break;
    case LCP_TERM_REQ:
      pcb->state = PPP_LCP_CLOSED;
      PPP_EVENT_DISCONNECTED(pcb, ERR_VAL, PPP_LCP, ret);
      break;
    case LCP_ECHO_REQ:
      PPP_EVENT_ECHO_RSP(pcb, ERR_VAL, ret);
      break;
    default:
      /* Silently discard packet */
      break;
    }
  case LCP_PROTO_REJ:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_CODE_REJ\n"));
    /* All rejections are catastrophic to this implementation */
    if(pcb->state == PPP_LCP_OPEN) {
      ppp_cp_output(pcb, PPP_LCP, LCP_TERM_REQ, ppp_next_id(), NULL);
      pcb->state = PPP_LCP_CLOSING;
    } else {
      pcb->state = PPP_LCP_CLOSED;
      PPP_EVENT_DISCONNECTED(pcb, ERR_VAL, *((u16_t *)p->payload), ret);
    }
    break;
  case LCP_ECHO_REQ:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_ECHO_REQ\n"));
    if(pcb->state < PPP_LCP_OPEN && pcb->state > PPP_IPCP_OPEN) {
      LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: Echo request silently discarded. State = %d\n", pcb->state));
      break; /* Silently discarded */
    }
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: Send echo response\n"));
    pbuf_ref(p);
    ppp_cp_output(pcb, PPP_LCP, LCP_ECHO_RSP, cphdr->id, p); /* Send echo reply */
    break;
  case LCP_ECHO_RSP:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_ECHO_RSP\n"));
    PPP_EVENT_ECHO_RSP(pcb, ERR_OK, ret);
    break;
  case LCP_DISCARD_REQ:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_DISCARD_REQ\n"));
    /* Silently discard packet */
    break;
 default:
   LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: LCP_UNKNOWN\n"));
   /* Code reject */
   pbuf_header(p, 4); /* The data field in the reject packet begin with the 
			 information field */
   ppp_pbuf_ref_chain(p); /* Increase reference count so that ppp_cp_output don't free it */
   ppp_cp_output(pcb, PPP_LCP, LCP_CODE_REJ, cphdr->id, p);
   break;
  }
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_process_ipcp():
 *
 * Parses the received IPCP packet and handles it.
 */
/*-----------------------------------------------------------------------------------*/
void
ppp_process_ipcp(struct ppp_pcb *pcb, struct pbuf *p)
{
  struct ppp_cp_hdr *cphdr;
  struct pbuf *r, *s;
  struct ppp_cfg_hdr *cfghdr;
  struct ppp_req *req;
  u8_t rspstate = IPCP_CFG_ACK;
  u8_t ipaddropt = 0; /* Used in IPCP config requests to check if we have received an 
			 ip address option. If not, we append it to a NAK since configuration
		         about the remote IP-address is required
		      */
  err_t ret;
  //struct ppp_pcb *tpcb;
  u16_t len;
  
  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp\n"));

  /* Check if the remote IP-address should have been negotiated */
  //for(tpcb = ppp_active_pcbs; tpcb != NULL; tpcb = tpcb->next) {
  //  if(pcb->rfcommpcb == tpcb->rfcommpcb) {
  //    break; /* We are serving a client that should request an IP-address from us */
  //  }
  //}

  cphdr = p->payload;
  pbuf_header(p, -PPP_CPHDR_LEN);
  cphdr->len = ntohs(cphdr->len);
  len = cphdr->len - PPP_CPHDR_LEN;

  if(cphdr->code == IPCP_CFG_ACK || cphdr->code == IPCP_CFG_NAK || 
     cphdr->code == IPCP_CFG_REJ || cphdr->code == IPCP_CODE_REJ || 
     cphdr->code == IPCP_TERM_ACK) {
    for(req = pcb->reqs; req != NULL; req = req->next) {
      /* Remove any matching request */
      if(cphdr->id == req->id) {
	PPP_REQ_RMV(&(pcb->reqs), req);
	pbuf_free(req->p);
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: Free memory for request with code 0x%x***********\n", req->id));
	lwbt_memp_free(MEMP_PPP_REQ, req);
      }
    }
  }

  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: cphdr->code = 0x%x\n", cphdr->code));
  /*
{
struct pbuf *q;
  for(q = p; q != NULL; q = q->next) {
  u16_t i;
    for(i = 0; i < q->len; ++i) {
      LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: 0x%x\n", ((u8_t *)q->payload)[i]));
    }
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: q->len == %d q->tot_len == %d\n", q->len, q->tot_len));
  }
}
  */
  r = NULL;
  switch(cphdr->code) {
  case IPCP_CFG_REQ:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: IPCP_CFG_REQ\n"));
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN || 
       pcb->state == PPP_LCP_CFG || pcb->state == PPP_LCP_CLOSING) {
      /* A terminate-ack is sent to indicate that ipcp is in a closed state */
      ppp_cp_output(pcb, PPP_IPCP, IPCP_TERM_ACK, ppp_next_id(), NULL);
      break;
    }
    pcb->state = PPP_IPCP_CFG;

    while(len > 0) {
      cfghdr = p->payload;
      //pbuf_header(p, -PPP_CFGHDR_LEN);
      //len -= PPP_CFGHDR_LEN;
      switch(cfghdr->type) {
#if PPP_VJ_COMP
      case IPCP_CFG_COMP:
	//TODO: IMPLEMENT
	pcb->ipcpcfg |= IPCP_CFG_IN_VJ;

	/* NAK - Parameter not acceptable */
	/*	if(rspstate == IPCP_CFG_ACK) {
	  if(r != NULL) {
	    pbuf_free(r);
	    r = NULL;
	  }
	  r = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	  memcpy((u8_t *)r->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	  *((u32_t *)(((u16_t *)r->payload)+1)) = 0;
	  rspstate = IPCP_CFG_NAK;
	} else if (rspstate == IPCP_CFG_NAK) {
	  s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	  memcpy((u8_t *)s->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	  *((u32_t *)(((u16_t *)r->payload)+1)) = 0;
	  if(r == NULL) {
	    r = s;
	  } else {
	    pbuf_chain(r, s);
	    pbuf_free(s);
	  }
	  }*/ /* if rspstate == IPCP_CFG_REJ do not add packet to outgoing pbuf */
	break;
#endif /* PPP_VJ_COMP */
      case IPCP_CFG_IPADDR:
	ipaddropt = 1;
	//if(tpcb == NULL) {
	if(pcb->pppcfg & PPP_IR) {
	  /* We are a client and the remote server has given us its ip-address */
	  pcb->bluetoothif->gw.addr = *((u32_t *)(((u16_t *)p->payload) + 1));
	  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("We are a client and the remote server has given us its ip-address\nppp_process_ipcp: Remote IP address: %d.%d.%d.%d %ld\n", (u8_t)(ntohl(pcb->bluetoothif->gw.addr) >> 24) & 0xff, (u8_t)(ntohl(pcb->bluetoothif->gw.addr) >> 16) & 0xff, (u8_t)(ntohl(pcb->bluetoothif->gw.addr) >> 8) & 0xff, (u8_t)ntohl(pcb->bluetoothif->gw.addr) & 0xff, pcb->bluetoothif->gw.addr));
	  /* ACK - Parameter accepted */
	  if(rspstate == IPCP_CFG_ACK) {
	    s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	    if(r == NULL) {
	      r = s;
	    } else {
	      pbuf_chain(r, s);
	      pbuf_free(s);
	    }
	  }
	}
	else {
	  /* We are serving a client that we provide with an ip-address */
	  if(*((u32_t *)(((u16_t *)p->payload) + 1)) == pcb->bluetoothif->gw.addr) {
	    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("We are serving a client that we have provided with an ip-address: %d.%d.%d.%d %ld\n", (u8_t)(ntohl(pcb->bluetoothif->gw.addr) >> 24) & 0xff, (u8_t)(ntohl(pcb->bluetoothif->gw.addr) >> 16) & 0xff, (u8_t)(ntohl(pcb->bluetoothif->gw.addr) >> 8) & 0xff, (u8_t)ntohl(pcb->bluetoothif->gw.addr) & 0xff, pcb->bluetoothif->gw.addr));
	    /* ACK - Parameter accepted */
	    if(rspstate == IPCP_CFG_ACK) {
	      s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	      memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	      if(r == NULL) {
		r = s;
	      } else {
		pbuf_chain(r, s);
		pbuf_free(s);
	      }
	    }
	  } else {
	    /* NAK - Parameter not acceptable */
	    if(rspstate == IPCP_CFG_ACK) {
	      if(r != NULL) {
		pbuf_free(r);
		r = NULL;
	      }
	      r = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	      memcpy((u8_t *)r->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	      *((u32_t *)(((u16_t *)r->payload)+1)) = pcb->bluetoothif->gw.addr;
	      rspstate = IPCP_CFG_NAK;
	    } else if (rspstate == IPCP_CFG_NAK) {
	      s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	      memcpy((u8_t *)s->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	      *((u32_t *)(((u16_t *)s->payload)+1)) = pcb->bluetoothif->gw.addr;
	      if(r == NULL) {
		r = s;
	      } else {
		pbuf_chain(r, s);
		pbuf_free(s);
	      }
	    } /* if rspstate == IPCP_CFG_REJ do not add packet to outgoing pbuf */
	  }
	}
	break;
      case IPCP_CFG_PDNS:
	if(*((u32_t *)(((u16_t *)p->payload) + 1)) == PPP_IPCP_PDNS) {
	  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: IPCP_CFG_PDNS ACK\n"));
	  /* ACK - Parameter accepted */
	  if(rspstate == IPCP_CFG_ACK) {
	    s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	    if(r == NULL) {
	      r = s;
	    } else {
	      pbuf_chain(r, s);
	      pbuf_free(s);
	    }
	  }
	} else {
	  /* NAK - Parameter not acceptable */
	  if(rspstate == IPCP_CFG_ACK) {
	    if(r != NULL) {
	      pbuf_free(r);
	      r = NULL;
	    }
	    r = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)r->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	    *((u32_t *)(((u16_t *)r->payload)+1)) = PPP_IPCP_PDNS;
	    rspstate = IPCP_CFG_NAK;
	  } else if (rspstate == IPCP_CFG_NAK) {
	    s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)s->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	    *((u32_t *)(((u16_t *)s->payload)+1)) = PPP_IPCP_PDNS;
	    if(r == NULL) {
	      r = s;
	    } else {
	      pbuf_chain(r, s);
	      pbuf_free(s);
	    }
	  } /* if rspstate == IPCP_CFG_REJ do not add packet to outgoing pbuf */
	}
	break;
      case IPCP_CFG_PNBNS:
	if(*((u32_t *)(((u16_t *)p->payload) + 1)) == PPP_IPCP_NBNS) {
	  /* ACK - Parameter accepted */
	  if(rspstate == IPCP_CFG_ACK) {
	    s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	    if(r == NULL) {
	      r = s;
	    } else {
	      pbuf_chain(r, s);
	      pbuf_free(s);
	    }
	  }
	} else {
	  /* NAK - Parameter not acceptable */
	  if(rspstate == IPCP_CFG_ACK) {
	    if(r != NULL) {
	      pbuf_free(r);
	      r = NULL;
	    }
	    r = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)r->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	    *((u32_t *)(((u16_t *)r->payload)+1)) = PPP_IPCP_NBNS;
	    rspstate = IPCP_CFG_NAK;
	  } else if (rspstate == IPCP_CFG_NAK) {
	    s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)s->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	    *((u32_t *)(((u16_t *)s->payload)+1)) = PPP_IPCP_NBNS;
	    if(r == NULL) {
	      r = s;
	    } else {
	      pbuf_chain(r, s);
	      pbuf_free(s);
	    }
	  } /* if rspstate == IPCP_CFG_REJ do not add packet to outgoing pbuf */
	}
	break;
      case IPCP_CFG_SDNS:
	if(*((u32_t *)(((u16_t *)p->payload) + 1)) == PPP_IPCP_SDNS) {
	  /* ACK - Parameter accepted */
	  if(rspstate == IPCP_CFG_ACK) {
	    s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	    if(r == NULL) {
	      r = s;
	    } else {
	      pbuf_chain(r, s);
	      pbuf_free(s);
	    }
	  }
	} else {
	  /* NAK - Parameter not acceptable */
	  if(rspstate == IPCP_CFG_ACK) {
	    if(r != NULL) {
	      pbuf_free(r);
	      r = NULL;
	    }
	    r = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)r->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	    *((u32_t *)(((u16_t *)r->payload)+1)) = PPP_IPCP_SDNS;
	    rspstate = IPCP_CFG_NAK;
	  } else if (rspstate == IPCP_CFG_NAK) {
	    s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)s->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	    *((u32_t *)(((u16_t *)s->payload)+1)) = PPP_IPCP_SDNS;
	    if(r == NULL) {
	      r = s;
	    } else {
	      pbuf_chain(r, s);
	      pbuf_free(s);
	    }
	  } /* if rspstate == IPCP_CFG_REJ do not add packet to outgoing pbuf */
	}
	break;
      case IPCP_CFG_SNBNS:
	if(*((u32_t *)(((u16_t *)p->payload) + 1)) == PPP_IPCP_SNBNS) {
	  /* ACK - Parameter accepted */
	  if(rspstate == IPCP_CFG_ACK) {
	    s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	    if(r == NULL) {
	      r = s;
	    } else {
	      pbuf_chain(r, s);
	      pbuf_free(s);
	    }
	  }
	} else {
	  /* NAK - Parameter not acceptable */
	  if(rspstate == IPCP_CFG_ACK) {
	    if(r != NULL) {
	      pbuf_free(r);
	      r = NULL;
	    }
	    r = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)r->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	    *((u32_t *)(((u16_t *)r->payload)+1)) = PPP_IPCP_SNBNS;
	    rspstate = IPCP_CFG_NAK;
	  } else if (rspstate == IPCP_CFG_NAK) {
	    s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	    memcpy((u8_t *)s->payload, (u8_t *)p->payload, PPP_CFGHDR_LEN);
	    *((u32_t *)(((u16_t *)s->payload)+1)) = PPP_IPCP_SNBNS;
	    if(r == NULL) {
	      r = s;
	    } else {
	      pbuf_chain(r, s);
	      pbuf_free(s);
	    }
	  } /* if rspstate == IPCP_CFG_REJ do not add packet to outgoing pbuf */
	}
	break;
      default:
	/* Reject parameter */
	if(rspstate != IPCP_CFG_REJ) {
	  rspstate = IPCP_CFG_REJ;
	  if(r != NULL) {
	    pbuf_free(r);
	    r = NULL;
	  }
	}
	s = pbuf_alloc(PBUF_RAW, cfghdr->len, PBUF_RAM);
	memcpy((u8_t *)s->payload, (u8_t *)p->payload, cfghdr->len);
	if(r == NULL) {
	  r = s;
	} else {
	  pbuf_chain(r, s);
	  pbuf_free(s);
	}
	break;
      }
      pbuf_header(p, -cfghdr->len);
      len -= cfghdr->len;
    } /* while */

    /* Check if we are serving a client that should have requested an IP-address from us */
    //if(!ipaddropt && tpcb != NULL) {
    if(!ipaddropt && !(pcb->pppcfg & PPP_IR)) {
      /* NAK - the remote IP-address must be negotiated */
      if(rspstate == IPCP_CFG_ACK) {
	if(r != NULL) {
	  pbuf_free(r);
	}
	r = pbuf_alloc(PBUF_RAW, PPP_CFGHDR_LEN + 4, PBUF_RAM);
	cfghdr = r->payload;
	cfghdr->type = IPCP_CFG_IPADDR;
	cfghdr->len =  PPP_CFGHDR_LEN + 4;
	*((u32_t *)(((u16_t *)r->payload)+1)) = pcb->bluetoothif->gw.addr;
	rspstate = IPCP_CFG_NAK;
      } else if (rspstate == IPCP_CFG_NAK) {
	s = pbuf_alloc(PBUF_RAW, PPP_CFGHDR_LEN + 4, PBUF_RAM);
	cfghdr = s->payload;
	cfghdr->type = IPCP_CFG_IPADDR;
	cfghdr->len = PPP_CFGHDR_LEN + 4;
	*((u32_t *)(((u16_t *)s->payload)+1)) = pcb->bluetoothif->gw.addr;
	if(r == NULL) {
	  r = s;
	} else {
	  pbuf_chain(r, s);
	  pbuf_free(s);
	}
      } /* if rspstate == IPCP_CFG_REJ do not add packet to outgoing pbuf */
    }

    if(!(pcb->pppcfg & PPP_IR) && !(pcb->ipcpcfg & IPCP_CFG_OUT_REQ)) {
      /* Send an IPCP configure request for outgoing link if it hasnt been configured */
      ipcp_cfg_req(pcb);
      pcb->ipcpcfg |= IPCP_CFG_OUT_REQ;
    }

    /* Send response to configuration request */
    ppp_cp_output(pcb, PPP_IPCP, rspstate, cphdr->id, r);

    if(rspstate == IPCP_CFG_ACK) {
      pcb->ipcpcfg |= IPCP_CFG_OUT_ACK;
      /* IPCP connection established if a configuration ack has been sent */
      if(pcb->ipcpcfg & IPCP_CFG_IN_ACK) {
	/* IPCP connection established, notify upper layer that connection is open */
	pcb->state = PPP_IPCP_OPEN;
	PPP_EVENT_CONNECTED(pcb, ERR_OK, ret);
      }
    }
    break;
  case IPCP_CFG_ACK:
     LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: IPCP_CFG_ACK\n"));
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN || 
       pcb->state == PPP_LCP_CFG || pcb->state == PPP_LCP_CLOSING) {
      /* A terminate-ack is sent to indicate that ipcp is in a closed state */
      ppp_cp_output(pcb, PPP_IPCP, IPCP_TERM_ACK, ppp_next_id(), NULL);
      break;
    }
    pcb->ipcpcfg |= IPCP_CFG_IN_ACK;
    pcb->naks = 0;
    while(len > 0) {
      cfghdr = p->payload;
      //pbuf_header(p, -PPP_CFGHDR_LEN);
      //len -= PPP_CFGHDR_LEN;
      switch(cfghdr->type) {
#if PPP_VJ_COMP
      case IPCP_CFG_COMP:
	//TODO: IMPLEMENT
	pcb->ipcpcfg |= IPCP_CFG_OUT_VJ;
	break;
#endif /* PPP_VJ_COMP */	  
      default:
	/* Silently discard option */
	break;
      } /* switch */
      pbuf_header(p, -(cfghdr->len));
      len -= cfghdr->len;
    } /* while */
    /* IPCP connection established if a configuration ack has been sent */
    if(pcb->ipcpcfg & IPCP_CFG_OUT_ACK) {
      /* IPCP connection established, notify upper layer that connection is open */
      pcb->state = PPP_IPCP_OPEN;
      PPP_EVENT_CONNECTED(pcb, ERR_OK, ret);
    }
    break;
  case IPCP_CFG_NAK:
     LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: IPCP_CFG_NAK\n"));
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN || 
       pcb->state == PPP_LCP_CFG || pcb->state == PPP_LCP_CLOSING) {
      /* A terminate-ack is sent to indicate that ipcp is in a closed state */
      ppp_cp_output(pcb, PPP_IPCP, IPCP_TERM_ACK, ppp_next_id(), NULL);
      break;
    }
    ++pcb->naks;
    if(pcb->naks == PPP_MAX_FAILURE) {
      LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_lcp: ipcp negotiation failed\n"));
      ppp_disconnect(pcb); /* IPCP configuration attempt failed. Disconnect LCP */
      PPP_EVENT_CONNECTED(pcb, ERR_CONN, ret);
      break;
    }
    while(len > 0) {
      cfghdr = p->payload;
      //pbuf_header(p, -PPP_CFGHDR_LEN);
      //len -= PPP_CFGHDR_LEN;
      switch(cfghdr->type) {
#if PPP_VJ_COMP
      case IPCP_CFG_COMP:
	//TODO: IMPLEMENT
	pcb->ipcpopt &= ~IPCP_CFG_OPT_VJ;
	break;
#endif /* PPP_VJ_COMP */
      case IPCP_CFG_IPADDR:
	/* We have been given a local ip address by the remote host */
	pcb->bluetoothif->ip_addr.addr = *((u32_t *)(((u16_t *)p->payload) + 1));
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: Local IP address: %d.%d.%d.%d %ld\n", (u8_t)(ntohl(pcb->bluetoothif->ip_addr.addr) >> 24) & 0xff, (u8_t)(ntohl(pcb->bluetoothif->ip_addr.addr) >> 16) & 0xff, (u8_t)(ntohl(pcb->bluetoothif->ip_addr.addr) >> 8) & 0xff, (u8_t)ntohl(pcb->bluetoothif->ip_addr.addr) & 0xff, pcb->bluetoothif->ip_addr.addr));
	break;
      default:
	/* Silently discard configuration option */
	break;
      } /* switch */
      pbuf_header(p, -cfghdr->len);
      len -= cfghdr->len;
    } /* while */
    ipcp_cfg_req(pcb); /* Send new ipcp configuration request */
    break;
  case IPCP_CFG_REJ:
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: IPCP_CFG_REJ\n"));
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN || 
       pcb->state == PPP_LCP_CFG || pcb->state == PPP_LCP_CLOSING) {
      /* A terminate-ack is sent to indicate that ipcp is in a closed state */
      ppp_cp_output(pcb, PPP_IPCP, IPCP_TERM_ACK, ppp_next_id(), NULL);
      break;
    }
    while(len > 0) {
      cfghdr = p->payload;
      //pbuf_header(p, -PPP_CFGHDR_LEN);
      //len -= PPP_CFGHDR_LEN;
      switch(cfghdr->type) {
#if PPP_VJ_COMP
      case IPCP_CFG_COMP:
	pcb->ipcpopt &= ~IPCP_CFG_OPT_VJ;
	break;
#endif /* PPP_VJ_COMP */
      case IPCP_CFG_IPADDR:
	pcb->ipcpopt &= ~IPCP_CFG_OPT_IP;
	break;
      default:
	/* Silently discard configuration option */
	break;
      } /* switch */
      pbuf_header(p, -cfghdr->len);
      len -= cfghdr->len;
    } /* while */
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: Send new ipcp configuration request\n"));
    ipcp_cfg_req(pcb); /* Send new ipcp configuration request */
    break;
  case IPCP_TERM_REQ:
     LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: IPCP_TERM_REQ\n"));
    ppp_pbuf_ref_chain(p); /* Increase reference count so that ppp_cp_output don't delete the buffer */
    ppp_cp_output(pcb, PPP_IPCP, IPCP_TERM_ACK, cphdr->id, p);
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN || 
       pcb->state == PPP_LCP_CFG || pcb->state == PPP_LCP_CLOSING) {
      /* We are already closed */
      break;
    }
    pcb->state = PPP_LCP_OPEN;
    PPP_EVENT_DISCONNECTED(pcb, ERR_OK, PPP_IPCP, ret);
    break;
  case IPCP_TERM_ACK:
     LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: IPCP_TERM_ACK\n"));
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN || 
       pcb->state == PPP_LCP_CFG || pcb->state == PPP_LCP_CLOSING) {
      break; /* Term-acks are silently discarded in this state to avoid creating a loop */
    }
    pcb->state = PPP_LCP_OPEN;
    PPP_EVENT_DISCONNECTED(pcb, ERR_OK, PPP_IPCP, ret);
    break;
  case IPCP_CODE_REJ:
     LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: IPCP_CODE_REJ\n"));
    if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN || 
       pcb->state == PPP_LCP_CFG || pcb->state == PPP_LCP_CLOSING) {
      /* A terminate-ack is sent to indicate that ipcp is in a closed state */
      ppp_cp_output(pcb, PPP_IPCP, IPCP_TERM_ACK, ppp_next_id(), NULL);
      break;
    }
    /* Handle a code reject depending on what code was rejected */
    switch(*((u8_t *)p->payload)) {
    case IPCP_CFG_REQ:
    case IPCP_CFG_ACK:
    case IPCP_CFG_NAK:
    case IPCP_CFG_REJ:
      pcb->state = PPP_LCP_OPEN;
      PPP_EVENT_CONNECTED(pcb, ERR_VAL, ret);
      break;
    case IPCP_TERM_REQ:
      pcb->state = PPP_LCP_OPEN;
      PPP_EVENT_DISCONNECTED(pcb, ERR_VAL, PPP_IPCP, ret);
      break;
    default:
      /* Silently discard */
      break;
    }
    break;
  default:
     LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_ipcp: IPCP_UNKNOWN\n"));
    /* Code reject */
    pbuf_header(p, 4); /* The data field in the reject packet begin with the 
			  information field */
    ppp_pbuf_ref_chain(p); /* Increase reference count so that ppp_cp_output don't free it */
    ppp_cp_output(pcb, PPP_IPCP, IPCP_CODE_REJ, cphdr->id, p);
    break;
  }
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_process_pap():
 *
 * Parses the received PAP packet and handles it.
 */
/*-----------------------------------------------------------------------------------*/
void
ppp_process_pap(struct ppp_pcb *pcb, struct pbuf *p) 
{
  //TODO: Implement
  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_process_pap: not implemented yet!\n"));
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_input():
 * 
 * Called by the lower layer. Reassembles and unstuffs the packet, does a frame check, 
 * parses the header and forward it to the upper layer, LCP, IPCP or PAP handler.
 */
/*-----------------------------------------------------------------------------------*/
err_t
ppp_input(void *arg, struct rfcomm_pcb *rfcommpcb, struct pbuf *p, err_t err)
{
  struct ppp_pcb *pcb;
  u16_t proto;
  struct pbuf *q, *r;
  u8_t *payload;
  err_t ret;
  u16_t i;

  

  /* Search active pcbs for matching connection */
  for(pcb = ppp_active_pcbs; pcb != NULL; pcb = pcb->next) {
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: Found matching connection\n"));
    if(pcb->rfcommpcb == rfcommpcb) {
      break;
    }
  }
  if(pcb == NULL) {
    for(pcb = ppp_listen_pcbs; pcb != NULL; pcb = pcb->next) {
      if(pcb->rfcommpcb == rfcommpcb) {
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: Found matching listening connection\n")); 
	break;
      }
    }
    if(pcb == NULL) {
      /* Silently discard incoming data not belonging to any RFCOMM connection */
      LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: Silently discard incoming data not belonging to any RFCOMM connection\n")); 
      pbuf_free(p);
      return ERR_OK;
    }
  }

  for(q = p; q != NULL; q = q->next) {
    payload = (u8_t *)q->payload;
    for(i = 0; i < q->len; ++i) {
      if(pcb->p == NULL) {
	/* Alloc new pbuf. LwIP will handle dealloc */
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: Pbuf == NULL. Allocate and add new head\n"));
	if((pcb->p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL)) == NULL) {
	  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: Could not allocate memory for pbuf pcb->p\n"));
	  pbuf_free(p);
	  return ERR_MEM; /* Could not allocate memory for pbuf */
	}  
	pcb->q = pcb->p; /* Make q the pointer to the last pbuf which initially is the first */
	pcb->psize = 0;
	pcb->qsize = 0;
      }
      
      switch(payload[i]) {
      case PPP_END:
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: PPP_END i = %d pcb->psize = %d\n", i, pcb->psize));
	if(pcb->psize > 0) {
	  /* Packet received */
	  /* Do a frame check */
	  if((fcs16_crc_check(pcb->p, pcb->psize))) {
	    /* Silently discard frame */
	    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: FCS failed pcb->psize = %d\n", pcb->psize));
	    pbuf_free(pcb->p);
	    pcb->p = NULL;
	    break;
	  }
	  pbuf_realloc(pcb->p, pcb->psize - 2); /* Remove the FCS field and adjust packet size */

	  /* Have address and control field been compressed? */
	  if(ntohs(*((u16_t *)pcb->p->payload)) != PPP_ADDRCTRL) {
	    if(!(pcb->lcpcfg & LCP_CFG_IN_ACCOMP)) {
	      /* Silently discard frame */
	      LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: Discard frame without address and control field when no compression negotiated i = %d pcb->psize = %d\n", i, pcb->psize));
	      pbuf_free(pcb->p);
	      pcb->p = NULL;
	      break;
	    }
	  } else {
	    pbuf_header(pcb->p, -2); /* Hide address and control field */
	  }

	  /* Check if least significant bit of octet is a '1' and if protocol compression 
	     is in use */
	  if(*((u8_t *)pcb->p->payload) & 1 && pcb->lcpcfg & LCP_CFG_IN_PCOMP) {
	    proto = *((u8_t *)pcb->p->payload);
	    pbuf_header(pcb->p, -1);
	  } else {
	    proto = ntohs(*((u16_t *)pcb->p->payload));
	    pbuf_header(pcb->p, -2);
	  }

	  /* Check that size of incoming packet does not exceed MRU */
	  if(pcb->p->tot_len > pcb->mru) {
	    /* Silently discard frame */
	    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: Size of incoming packet exceed MRU, %d, %d\n", i, pcb->psize));
	    pbuf_free(pcb->p);
	    pcb->p = NULL;
	    break;
	  }

	  switch(proto) {
	    //struct ppp_cp_hdr *cphdr;
	  case PPP_LCP:
	    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: PPP_LCP i = %d pcb->psize = %d code = 0x%x\n", i, pcb->psize, ((u8_t *)pcb->p)[0]));
	    ppp_process_lcp(pcb, pcb->p);
	    pbuf_free(pcb->p);
	    pcb->p = NULL;
	    break;
	  case PPP_PAP:
	    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: PPP_PAP i = %d pcb->psize = %d\n", i, pcb->psize));
	    ppp_process_pap(pcb, pcb->p);
	    pbuf_free(pcb->p);
	    pcb->p = NULL;
	    break;
	  case PPP_IPCP:
	    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input:  PPP_IPCP i = %d pcb->psize = %d\n", i, pcb->psize));
	    ppp_process_ipcp(pcb, pcb->p);
	    pbuf_free(pcb->p);
	    pcb->p = NULL;
	    break;
	  case PPP_IP_DATA:
	    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: PPP_IP_DATA i = %d pcb->psize = %d pcb->p->tot_len = %d\n", i, pcb->psize, pcb->p->tot_len));
	    for(r = pcb->p; r != NULL; r = r->next) {
	      u16_t j;
	      for(j = 0; j < r->len; ++j) {
		LWIP_DEBUGF(TCP_DEBUG, ("IN: 0x%x\n", ((u8_t *)r->payload)[j]));
	      }
	      LWIP_DEBUGF(TCP_DEBUG, ("\n"));
	    }
	    //for(r = pcb->p; r != NULL; r = r->next) {
	    //  u16_t j;
	    //  for(j = 0; j < r->len; ++j) {
	    //LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: 0x%x\n", ((u8_t *)r->payload)[j]));
	    //  }
	      //LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: r->len == %d r->tot_len == %d\n", r->len, r->tot_len));
	    //}
	    

	    PPP_EVENT_RECV(pcb,ret);
	    pcb->p = NULL; /* Upper layer will dealloc pbuf. Delete any reference to it so we
			      don't overwrite it */
	    break;
	  default:
	    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: Unknown protocol, proto == %d, %d, %d\n", proto, i, pcb->psize));
	    r = pbuf_alloc(PBUF_RAW, 2, PBUF_RAM);
	    *((u16_t *)r->payload) = htons(proto);
	    pbuf_chain(r, pcb->p);
	    pbuf_free(pcb->p);
	    pcb->p = NULL;
	    ppp_cp_output(pcb, PPP_LCP, LCP_PROTO_REJ, ppp_next_id(), r);
	    //cphdr = pcb->p->payload;
	    //ppp_cp_output(pcb, proto, LCP_CFG_ACK, cphdr->id, NULL);
	    break;
	  }
	}
	break;
      case PPP_ESC:
	if(i < q->len-1) {
	  ++i; /* Move to next element in buffer */
	} else {
	  if(q->next != NULL) {
	    q = q->next; /* Move to next buffer in chain */
	    i = 0;
	    payload = (u8_t *)q->payload;
	  } else {
	    /* Last character in receive buffer is an escape character */
	    //TODO: Is this a problem?
	    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: Last character in receive buffer is an escape character!!!\n"));
	    break; /* No more data in buffer */
	  }
	}
	payload[i] ^= 0x20; /* Character following an escape character is exclusive-ord with 
			       0x20 */
	/* FALLTHROUGH */
      default:
	((u8_t *)pcb->q->payload)[pcb->qsize] = payload[i];
	++pcb->psize;
	++pcb->qsize;
	if(pcb->qsize == pcb->q->len) { /* Pbuf full. Allocate and add new tail to chain */
	  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: Pbuf full. Allocate and add new tail to chain\n"));
	  pcb->qsize = 0;
	  if((pcb->q = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL)) == NULL) {
	    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: Could not allocate memory for pbuf pcb->q\n"));
	    pbuf_free(p);
	    return ERR_MEM; /* Could not allocate memory for pbuf */
	  }
	  pbuf_chain(pcb->p, pcb->q);
	  pbuf_free(pcb->q);
	}
	if(pcb->psize > PPP_MAX_SIZE) {
	  /* PPP packet too big. Drop */
	  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_input: PPP packet too big. Drop, i = %d pcb->psize = %d\n", i, pcb->psize));
	  pbuf_free(pcb->p);
	  pcb->p = NULL;
	}
	break;
      }
    }
  }
  pbuf_free(p);
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_new():
 *
 * Creates a new PPP protocol control block but doesn't place it on
 * any of the PPP PCB lists.
 */
/*-----------------------------------------------------------------------------------*/
struct ppp_pcb *
ppp_new(struct rfcomm_pcb *rfcommpcb) 
{
  struct ppp_pcb *pcb;

  pcb = lwbt_memp_malloc(MEMP_PPP_PCB);
  if(pcb != NULL) {
    memset(pcb, 0, sizeof(struct ppp_pcb));
    pcb->rfcommpcb = rfcommpcb;
    pcb->state = PPP_LCP_CLOSED;
    pcb->mru = LCP_DEFAULT_MRU;
    pcb->outaccm = LCP_DEFAULT_ACCM;
    pcb->lcpopt = PPP_LCP_OPT;
    pcb->ipcpopt = PPP_IPCP_OPT;
    return pcb;
  }
  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_new: Could not allocate a new pcb\n"));
  return NULL;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_close():
 *
 * Closes the PPP protocol control block.
 */
/*-----------------------------------------------------------------------------------*/
err_t
ppp_close(struct ppp_pcb *pcb) 
{
  struct ppp_req *req;
  err_t err;

  /* Remove any outstanding requests */
  for(req = pcb->reqs; req != NULL; req = req->next) {
    PPP_REQ_RMV(&(pcb->reqs), req);
    if(req->p != NULL) {
      pbuf_free(req->p);
    }
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_close: Free memory for request with ID 0x%x***********\n", req->id));
    lwbt_memp_free(MEMP_PPP_REQ, req);
  }

  /* Remove PCB from its list */
  switch(pcb->state) {
  case PPP_LCP_LISTEN:
    PPP_RMV(&(ppp_listen_pcbs), pcb);
    break;
  default:
    PPP_RMV(&(ppp_active_pcbs), pcb);
    break;
  }

  err = ERR_OK;
  if(pcb->p != NULL) {//&& pcb->p->ref) {
    pbuf_free(pcb->p); /* Make sure we have freed any incoming packet */
    pcb->p = NULL;
  }
  lwbt_memp_free(MEMP_PPP_PCB, pcb);
  pcb = NULL;
    
  return err;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_reset_all():
 *
 * Closes all active PPP protocol control blocks.
 */
/*-----------------------------------------------------------------------------------*/
void
ppp_reset_all(void) {
  struct ppp_pcb *pcb, *tpcb;

  for(pcb = ppp_active_pcbs; pcb != NULL;) {
    tpcb = pcb->next;
    ppp_close(pcb);
    pcb = tpcb;
  }
  
  for(pcb = ppp_listen_pcbs; pcb != NULL;) {
    tpcb = pcb->next;
    ppp_close(pcb);
    pcb = tpcb;
  }

  ppp_init();
}
/*-----------------------------------------------------------------------------------*/
/* 
 * lcp_cfg_req():
 *
 * Assembles but do not send a LCP configuration request. The Options field is 
 * filled with any desired changes to the link defaults. 
 */
/*-----------------------------------------------------------------------------------*/
struct pbuf *
lcp_cfg_req(struct ppp_pcb *pcb, struct pbuf *options)
{
  struct pbuf *p, *q;
  struct ppp_cfg_hdr *cfghdr;

  p = NULL;
  if(pcb->lcpopt & LCP_CFG_OPT_MRU) {
    if(PPP_IN_MRU != LCP_DEFAULT_MRU) {
      p = pbuf_alloc(PBUF_RAW, PPP_CFGHDR_LEN + 2, PBUF_RAM);
      cfghdr = p->payload;
      cfghdr->type = LCP_CFG_MRU;
      cfghdr->len = p->len;
      ((u16_t *)p->payload)[1] = htons(PPP_IN_MRU);
    }
  }
  if(pcb->lcpopt & LCP_CFG_OPT_ACCM) {
    /* Add ACCM for outgoing packets */
    if(PPP_ACCM != LCP_DEFAULT_ACCM) {
      q = pbuf_alloc(PBUF_RAW, PPP_CFGHDR_LEN + 4, PBUF_RAM);
      cfghdr = q->payload;
      cfghdr->type = LCP_CFG_ACCM;
      cfghdr->len =  q->len;
      *((u32_t *)(((u16_t *)q->payload)+1)) = htonl(PPP_ACCM);
      if(p == NULL) {
	p = q;
      } else {
	pbuf_chain(p, q);
	pbuf_free(q);
      }
    }
  }
  if(pcb->lcpopt & LCP_CFG_OPT_PAP) {
    //TODO: IMPLEMENT PAP configuration request
  }
#if PPP_PHDR_COMP
  if(pcb->lcpopt & LCP_CFG_OPT_PFC) {
    q = pbuf_alloc(PBUF_RAW, PPP_CFGHDR_LEN, PBUF_RAM);
    cfghdr = q->payload;
    cfghdr->type = LCP_CFG_P_COMP;
    cfghdr->len = q->len;
    if(p == NULL) {
      p = q;
    } else {
      pbuf_chain(p, q);
      pbuf_free(q);
    }
  }
#endif /* PPP_PHDR_COMP */
#if PPP_ACHDR_COMP
  if(pcb->lcpopt & LCP_CFG_OPT_ACFC) {
    q = pbuf_alloc(PBUF_RAW, PPP_CFGHDR_LEN, PBUF_RAM);
    cfghdr = q->payload;
    cfghdr->type = LCP_CFG_AC_COMP;
    cfghdr->len = q->len;
    if(p == NULL) {
      p = q;
    } else {
      pbuf_chain(p, q);
      pbuf_free(q);
    }
  }
#endif /* PPP_ACHDR_COMP */
  if(options != NULL) {
    pbuf_chain(p, options);
    pbuf_free(options);
    
  }
  return p;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ipcp_cfg_req():
 *
 * Sends a IPCP configuration request to open an IPCP connection. The Options field 
 * is filled with any desired changes to the link defaults.
 */
/*-----------------------------------------------------------------------------------*/
err_t
ipcp_cfg_req(struct ppp_pcb *pcb)
{
  struct pbuf *p = NULL;
  struct ppp_cfg_hdr *cfghdr;

  if(pcb->ipcpopt & IPCP_CFG_OPT_IP) {
    if((p = pbuf_alloc(PBUF_RAW, PPP_CFGHDR_LEN + 4, PBUF_RAM)) == NULL) {
      LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ipcp_cfg_req: Could not allocate memory for pbuf\n"));
      return ERR_MEM; /* Could not allocate memory for pbuf */
    }
    cfghdr = p->payload;
    cfghdr->type = IPCP_CFG_IPADDR;
    cfghdr->len = p->len;
    *((u32_t *)(((u16_t *)p->payload)+1)) = pcb->bluetoothif->ip_addr.addr;
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ipcp_cfg_req: Local IP address: %d.%d.%d.%d\n", (u8_t)(ntohl(pcb->bluetoothif->ip_addr.addr) >> 24) & 0xff, (u8_t)(ntohl(pcb->bluetoothif->ip_addr.addr) >> 16) & 0xff, (u8_t)(ntohl(pcb->bluetoothif->ip_addr.addr) >> 8) & 0xff, (u8_t)ntohl(pcb->bluetoothif->ip_addr.addr) & 0xff));
  }
#if PPP_VJ_COMP
  if(pcb->ipcpopt & IPCP_CFG_OPT_VJ) {
    if((q = pbuf_alloc(PBUF_RAW, PPP_CFGHDR_LEN + 4, PBUF_RAM)) == NULL) {
      LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ipcp_cfg_req: Could not allocate memory for pbuf\n"));
      return ERR_MEM; /* Could not allocate memory for pbuf */
    }
    cfghdr = q->payload;
    cfghdr->type = IPCP_CFG_COMP;
    cfghdr->len = q->len;
    ((u16_t *)q->payload)[1] = PPP_VJ_TYPE;
    ((u8_t *)q->payload)[4] = PPP_MAXSLOT_ID;
    ((u8_t *)q->payload)[5] = PPP_COMPSLOT_ID;
    if(p == NULL) {
      p = q;
    } else {
      pbuf_chain(p, q);
      pbuf_free(q);
    }
  }
#endif /* PPP_VJ_COMP */
  return ppp_cp_output(pcb, PPP_IPCP, IPCP_CFG_REQ, ppp_next_id(), p);
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_connect():
 *
 * Initiates a connection attempt (LCP and IPCP) and specify the function to be 
 * called when both LCP and IPCP has been connected.
 */
/*-----------------------------------------------------------------------------------*/
err_t //INITIATOR
ppp_connect(struct ppp_pcb *pcb, err_t (* connected)(void *arg,
						     struct ppp_pcb *tpcb,
						     err_t err))
{
  struct pbuf *p;
  err_t ret = ERR_OK;

  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_connect\n"));

  pcb->connected = connected;
  pcb->pppcfg |= PPP_IR; /* We are the initiator */
  if(pcb->state == PPP_LCP_OPEN) {
    if((ret = ipcp_cfg_req(pcb)) == ERR_OK) {
      pcb->state = PPP_IPCP_CFG;
    }
  } else {
    PPP_REG(&ppp_active_pcbs, pcb);
    p = lcp_cfg_req(pcb, NULL);
    if((ret = ppp_cp_output(pcb, PPP_LCP, LCP_CFG_REQ, ppp_next_id(), p)) == ERR_OK) {
      pcb->state = PPP_LCP_CFG;
    }
  }
  return ret;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_disconnect():
 *
 * Closes the connection held by the PCB but does not dealloc it. Call the 
 * disconnected callback when disconnection is complete.
 */
/*-----------------------------------------------------------------------------------*/
err_t
ppp_disconnect(struct ppp_pcb *pcb)
{
  err_t ret;

  if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN || 
     pcb->state == PPP_LCP_CLOSING) {
    PPP_EVENT_DISCONNECTED(pcb, ERR_OK, PPP_LCP, ret);
    return ERR_OK;
  }
  pcb->state = PPP_LCP_CLOSING;
  return ppp_cp_output(pcb, PPP_LCP, LCP_TERM_REQ, ppp_next_id(), NULL);
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_echo():
 *
 * Sends an empty PPP echo request message. Also specify the function that should
 * be called when a PPP echo reply has been received.
 */
/*-----------------------------------------------------------------------------------*/
err_t
ppp_echo(struct ppp_pcb *pcb, err_t (* echo_rsp)(void *arg,
						 struct ppp_pcb *tpcb,
						 err_t err))
{
  if(pcb->state == PPP_LCP_CLOSED || pcb->state == PPP_LCP_LISTEN || 
     pcb->state == PPP_LCP_CFG || pcb->state == PPP_LCP_CLOSING) {
    return ERR_CLSD;
  }
  pcb->echo_rsp = echo_rsp;
  return ppp_cp_output(pcb, PPP_LCP, LCP_ECHO_REQ, ppp_next_id(), NULL);
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_cp_output():
 *
 * Sends a PPP control protocol packet (IPCP or LCP).
 */
/*-----------------------------------------------------------------------------------*/
err_t
ppp_cp_output(struct ppp_pcb *pcb, u16_t proto, u8_t code, u8_t id, struct pbuf *q)
{
  struct pbuf *p, *r;
  struct ppp_req *req;
  err_t ret;
  u16_t fcs;
  u8_t j = 0;

  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_cp_output: Code == 0x%x\n", code));

  if(pcb->lcpcfg & LCP_CFG_OUT_ACCOMP && pcb->state != PPP_LCP_CFG) {
    p = pbuf_alloc(PBUF_RAW, PPP_HDR_LEN - 2 + PPP_CPHDR_LEN, PBUF_RAM);
  } else {
    p = pbuf_alloc(PBUF_RAW, PPP_HDR_LEN + PPP_CPHDR_LEN, PBUF_RAM);
  }
  
  if(!(pcb->lcpcfg & LCP_CFG_OUT_ACCOMP && pcb->state != PPP_LCP_CFG)) {
    ((u16_t *)p->payload)[j/2] = htons(PPP_ADDRCTRL); /* Set adddress control field */
    j+= 2;
  }
  ((u16_t *)p->payload)[j/2] = htons(proto); /* Set protocol field */
  j+= 2;
  ((u8_t *)p->payload)[j++] = code; /* Set type of control packet */
  ((u8_t *)p->payload)[j++] = id; /* Set packet identifier */
  if(q == NULL) {
    ((u16_t *)p->payload)[j/2] = htons(PPP_CPHDR_LEN); /* Set length of control packet */
    j+= 2;
  } else {
    ((u16_t *)p->payload)[j/2] = htons(q->tot_len + PPP_CPHDR_LEN); /* Set length of control packet */
    j+= 2;
    pbuf_chain(p, q);
    pbuf_free(q);
  }

  r = pbuf_alloc(PBUF_RAW, 2, PBUF_RAM); /* Alloc a pbuf for fcs */

  /* Add FCS to packet */
  fcs =  fcs16_crc_calc(p, p->tot_len);
  ((u8_t *)r->payload)[0]  = (fcs & 0xFF); /* Least significant byte first */
  ((u8_t *)r->payload)[1] = ((fcs >> 8) & 0xFF);
  pbuf_chain(p, r);
  pbuf_free(r);

  ret = ppp_output(pcb, p);
  /* Check if a timer should be associated with the request */
  if(ret == ERR_OK && (code == LCP_CFG_REQ || code == IPCP_CFG_REQ || 
     code == LCP_TERM_REQ || code == IPCP_TERM_REQ)) {
    /* Check if the request already is in the outstanding requests list */
    for(req = pcb->reqs; req != NULL; req = req->next) {
      if(req->id == id) {
	break;
      }
    }
    if(req == NULL) {
      /* Add request to outstanding requests list */
      LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_cp_output: Allocate memory for request with ID 0x%x Code 0x%x***********\n", id, code));
      if((req = lwbt_memp_malloc(MEMP_PPP_REQ)) == NULL) {
	LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_cp_output: could not allocate memory for request\n"));
	pbuf_free(p); /* Dealloc control packet */
	return ERR_MEM;
      }
      req->p = p;
      req->id = id;
      req->proto = proto;
      if((req->rto = (PPP_RTO/(PPP_NRTX + 1))) == 0) {
	req->rto = 1;
      }
      req->nrtx = PPP_NRTX;
      PPP_REQ_REG(&(pcb->reqs), req);
    }
  } else {
    pbuf_free(p); /* Free control packet */
  }
  return ret;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_netif_output():
 *
 * Output IP data from a lwIP network interface over PPP.
 */
/*-----------------------------------------------------------------------------------*/
err_t
ppp_netif_output(struct netif *netif, struct pbuf *p, struct ip_addr *ipaddr)
{
  struct ppp_pcb *pcb;

  /*
  struct pbuf *q;
  u16_t i;  
  for(q = p; q != NULL; q = q->next) { 
    for(i = 0; i < q->len; ++i) {
      LWIP_DEBUGF(TCP_DEBUG, ("Out: 0x%x\n", ((u8_t *)q->payload)[i]));
    }
    LWIP_DEBUGF(TCP_DEBUG, ("\n"));
  }
  */
  for(pcb = ppp_active_pcbs; pcb != NULL; pcb = pcb->next) {
    if(pcb->bluetoothif == netif) {
      break;
    }
  }
  if(pcb != NULL) {
    LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_netif_output: Local IP address: %d.%d.%d.%d\n", (u8_t)(ntohl(pcb->bluetoothif->ip_addr.addr) >> 24) & 0xff, (u8_t)(ntohl(pcb->bluetoothif->ip_addr.addr) >> 16) & 0xff, (u8_t)(ntohl(pcb->bluetoothif->ip_addr.addr) >> 8) & 0xff, (u8_t)ntohl(pcb->bluetoothif->ip_addr.addr) & 0xff));
    return ppp_data_output(pcb, p);
  }
  return ERR_CONN; /* No matching PPP connection exists */
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_data_output():
 *
 * Output IP data over PPP.
 */
/*-----------------------------------------------------------------------------------*/
err_t
ppp_data_output(struct ppp_pcb *pcb, struct pbuf *q)
{
  struct pbuf *r, *p;
  u8_t hdrlen = PPP_HDR_LEN;
  u8_t i = 0;
  err_t ret;
  u16_t fcs;
  
  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_data_output: q->tot_len = %d\n", q->tot_len));

  if(pcb == NULL) {
    return ERR_CONN; /* Not connected */
  }
  if(pcb->state != PPP_IPCP_OPEN) {
    return ERR_CONN; /* Not connected */
  }

  if(pcb->lcpcfg & LCP_CFG_OUT_ACCOMP) {
    hdrlen -= 2;
  }
  if(pcb->lcpcfg & LCP_CFG_OUT_PCOMP) {
    --hdrlen;
  }
  
  p = pbuf_alloc(PBUF_RAW, hdrlen, PBUF_RAM);
  
  if(!(pcb->lcpcfg & LCP_CFG_OUT_ACCOMP)) {
    *((u16_t *)p->payload) = htons(PPP_ADDRCTRL);
    i += 2;
  }
  if(pcb->lcpcfg & LCP_CFG_OUT_PCOMP) {
    ((u8_t *)p->payload)[i] = PPP_IP_DATA;
  } else {
    ((u16_t *)p->payload)[i/2] = htons(PPP_IP_DATA); //TODO: WE MAY DIVIDE BY ZERO HERE...
  }

  /* Chain any information data to header */
  if(q != NULL) {
    pbuf_chain(p, q);
    //pbuf_free(q);
  }

  r = pbuf_alloc(PBUF_RAW, 2, PBUF_RAM); /* Alloc a pbuf for fcs */

  /* Add FCS to packet */
  fcs =  fcs16_crc_calc(p, p->tot_len);
  ((u8_t *)r->payload)[0]  = (fcs & 0x00ff); /* Least significant byte first */
  ((u8_t *)r->payload)[1] = ((fcs >> 8) & 0x00ff);
  pbuf_chain(p, r);
  pbuf_free(r);

  ret = ppp_output(pcb, p);

  /* Free PPP header. Higher layers will handle rest of packet */
  if(q != NULL) {
    pbuf_dechain(p);
    pbuf_realloc(q, q->tot_len-2); /* Remove FCS from packet */
  }
  pbuf_free(p);

  return ret;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_output_alloc():
 *
 * Called by ppp_output() to allocate a pbuf with the same size as the remaining
 * characters in the packet to be byte stuffed.
 */
/*-----------------------------------------------------------------------------------*/
struct pbuf *
ppp_output_alloc(struct pbuf *p, u16_t i)
{
  struct pbuf *q;
  u16_t k = 0;
  
  /* Get length of remaining characters in packet */
  for(q = p; q != NULL; q = q->next) {
    k += q->len; 
  }
  k -= i;
  q = pbuf_alloc(PBUF_RAW, k, PBUF_RAM); /* Alloc a pbuf for that size */
  return q;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_output():
 *
 * Output PPP data. Byte stuffs the packet and forward it to RFCOMM.
 */
/*-----------------------------------------------------------------------------------*/
err_t
ppp_output(struct ppp_pcb *pcb, struct pbuf *p) 
{
  struct pbuf *q, *r, *s;
  u16_t i;
  u16_t j = 0;
  u8_t c;
  err_t ret;

  /* To minimize the number of pbufs we need to chain together we always allocate the same size as the
     number of characters remaining to be checked */
  r = pbuf_alloc(PBUF_RAW, p->tot_len + 1, PBUF_RAM);
  s = r;
  ((u8_t *)s->payload)[j++] = PPP_END;
  
  for(q = p; q != NULL; q = q->next) {
    for(i = 0; i < q->len; ++i) {
      c = ((u8_t *)q->payload)[i];
      if(j == s->len) {
	s = ppp_output_alloc(q, i);
	pbuf_chain(r, s);
	pbuf_free(s);
	j = 0;
      }
      switch(c) {
      case PPP_END:
	((u8_t *)s->payload)[j++] = PPP_ESC;
	if(j == s->len) {
	  s = ppp_output_alloc(q, i);
	  pbuf_chain(r, s);
	  pbuf_free(s);
	  j = 0;
	}
	((u8_t *)s->payload)[j++] = PPP_ESC_END;
	break;
      case PPP_ESC:
	((u8_t *)s->payload)[j++] = PPP_ESC;
	if(j == s->len) {
	  s = ppp_output_alloc(q, i);
	  pbuf_chain(r, s);
	  pbuf_free(s);
	  j = 0;
	}
	((u8_t *)s->payload)[j++] = PPP_ESC_ESC;
	break;
      default:
	/* Check if the character should be escaped according to the ACCM */
	if(c < 0x20) {
	  if(pcb->outaccm & (1 << c) || pcb->state == PPP_LCP_CFG) {
	    ((u8_t *)s->payload)[j++] = PPP_ESC;
	    if(j == s->len) {
	      s = ppp_output_alloc(q, i);
	      pbuf_chain(r, s);
	      pbuf_free(s);
	      j = 0;
	    }
	    ((u8_t *)s->payload)[j++] = c ^ 0x20; /* Character following escape character is 
						     exclusive-ord with 0x20 */
	  } else {
	    ((u8_t *)s->payload)[j++] = c;
	  }
	} else {
	  ((u8_t *)s->payload)[j++] = c;
	}
	break;
      }
    }
  }

  s = pbuf_alloc(PBUF_RAW, 1, PBUF_RAM); /* Alloc a pbuf for PPP_END */
  *((u8_t *)s->payload) = PPP_END;
  pbuf_chain(r, s);
  pbuf_free(s);
  
  LWIP_DEBUGF(LWBT_PPP_DEBUG, ("ppp_output: r->tot_len = %d\n", r->tot_len));

  /*
  for(s = r; s != NULL; s = s->next) {
    //u8_t i;
    for(i = 0; i < s->len; ++i) {
      LWIP_DEBUGF(SDP_DEBUG, ("ppp_output: REQ 0x%x\n", ((u8_t *)s->payload)[i]));
    }
    LWIP_DEBUGF(SDP_DEBUG, ("ppp_output: STOP\n"));
  }
  */

  /* Check which convergence layer is in use */
  if(rfcomm_cl(pcb->rfcommpcb)) {
    ret = rfcomm_uih_credits(pcb->rfcommpcb, PBUF_POOL_SIZE - rfcomm_remote_credits(pcb->rfcommpcb), r);
  } else {
    ret = rfcomm_uih(pcb->rfcommpcb, rfcomm_cn(pcb->rfcommpcb), r);
  }

  pbuf_free(r); /* Free byte stuffed copy of outgoing packet */

  return ret;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_next_id():
 *
 * Return a new control protocol identifier. Aids in matching requests and replies.
 */
/*-----------------------------------------------------------------------------------*/
u8_t
ppp_next_id(void)
{
  return ++id_nxt;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_disconnected():
 *
 * Used to specify the function that should be called when a PPP connection is 
 * disconnected
 */
/*-----------------------------------------------------------------------------------*/
void
ppp_disconnected(struct ppp_pcb *pcb, err_t (* disconnected)(void *arg, 
							     struct ppp_pcb *pcb,
							     u16_t proto,
							     err_t err))
{
  pcb->disconnected = disconnected;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_listen():
 *
 * Set the state of the connection to be PPP_LCP_LISTEN, which means that it is able 
 * to accept incoming connections.
 */
/*-----------------------------------------------------------------------------------*/
#if LWBT_LAP
err_t
ppp_listen(struct ppp_pcb *pcb, err_t (* accept)(void *arg, 
						 struct ppp_pcb *pcb, 
						 err_t err))
{
  pcb->connected = accept;
  pcb->state = PPP_LCP_LISTEN;

  PPP_REG(&ppp_listen_pcbs, pcb);
  return ERR_OK;
}
#endif /* LWBT_LAP */
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_netif():
 *
 * Used to specify the network interface that should be used to pass incoming IP
 * data to lwIP.
 */
/*-----------------------------------------------------------------------------------*/
void
ppp_netif(struct ppp_pcb *pcb, struct netif *netif) 
{
  pcb->bluetoothif = netif;
}
/*-----------------------------------------------------------------------------------*/
/* 
 * ppp_arg():
 *
 * Used to specify the argument that should be passed callback functions.
 */
/*-----------------------------------------------------------------------------------*/
void
ppp_arg(struct ppp_pcb *pcb, void *arg) 
{
  pcb->callback_arg = arg;
}
/*-----------------------------------------------------------------------------------*/
void
ppp_pbuf_ref_chain(struct pbuf *p)
{
  while (p != NULL) {
    ++p->ref;
    p = p->next;
  }
}
/*-----------------------------------------------------------------------------------*/
