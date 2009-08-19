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
 
#ifndef __LWBT_LWBTOPTS_H__
#define __LWBT_LWBTOPTS_H__

#define PHYBUSIF_DEBUG DBG_OFF
#define HCI_DEBUG DBG_OFF
#define HCI_EV_DEBUG DBG_OFF
#define L2CAP_DEBUG DBG_OFF
#define SDP_DEBUG DBG_OFF
#define RFCOMM_DEBUG DBG_OFF
#define LWBT_PPP_DEBUG DBG_OFF
#define NAT_DEBUG DBG_OFF

/* ---------- LAP profile options ---------- */
/* LAP_CN: RFCOMM server channel that the PPP server listens to */
#define LAP_CN 1
/* LWBT_DT:  Control if we use the services of a LAP */
#define LWBT_DT 1
/* LWBT_LAP:  Control if this we provide access to a LAN */
#define LWBT_LAP 1

/* ---------- Memory options ---------- */
#define MAX_NUM_CLIENTS 6 /* Maximum number of connected Bluetooth clients. No more than 6 */ 

#define MEMP_NUM_NAT_PCB 8 /* Must be set to 1 or more if NAT is used. Set this to the number of diffrent ports/ids that the application use */

#define MEMP_NUM_HCI_PCB 1 /* Always set to one */
#define MEMP_NUM_HCI_LINK (1 + MAX_NUM_CLIENTS) /* One for DT + One per ACL connection */
#define MEMP_NUM_HCI_INQ 1 /* One per max number of returned results from an inquiry */

/* MEMP_NUM_L2CAP_PCB: the number of simulatenously active L2CAP
   connections. */
#define MEMP_NUM_L2CAP_PCB (2 + MAX_NUM_CLIENTS) /* One for a closing connection + one for DT + one per number of connected Bluetooth clients */
/* MEMP_NUM_L2CAP_PCB_LISTEN: the number of listening L2CAP
   connections. */
#define MEMP_NUM_L2CAP_PCB_LISTEN 2 /* One per listening PSM */
/* MEMP_NUM_L2CAP_SIG: the number of simultaneously unresponded
   L2CAP signals */
#define MEMP_NUM_L2CAP_SIG (2 * MAX_NUM_CLIENTS)/* Two per number of connected Bluetooth clients but min 2 */
#define MEMP_NUM_L2CAP_SEG (2 + 2 * MAX_NUM_CLIENTS) /* One per number of L2CAP connections */

#define MEMP_NUM_SDP_PCB MAX_NUM_CLIENTS /* One per number of connected Bluetooth clients */
#define MEMP_NUM_SDP_RECORD 1 /* One per registered service record */

#define MEMP_NUM_RFCOMM_PCB (2 + 2 * MAX_NUM_CLIENTS) /* Two for DT + Two per number of connected Bluetooth clients */
#define MEMP_NUM_RFCOMM_PCB_LISTEN (2 * MAX_NUM_CLIENTS) /* Two per number of connected Bluetooth clients */

#define MEMP_NUM_PPP_PCB (1 + MAX_NUM_CLIENTS) /* One for DT + One per number of connected Bluetooth clients */
#define MEMP_NUM_PPP_REQ MAX_NUM_CLIENTS /* One per number of connected Bluetooth clients but min 1 */

/* ---------- HCI options ---------- */
/* HCI: Defines if we have lower layers of the Bluetooth stack running on a separate host
   controller */
#define HCI 1

#if HCI
/* HCI_HOST_MAX_NUM_ACL: The maximum number of ACL packets that the host can buffer */
#define HCI_HOST_MAX_NUM_ACL 8 //TODO: Should be equal to PBUF_POOL_SIZE/2??? */
/* HCI_HOST_ACL_MAX_LEN: The maximum size of an ACL packet that the host can buffer */
#define HCI_HOST_ACL_MAX_LEN (RFCOMM_N + 14) /* Default: RFCOMM MFS + ACL header size, L2CAP header size, 
                                                RFCOMM header size and RFCOMM FCS size */
/* HCI_PACKET_TYPE: The set of packet types which may be used on the connection. In order to 
   maximize packet throughput, it is recommended that RFCOMM should make use of the 3 and 5 
   slot baseband packets.*/
#define HCI_PACKET_TYPE 0xCC18 /* Default DM1, DH1, DM3, DH3, DM5, DH5 */
/* HCI_ALLOW_ROLE_SWITCH: Tells the host controller whether to accept a Master/Slave switch 
   during establishment of a connection */
#define HCI_ALLOW_ROLE_SWITCH 1 /* Default 1 */
/* HCI_FLOW_QUEUEING: Control if a packet should be queued if the host controller is out of 
   bufferspace for outgoing packets. Only the first packet sent when out of credits will be 
   queued */
#define HCI_FLOW_QUEUEING 0 /* Default: 0 */

#endif /* HCI */

/* ---------- L2CAP options ---------- */
/* L2CAP_HCI: Option for including HCI to access the Bluetooth baseband capabilities */
#define L2CAP_HCI 1 //TODO: NEEDED?
/* L2CAP_CFG_QOS: Control if a flow specification similar to RFC 1363 should be used */
#define L2CAP_CFG_QOS 0
/* L2CAP_MTU: Maximum transmission unit for L2CAP packet payload (min 48) */
#define L2CAP_MTU (RFCOMM_N + 6)/* Default for this implementation is RFCOMM MFS + RFCOMM header size and 
				   RFCOMM FCS size while the L2CAP default is 672 */
/* L2CAP_OUT_FLUSHTO: For some networking protocols, such as many real-time protocols, guaranteed delivery
   is undesirable. The flush time-out value SHALL be set to its default value 0xffff for a reliable L2CAP 
   channel, and MAY be set to other values if guaranteed delivery is not desired. (min 1) */
#define L2CAP_OUT_FLUSHTO 0xFFFF /* Default: 0xFFFF. Infinite number of retransmissions (reliable channel)
				    The value of 1 implies no retransmissions at the Baseband level 
				    should be performed since the minimum polling interval is 1.25 ms.*/ 
/* L2CAP_RTX: The Responsive Timeout eXpired timer is used to terminate
   the channel when the remote endpoint is unresponsive to signalling
   requests (min 1s, max 60s) */
#define L2CAP_RTX 60
/* L2CAP_ERTX: The Extended Response Timeout eXpired timer is used in
   place of the RTC timer when a L2CAP_ConnectRspPnd event is received
   (min 60s, max 300s) */
#define L2CAP_ERTX 300
/* L2CAP_MAXRTX: Maximum number of Request retransmissions before
   terminating the channel identified by the request. The decision
   should be based on the flush timeout of the signalling link. If the
   flush timeout is infinite, no retransmissions should be performed */
#define L2CAP_MAXRTX 0
/* L2CAP_CFG_TO: Amount of time spent arbitrating the channel parameters
   before terminating the connection (max 120s) */  
#define L2CAP_CFG_TO 30

/* ---------- SDP options ---------- */

/* ---------- RFCOMM options ---------- */
/* RFCOMM_N: Maximum frame size for RFCOMM segments (min 23, max 32767)*/
#define RFCOMM_N ((PPP_IN_MRU * 2) + 8) /* Default: Worst case byte stuffed PPP packet size + 
					   non-compressed PPP header size and FCS size */
/* RFCOMM_K: Initial amount of credits issued to the peer (min 0, max 7) */
#define RFCOMM_K 0
/* RFCOMM_TO: Acknowledgement timer (T1) and response timer for multiplexer control channel (T2).
   T1 is the timeout for frames sent with the P/F bit set to 1 (SABM and DISC) and T2 is the timeout
   for commands sent in UIH frames on DLCI 0 (min 10s, max 60s) */
#define RFCOMM_TO 20
/* RFCOMM_FLOW_QUEUEING: Control if a packet should be queued if a channel is out of credits for 
   outgoing packets. Only the first packet sent when out of credits will be queued */
#define RFCOMM_FLOW_QUEUEING 0 /* Default: 0 */

/* ---------- PPP options ---------- */
/* PPP_MAX_SIZE: Maximum size of an incoming PPP packet */
#define PPP_MAX_SIZE 1500
/* PPP_IN_MRU: Maximum receive unit that the implementation can receive. Indicates maximum number 
   of octets in the information field */
#define PPP_IN_MRU 576//(TCP_MSS + 40) /* Default: TCP_MSS + default TCP/IP header size */
/* PPP_ACCM: ASCII Characters 0-31 that need to be mapped. Bit 0 represents character 0 and 
   bit 31 indicates character 31 */
#define PPP_ACCM 0x00000000 /* Default 0xFFFFFFFF for PPP, 0x00000000 for this 
			       implementation */
/* PPP_AUTH: Control if the Password authentication protocol should be used for client 
   identification before allowing network-layer protocol packets to be exchanged */
#define PPP_AUTH 0 //TODO: Implement
/* PPP_PHDR_COMP: Control if we should compress the PPP Protocol field */
#define PPP_PHDR_COMP 1
/* PPP_ACHDR_COMP: Control if we should compress the Data Link Layer Address and Control fields */
#define PPP_ACHDR_COMP 1
/* PPP_LCP_OPT: Control which LCP options should be sent in the initial LCP configuration 
   request */
#define PPP_LCP_OPT 0x1D
/* PPP_IPCP_OPT: Control which IPCP options should be sent in the initial IPCP configuration 
   request */
#define PPP_IPCP_OPT 0x06
/* PPP_VJ_COMP: Control if Van-Jacobsen compression protocol should be used */
//TODO: Implement
#define PPP_VJ_COMP 0 
#define PPP_VJ_TYPE 0
#define PPP_MAXSLOT_ID 0
#define PPP_COMPSLOT_ID 0
/* PPP_RTO: Restart timer for Configure- and Terminate-Requests */
#define PPP_RTO 9 /* Default 3s */
/* PPP_RTO: Number of retransmissions of timed out requests */
#define PPP_NRTX 3 /* Default 3 */
/* PPP_MAX_FAILURE: Number of Configure-Request packets sent without receiving a valid 
   Configure-Ack, Configure-Nak or Configure-Reject before assuming that the peer is 
   unable to respond */
#define PPP_MAX_FAILURE 5 /* Default 5 */
/* PPP_IPCP_PDNS: The primary DNS server to be used on the local end of the link */
#define PPP_IPCP_PDNS htonl(((u32_t)(130 & 0xff) << 24) | ((u32_t)(240 & 0xff) << 16) | \
			  ((u32_t)(3 & 0xff) << 8) | (u32_t)(1 & 0xff))
/* PPP_IPCP_NBNS: The primary NBNS server to be used on the local end of the link */
#define PPP_IPCP_NBNS htonl(((u32_t)(130 & 0xff) << 24) | ((u32_t)(240 & 0xff) << 16) | \
			  ((u32_t)(42 & 0xff) << 8) | (u32_t)(94 & 0xff))
/* PPP_IPCP_SDNS: The secondary DNS server to be used on the local end of the link */
#define PPP_IPCP_SDNS htonl(((u32_t)(130 & 0xff) << 24) | ((u32_t)(240 & 0xff) << 16) | \
			  ((u32_t)(19 & 0xff) << 8) | (u32_t)(2 & 0xff))
/* PPP_IPCP_SNBNS: The secondary SNBNS server to be used on the local end of the link */
#define PPP_IPCP_SNBNS htonl(((u32_t)(130 & 0xff) << 24) | ((u32_t)(240 & 0xff) << 16) | \
			   ((u32_t)(42 & 0xff) << 8) | (u32_t)(95 & 0xff))

#endif /* __LWBTOPTS_H__ */
