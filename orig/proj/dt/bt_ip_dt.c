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
 /* bt_ip_dt.c
 *
 * This is a control application that initialises a host controller and connects to a 
 * network as a DT through a DUN or LAP enabled device.
 */
/*-----------------------------------------------------------------------------------*/

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

#include "stdlib.h"

#define BT_IP_DEBUG DBG_ON /* Control debug messages */

enum bt_profile {
  LAP_PROFILE, DUN_PROFILE
};

struct bt_state {
  enum bt_profile profile;
  struct tcp_pcb *tcppcb;
  struct bd_addr bdaddr;
  u8_t cn;
} bt_ip_state;

/* Forward declarations */
err_t rfcomm_connected(void *arg, struct rfcomm_pcb *tpcb, err_t err);
err_t l2cap_connected(void *arg, struct l2cap_pcb *pcb, u16_t result, u16_t status);
err_t inquiry_complete(void *arg, struct hci_pcb *pcb, struct hci_inq_res *ires, u16_t result);
err_t command_complete(void *arg, struct hci_pcb *pcb, u8_t ogf, u8_t ocf, u8_t result);
err_t pin_req(void *arg, struct bd_addr *bdaddr);

err_t http_accept(void *arg, struct tcp_pcb *pcb, err_t err);

static const u8_t lap_service_record[] = {
  0x35, 0x8, 
  0x9, 0x0, 0x0, 0xa, 0x0, 0x0, 0xff, 0xff, /* Service record handle attribute */
  0x35, 0x8, 
  0x9, 0x0, 0x1, 0x35, 0x3, 0x19, 0x11, 0x2, /* Service class ID list attribute */
  0x35, 0x11,
  0x9, 0x0, 0x4, 0x35, 0xc, 0x35, 0x3, 0x19, 0x1, 0x0, 0x35, 0x5, 0x19, 0x0, 0x3, 0x8, 0x1 /* Protocol descriptor list attribute */
};

u8_t bt_ip_netifn;

/*-----------------------------------------------------------------------------------*/
/* 
* bt_ip_start():
*
* Called by the main application to initialize and connect to a network
*
*/
/*-----------------------------------------------------------------------------------*/
void
bt_ip_start(void)
{
  hci_reset_all();
  l2cap_reset_all();
  sdp_reset_all();
  rfcomm_reset_all();
  ppp_reset_all();

  LWIP_DEBUGF(BT_IP_DEBUG, ("bt_ip_start\n"));

  hci_cmd_complete(command_complete);
  hci_pin_req(pin_req);
  bt_ip_state.tcppcb = NULL;
  bt_ip_netifn = 0;
  hci_reset();
}
/*-----------------------------------------------------------------------------------*/
/* 
* bt_ip_tmr():
*
* Called by the main application to initialize and connect to a network
*
*/
/*-----------------------------------------------------------------------------------*/
void
bt_ip_tmr(void)
{
  u8_t update_cmd[12];

  update_cmd[0] = 1;
  update_cmd[1] = 0;
  update_cmd[2] = bt_ip_state.bdaddr.addr[5];
  update_cmd[3] = bt_ip_state.bdaddr.addr[4]; 
  update_cmd[4] = bt_ip_state.bdaddr.addr[3];
  update_cmd[5] = bt_ip_state.bdaddr.addr[2]; 
  update_cmd[6] = bt_ip_state.bdaddr.addr[1];
  update_cmd[7] = bt_ip_state.bdaddr.addr[0];
  update_cmd[8] = 0x00;
  update_cmd[9] = 0x00;
  update_cmd[10] = 0x00;
  update_cmd[11] = 0x00;

  LWIP_DEBUGF(BT_IP_DEBUG, ("bt_ip_tmr: Update cmd bd address: 0x%x:0x%x:0x%x:0x%x:0x%x:0x%x\n", update_cmd[2], update_cmd[3], update_cmd[4], update_cmd[5], update_cmd[6], update_cmd[7]));
   
  if(bt_ip_state.tcppcb != NULL) {
    tcp_write(bt_ip_state.tcppcb, &update_cmd, 12, 1);
  }
}
/*-----------------------------------------------------------------------------------*/
/* 
* ppp_is_disconnected():
*
* Called by PPP when the remote PPP protocol or upper layer was disconnected.
* Disconnects the IP layer.
*
*/
/*-----------------------------------------------------------------------------------*/
err_t 
ppp_is_disconnected(void *arg, struct ppp_pcb *pcb, u16_t proto, err_t err)
{
  //TODO: CHECK PROTOCOL
  LWIP_DEBUGF(BT_IP_DEBUG, ("ppp_disconnected\n"));
  ppp_close(pcb);
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/* 
* rfcomm_disconnected():
*
* Called by RFCOMM when the remote RFCOMM protocol or upper layer was disconnected.
* Disconnects the PPP protocol.
*
*/
/*-----------------------------------------------------------------------------------*/
err_t
rfcomm_disconnected(void *arg, struct rfcomm_pcb *pcb, err_t err) 
{
  err_t ret = ERR_OK;

  LWIP_DEBUGF(BT_IP_DEBUG, ("rfcomm_disconnected: CN = %d\n", rfcomm_cn(pcb)));
  if(rfcomm_cn(pcb) != 0) {
    ret = ppp_lp_disconnected(pcb);
  }
  rfcomm_close(pcb);
  
  return ret;
}
/*-----------------------------------------------------------------------------------*/
/* 
* l2cap_disconnected_ind():
*
* Called by L2CAP to indicate that remote L2CAP protocol disconnected.
* Disconnects the RFCOMM protocol and the ACL link before it initializes a search for 
* other devices.
*
*/
/*-----------------------------------------------------------------------------------*/
err_t
l2cap_disconnected_ind(void *arg, struct l2cap_pcb *pcb, err_t err)
{
  err_t ret = ERR_OK;

  LWIP_DEBUGF(BT_IP_DEBUG, ("l2cap_disconnected_ind: L2CAP disconnected\n"));

  if(pcb->psm == SDP_PSM) { 
    sdp_lp_disconnected(pcb);
    l2cap_close(pcb);
  } else if(pcb->psm == RFCOMM_PSM) {
    ret = rfcomm_lp_disconnected(pcb);
    /* We can do this since we know that we are the only channel on the ACL link. If ACL link already is 
     down we get an ERR_CONN returned */
    hci_disconnect(&(pcb->remote_bdaddr), HCI_OTHER_END_TERMINATED_CONN_USER_ENDED);
    l2cap_close(pcb);
    bt_ip_start();
  }
  
  return ret;
}
/*-----------------------------------------------------------------------------------*/
/* 
* bluetoothif_init():
*
* Called by lwIP to initialize the lwBT network interface.
*
*/
/*-----------------------------------------------------------------------------------*/
err_t
bluetoothif_init(struct netif *netif)
{
  netif->name[0] = 'b';
  netif->name[1] = '0' + bt_ip_netifn++;
  netif->output = ppp_netif_output;

  netif->state = NULL;
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/* 
* tcp_connected():
*
* Called by TCP when a connection has been established.
* Connects to a remote gateway and give the TCP connection to the HTTP server 
* application
*
*/
/*-----------------------------------------------------------------------------------*/
err_t
tcp_connected(void *arg, struct tcp_pcb *pcb, err_t err)
{
  u8_t update_cmd[12];

  LWIP_DEBUGF(BT_IP_DEBUG, ("tcp_connected\n"));
  
  update_cmd[0] = 1;
  update_cmd[1] = 0;

  update_cmd[2] = bt_ip_state.bdaddr.addr[5];
  update_cmd[3] = bt_ip_state.bdaddr.addr[4]; 
  update_cmd[4] = bt_ip_state.bdaddr.addr[3];
  update_cmd[5] = bt_ip_state.bdaddr.addr[2]; 
  update_cmd[6] = bt_ip_state.bdaddr.addr[1];
  update_cmd[7] = bt_ip_state.bdaddr.addr[0];

  LWIP_DEBUGF(BT_IP_DEBUG, ("tcp_connected: bd address: 0x%x:0x%x:0x%x:0x%x:0x%x:0x%x\n", bt_ip_state.bdaddr.addr[0], bt_ip_state.bdaddr.addr[1], bt_ip_state.bdaddr.addr[2], bt_ip_state.bdaddr.addr[3], bt_ip_state.bdaddr.addr[4], bt_ip_state.bdaddr.addr[5]));
  LWIP_DEBUGF(BT_IP_DEBUG, ("tcp_connected: Update cmd bd address: 0x%x:0x%x:0x%x:0x%x:0x%x:0x%x\n", update_cmd[2], update_cmd[3], update_cmd[4], update_cmd[5], update_cmd[6], update_cmd[7]));

  update_cmd[8] = 0x00;
  update_cmd[9] = 0x00;
  update_cmd[10] = 0x00;
  update_cmd[11] = 0x00;
  
  tcp_write(pcb, &update_cmd, 12, 1);

  LWIP_DEBUGF(BT_IP_DEBUG, ("tcp_connected: Update command sent\n"));
  
  bt_ip_state.tcppcb = pcb;
  
  return http_accept((void *)&(bt_ip_state.bdaddr), pcb, ERR_OK);
}
/*-----------------------------------------------------------------------------------*/
/*
 * ppp_connected():
 *
 * Called by PPP when a LCP and IPCP connection has been established.
 * Connects to a given TCP host.
 *
 */
/*-----------------------------------------------------------------------------------*/
err_t
ppp_connected(void *arg, struct ppp_pcb *pcb, err_t err)
{
  struct tcp_pcb *tcppcb;
  struct ip_addr ipaddr;

  LWIP_DEBUGF(BT_IP_DEBUG, ("ppp_connected: err = %d\n", err));
  
  /* return ppp_echo(pcb, NULL); */

  if(err != ERR_OK) {
    netif_remove(pcb->bluetoothif);
    pcb->bluetoothif = NULL;
    //TODO: RESTART??
    return ERR_OK;
  }

  tcppcb = tcp_new();
  IP4_ADDR(&ipaddr, 130,240,45,234);
  return tcp_connect(tcppcb, &ipaddr, 8989, tcp_connected);
}
/*-----------------------------------------------------------------------------------*/
/*
 * at_input():
 *
 * Called by RFCOMM during the DUN profile GPRS connection attempt.
 * When a GPRS connection is established, PPP is connected.
 *
 */
/*-----------------------------------------------------------------------------------*/
u8_t at_state;
err_t
at_input(void *arg, struct rfcomm_pcb *pcb, struct pbuf *p, err_t err) 
{
  struct ppp_pcb *ppppcb;
  struct ip_addr ipaddr, netmask, gw;
  struct netif *netif;;
  
  //  u16_t i;
  struct pbuf *q;
  
  //for(q = p; q != NULL; q = q->next) {
  //  for(i = 0; i < q->len; ++i) {
  //    LWIP_DEBUGF(BT_IP_DEBUG, ("at_input: 0x%x\n",((u8_t *)p->payload)[i]));
  //  }
  //  LWIP_DEBUGF(BT_IP_DEBUG, ("*\n"));
  //}
  
  LWIP_DEBUGF(BT_IP_DEBUG, ("at_input: %s\n", ((u8_t *)p->payload)));
  LWIP_DEBUGF(BT_IP_DEBUG, ("state == %d\n", at_state));
  if(at_state == 0 && ((u8_t *)p->payload)[2] == 'O') {
    //q = pbuf_alloc(PBUF_RAW, sizeof("AT&F\r")-1, PBUF_RAM);
    //((u8_t *)q->payload) = "AT&F\r";
    q = pbuf_alloc(PBUF_RAW, sizeof("ATE1\r"), PBUF_RAM);
    ((u8_t *)q->payload) = "ATE1\r";
    if(rfcomm_cl(pcb)) {
      rfcomm_uih_credits(pcb, 2, q);
    } else {
      rfcomm_uih(pcb, rfcomm_cn(pcb), q);
    }
    pbuf_free(q);
    
    at_state = 1;
  } else if(at_state == 1 && ((u8_t *)p->payload)[2] == 'O') {
    q = pbuf_alloc(PBUF_RAW, sizeof("AT+cgdcont=1,\"IP\",\"online.telia.se\"\r"), PBUF_RAM);
    ((u8_t *)q->payload) = "AT+cgdcont=1,\"IP\",\"online.telia.se\"\r";
    if(rfcomm_cl(pcb)) {
      rfcomm_uih_credits(pcb, 2, q);
    } else {
      rfcomm_uih(pcb, rfcomm_cn(pcb), q);
    }
    pbuf_free(q);
   
    at_state = 4;
  } else if(at_state == 4 && ((u8_t *)p->payload)[2] == 'O') {
    q = pbuf_alloc(PBUF_RAW, sizeof("ATD*99***1#\r"), PBUF_RAM);
    ((u8_t *)q->payload) = "ATD*99***1#\r";
    if(rfcomm_cl(pcb)) {
      rfcomm_uih_credits(pcb, 2, q);
    } else {
      rfcomm_uih(pcb, rfcomm_cn(pcb), q);
    }
    pbuf_free(q);
  
    at_state = 5;
  } else if(at_state == 5 && ((u8_t *)p->payload)[2] == 'C') {
    at_state = 6;
    /* Establish a PPP connection */
    if((ppppcb = ppp_new(pcb)) == NULL) {
      LWIP_DEBUGF(BT_IP_DEBUG, ("rfcomm_msc_rsp: Could not alloc PPP pcb\n"));
      return ERR_MEM;
    }
    /* Add PPP network interface to lwIP */
    gw.addr = 0;
    ipaddr.addr = 0;
    IP4_ADDR(&netmask, 255,255,255,0);
    
    netif = netif_add(&ipaddr, &netmask, &gw, NULL, bluetoothif_init, ip_input);
    
    netif_set_default(netif);
    
    ppp_netif(ppppcb, netif);

    rfcomm_recv(pcb, ppp_input);
    ppp_disconnected(ppppcb, ppp_is_disconnected);
    return ppp_connect(ppppcb, ppp_connected);
  }
  pbuf_free(p);

  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/*
 * pin_req():
 *
 * Called by HCI when a request for a PIN code has been received. A PIN code is 
 * required to create a new link key.
 * Replys to the request with the given PIN code
 *
 */
/*-----------------------------------------------------------------------------------*/
err_t 
pin_req(void *arg, struct bd_addr *bdaddr)
{
  LWIP_DEBUGF(BT_IP_DEBUG, ("pin_req\n"));
  return hci_pin_code_request_reply(bdaddr, 4, "1234");
}
/*-----------------------------------------------------------------------------------*/
/*
* link_key_not():
 *
 * Called by HCI when a new link key has been created for the connection
 * Writes the key to the Bluetooth host controller, where it can be stored for future
 * connection attempts.
  *
*/
/*-----------------------------------------------------------------------------------*/
err_t 
link_key_not(void *arg, struct bd_addr *bdaddr, u8_t *key)
{
  LWIP_DEBUGF(BT_IP_DEBUG, ("link_key_not\n"));
  return hci_write_stored_link_key(bdaddr, key); /* Write link key to be stored in the
                                                    Bluetooth host controller */
}
/*-----------------------------------------------------------------------------------*/
/*
 * l2cap_disconnected_cfm():
 *
 * Called by L2CAP to confirm that the L2CAP disconnection request was successful
 *
 */
/*-----------------------------------------------------------------------------------*/
err_t
l2cap_disconnected_cfm(void *arg, struct l2cap_pcb *pcb) 
{
  LWIP_DEBUGF(BT_IP_DEBUG, ("l2cap_disconnected_cfm\n"));
  l2cap_close(pcb);
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/*
 * get_rfcomm_cn():
 *
 * Parse the RFCOMM channel number from an SDP attribute list
 *
*/
/*-----------------------------------------------------------------------------------*/
u8_t
get_rfcomm_cn(u16_t attribl_bc, struct pbuf *attribute_list)
{
  u8_t i;
  for(i = 0; i < attribl_bc; i++) {
    if(((u8_t *)attribute_list->payload)[i] == (SDP_DE_TYPE_UUID | SDP_DE_SIZE_16)) {
      if(ntohs(*((u16_t *)(((u8_t *)attribute_list->payload)+i+1))) == 0x0003) {
	return *(((u8_t *)attribute_list->payload)+i+4);
      }
    }
  }
  return 0;
}
/*-----------------------------------------------------------------------------------*/
/*
 * rfcomm_connected():
 *
 * Called by RFCOMM when a connection attempt response was received.
 * Creates a RFCOMM connection for the channel retreived from SDP.
 * Initializes a search for other devices if the connection attempt failed.
 */
/*-----------------------------------------------------------------------------------*/
err_t
rfcomm_connected(void *arg, struct rfcomm_pcb *pcb, err_t err) 
{
  struct pbuf *p;
  struct ip_addr ipaddr, netmask, gw;
  struct netif *netif;
  struct ppp_pcb *ppppcb;

  if(err == ERR_OK) {
    LWIP_DEBUGF(BT_IP_DEBUG, ("rfcomm_connected. CN = %d\n", rfcomm_cn(pcb)));
    rfcomm_disc(pcb, rfcomm_disconnected);

    if(bt_ip_state.profile == DUN_PROFILE) {
      /* Establish a GPRS connection */
      LWIP_DEBUGF(BT_IP_DEBUG, ("rfcomm_msc_rsp: Establish a GPRS connection\n"));
      rfcomm_recv(pcb, at_input);
      //p = pbuf_alloc(PBUF_RAW, sizeof("ATZ\r")-1, PBUF_RAM);
      //((u8_t *)p->payload) = "ATZ\r";
      p = pbuf_alloc(PBUF_RAW, sizeof("AT\r"), PBUF_RAM);
      ((u8_t *)p->payload) = "AT\r";
      at_state = 0;
      if(rfcomm_cl(pcb)) {
	rfcomm_uih_credits(pcb, 6,  p);
      } else {
	rfcomm_uih(pcb, rfcomm_cn(pcb), p);
      }
      pbuf_free(p);
    } else {
      /* Establish a PPP connection */
      if((ppppcb = ppp_new(pcb)) == NULL) {
	LWIP_DEBUGF(BT_IP_DEBUG, ("rfcomm_msc_rsp: Could not alloc PPP pcb\n"));
	return ERR_MEM;
      }
      
      /* Add PPP network interface to lwIP */
      gw.addr = 0;
      ipaddr.addr = 0;
      IP4_ADDR(&netmask, 255,255,255,0);
      
      netif = netif_add(&ipaddr, &netmask, &gw, NULL, bluetoothif_init, ip_input);
      
      netif_set_default(netif);
      
      ppp_netif(ppppcb, netif);
      
      rfcomm_recv(pcb, ppp_input);
      ppp_disconnected(ppppcb, ppp_is_disconnected);
      return ppp_connect(ppppcb, ppp_connected);
    }
  } else {
    LWIP_DEBUGF(BT_IP_DEBUG, ("rfcomm_connected. Connection attempt failed CN = %d\n", rfcomm_cn(pcb)));
    l2cap_close(pcb->l2cappcb);
    rfcomm_close(pcb);
    bt_ip_start();
  }
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/*
 * sdp_attributes_recv():
 *
 * Can be used as a callback by SDP when a response to a service attribute request or 
 * a service search attribute request was received.
 * Disconnects the L2CAP SDP channel and connects to the RFCOMM one.
 * If no RFCOMM channel was found it initializes a search for other devices.
 */
/*-----------------------------------------------------------------------------------*/
void
sdp_attributes_recv(void *arg, struct sdp_pcb *sdppcb, u16_t attribl_bc, struct pbuf *p)
{
  struct l2cap_pcb *l2cappcb;

  l2ca_disconnect_req(sdppcb->l2cappcb, l2cap_disconnected_cfm);
  /* Get the RFCOMM channel identifier from the protocol descriptor list */
  if((bt_ip_state.cn = get_rfcomm_cn(attribl_bc, p)) != 0) {
    if((l2cappcb = l2cap_new()) == NULL) {
      LWIP_DEBUGF(BT_IP_DEBUG, ("sdp_attributes_recv: Could not alloc L2CAP pcb\n"));
      return;
    }
    LWIP_DEBUGF(BT_IP_DEBUG, ("sdp_attributes_recv: RFCOMM channel: %d\n", bt_ip_state.cn));
    if(bt_ip_state.profile == DUN_PROFILE) {
      l2ca_connect_req(l2cappcb, &(sdppcb->l2cappcb->remote_bdaddr), RFCOMM_PSM, 0, l2cap_connected);
    } else {
      l2ca_connect_req(l2cappcb, &(sdppcb->l2cappcb->remote_bdaddr), RFCOMM_PSM, HCI_ALLOW_ROLE_SWITCH, l2cap_connected);
    }
    
  } else {
    bt_ip_start();
  }
  sdp_free(sdppcb);
}
/*-----------------------------------------------------------------------------------*/
/*
 * l2cap_connected():
 *
 * Called by L2CAP when a connection response was received.
 * Sends a L2CAP configuration request.
 * Initializes a search for other devices if the connection attempt failed.
 */
/*-----------------------------------------------------------------------------------*/
err_t
l2cap_connected(void *arg, struct l2cap_pcb *l2cappcb, u16_t result, u16_t status)
{
  struct sdp_pcb *sdppcb;
  struct rfcomm_pcb *rfcommpcb;

  u8_t ssp[] = {0x35, 0x03, 0x19, 0x11, 0x02}; /* Service search pattern with LAP UUID is default */ 
  err_t ret;

  u8_t attrids[] = {0x35, 0x03, 0x09, 0x00, 0x04}; /* Attribute IDs to search for in data element 
						      sequence form */

  if(result == L2CAP_CONN_SUCCESS) {
    LWIP_DEBUGF(BT_IP_DEBUG, ("l2cap_connected: L2CAP connected pcb->state = %d\n", l2cappcb->state));
    /* Tell L2CAP that we wish to be informed of a disconnection request */
    l2cap_disconnect_ind(l2cappcb, l2cap_disconnected_ind);
    switch(l2cap_psm(l2cappcb)) {
    case SDP_PSM:
      LWIP_DEBUGF(BT_IP_DEBUG, ("l2cap_connected: SDP L2CAP configured. Result = %d\n", result));
      if(bt_ip_state.profile == DUN_PROFILE) {
	LWIP_DEBUGF(BT_IP_DEBUG, ("l2cap_connected: Using DUN profile\n"));
	ssp[4] = 0x03; /* Change service search pattern to contain DUN UUID */
      } else {
	LWIP_DEBUGF(BT_IP_DEBUG, ("l2cap_connected: Using LAP profile\n"));
      }
      
      if((sdppcb = sdp_new(l2cappcb)) == NULL) {
	LWIP_DEBUGF(BT_IP_DEBUG, ("l2cap_connected: Failed to create a SDP PCB\n"));
	return ERR_MEM;
      }
      
      l2cap_recv(l2cappcb, sdp_recv);
      
      ret = sdp_service_search_attrib_req(sdppcb, 0xFFFF, ssp, sizeof(ssp), attrids, sizeof(attrids),
					  sdp_attributes_recv);
      return ret;
    case RFCOMM_PSM:
      LWIP_DEBUGF(BT_IP_DEBUG, ("l2cap_connected: RFCOMM L2CAP configured. Result = %d CN = %d\n", result, bt_ip_state.cn));
      l2cap_recv(l2cappcb, rfcomm_input);

      if((rfcommpcb = rfcomm_new(l2cappcb)) == NULL) {
	LWIP_DEBUGF(BT_IP_DEBUG, ("l2cap_connected: Failed to create a RFCOMM PCB\n"));
	return ERR_MEM;
      }

      hci_link_key_not(link_key_not); /* Set function to be called if a new link key is created */

      return rfcomm_connect(rfcommpcb, bt_ip_state.cn, rfcomm_connected); /* Connect with DLCI 0 */
    default:
      return ERR_VAL;
    }
  } else {
    LWIP_DEBUGF(BT_IP_DEBUG, ("l2cap_connected: L2CAP not connected. Redo inquiry\n"));
    l2cap_close(l2cappcb);
    bt_ip_start();
  }
  
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/*
 * inquiry_complete():
 *
 * Called by HCI when a inquiry complete event was received.
 * Connects to the first device in the list.
 * Initializes a search for other devices if the inquiry failed.
 */
/*-----------------------------------------------------------------------------------*/
err_t
inquiry_complete(void *arg, struct hci_pcb *pcb, struct hci_inq_res *ires, u16_t result)
{
  struct l2cap_pcb *l2cappcb;

  if(result == HCI_SUCCESS) {
    LWIP_DEBUGF(BT_IP_DEBUG, ("successful Inquiry\n"));
    if(ires != NULL) {
      LWIP_DEBUGF(BT_IP_DEBUG, ("Initiate L2CAP connection\n"));
      LWIP_DEBUGF(BT_IP_DEBUG, ("ires->psrm %d\n ires->psm %d\n ires->co %d\n", ires->psrm, ires->psm, ires->co));
      LWIP_DEBUGF(BT_IP_DEBUG, ("ires->bdaddr 0x%x:0x%x:0x%x:0x%x:0x%x:0x%x\n", ires->bdaddr.addr[5], ires->bdaddr.addr[4], ires->bdaddr.addr[3], ires->bdaddr.addr[2], ires->bdaddr.addr[1], ires->bdaddr.addr[0]));

      if((ires->cod[1] & 0x1F) == 0x03) {
	bt_ip_state.profile = LAP_PROFILE;
      } else {
	bt_ip_state.profile = DUN_PROFILE;
      }
      
      if((l2cappcb = l2cap_new()) == NULL) {
	LWIP_DEBUGF(BT_IP_DEBUG, ("inquiry_complete: Could not alloc L2CAP pcb\n"));
	return ERR_MEM;
      } 

      if(bt_ip_state.profile == DUN_PROFILE) {
        l2ca_connect_req(l2cappcb, &(ires->bdaddr), SDP_PSM, 0, l2cap_connected);
      } else {
	l2ca_connect_req(l2cappcb, &(ires->bdaddr), SDP_PSM, HCI_ALLOW_ROLE_SWITCH, l2cap_connected);
      }
    } else {
      hci_inquiry(0x009E8B33, 0x04, 0x01, inquiry_complete);
    }
  } else {
    LWIP_DEBUGF(BT_IP_DEBUG, ("Unsuccessful Inquiry.\n"));
    hci_inquiry(0x009E8B33, 0x04, 0x01, inquiry_complete);
  }
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/*
 * acl_wpl_complete():
 *
 * Called by HCI when a successful write link policy settings complete event was
 * received.
 */
/*-----------------------------------------------------------------------------------*/
err_t
acl_wpl_complete(void *arg, struct bd_addr *bdaddr)
{
  hci_sniff_mode(bdaddr, 200, 100, 10, 10);
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/*
 * acl_conn_complete():
 *
 * Called by HCI when a connection complete event was received.
 */
/*-----------------------------------------------------------------------------------*/
err_t
acl_conn_complete(void *arg, struct bd_addr *bdaddr)
{
  //hci_wlp_complete(acl_wpl_complete);
  //hci_write_link_policy_settings(bdaddr, 0x000F);
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/*
 * read_bdaddr_complete():
 *
 * Called by HCI when a read local bluetooth device address complete event was received.
 */
/*-----------------------------------------------------------------------------------*/
err_t
read_bdaddr_complete(void *arg, struct bd_addr *bdaddr)
{
  memcpy(&(bt_ip_state.bdaddr), bdaddr, 6);
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
/*
 * command_complete():
 *
 * Called by HCI when an issued command has completed during the initialization of the
 * host controller.
 * Initializes a search for other devices when host controller initialization is
 * completed.
 */
/*-----------------------------------------------------------------------------------*/
err_t
command_complete(void *arg, struct hci_pcb *pcb, u8_t ogf, u8_t ocf, u8_t result)
{
  u8_t cod_lap_dun[] = {0x00,0x02,0x00,0x00,0x1E,0x00};
    
  switch(ogf) {
  case HCI_INFO_PARAM:
    switch(ocf) {
    case HCI_READ_BUFFER_SIZE:
      if(result == HCI_SUCCESS) {
	LWIP_DEBUGF(BT_IP_DEBUG, ("successful HCI_READ_BUFFER_SIZE.\n"));
	hci_read_bd_addr(read_bdaddr_complete);
      } else {
	LWIP_DEBUGF(BT_IP_DEBUG, ("Unsuccessful HCI_READ_BUFFER_SIZE.\n"));
	return ERR_CONN;
      }
      break;
    case HCI_READ_BD_ADDR:
      if(result == HCI_SUCCESS) {
	LWIP_DEBUGF(BT_IP_DEBUG, ("successful HCI_READ_BD_ADDR.\n"));
	hci_set_event_filter(0x01, 0x01, cod_lap_dun); /* Report only devices with a specific type of CoD */
	
      } else {
	LWIP_DEBUGF(BT_IP_DEBUG, ("Unsuccessful HCI_READ_BD_ADDR.\n"));
	return ERR_CONN;
      }
      break;
    default:
      LWIP_DEBUGF(BT_IP_DEBUG, ("Unknown HCI_INFO_PARAM command complete event\n"));
      break;
    }
    break;
  case HCI_HC_BB_OGF:
    switch(ocf) {
    case HCI_RESET:
      if(result == HCI_SUCCESS) {
	LWIP_DEBUGF(BT_IP_DEBUG, ("successful HCI_RESET.\n")); 
	hci_read_buffer_size();
      } else {
	LWIP_DEBUGF(BT_IP_DEBUG, ("Unsuccessful HCI_RESET.\n"));
	return ERR_CONN;
      }
      break;
    case HCI_SET_EVENT_FILTER:
      if(result == HCI_SUCCESS) {
	LWIP_DEBUGF(BT_IP_DEBUG, ("successful HCI_SET_EVENT_FILTER.\n"));
	hci_write_page_timeout(0x4000); /* 10.24s */
      } else {
	LWIP_DEBUGF(BT_IP_DEBUG, ("Unsuccessful HCI_SET_EVENT_FILTER.\n"));
	return ERR_CONN;
      }
      break;
    case HCI_WRITE_PAGE_TIMEOUT:
      if(result == HCI_SUCCESS) {
	LWIP_DEBUGF(BT_IP_DEBUG, ("successful HCI_WRITE_PAGE_TIMEOUT.\n"));
	hci_cmd_complete(NULL); /* Initialization done, don't come back */
	hci_connection_complete(acl_conn_complete);
	LWIP_DEBUGF(BT_IP_DEBUG, ("Initialization done.\n"));
	LWIP_DEBUGF(BT_IP_DEBUG, ("Discover other Bluetooth devices.\n"));
	hci_inquiry(0x009E8B33, 0x04, 0x01, inquiry_complete); //TODO: FAILED????
      } else {
	LWIP_DEBUGF(BT_IP_DEBUG, ("Unsuccessful HCI_WRITE_PAGE_TIMEOUT.\n"));
	return ERR_CONN;
      }
      break;
    default:
      LWIP_DEBUGF(BT_IP_DEBUG, ("Unknown HCI_HC_BB_OGF command complete event\n"));
      break;
    }
    break;
  default:
    LWIP_DEBUGF(BT_IP_DEBUG, ("Unknown command complete event. OGF = 0x%x OCF = 0x%x\n", ogf, ocf));
    break;
  }
  return ERR_OK;
}
/*-----------------------------------------------------------------------------------*/
