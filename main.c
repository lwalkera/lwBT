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
//#include "sdp.h"
//#include "rfcomm.h"
//#include "ppp.h"
//#include "nat.h"
#include "lwip/memp.h"
#include "lwip/mem.h"
#include "lwip/sys.h"
#include "lwip/stats.h"

#include <sys/time.h>
#include <stdlib.h>
#include <unistd.h>

#define BT_IP_DEBUG LWIP_DBG_ON

err_t command_complete(void *arg, struct hci_pcb *pcb, u8_t ogf, u8_t ocf, u8_t result)
{
	printf("HCI command complete %d\n", ogf);
	u8_t cod_lap_dun[] = {0x00,0x02,0x00,0x00,0x1E,0x00};
	u8_t cod_lap[] = {0x00,0x03,0x00};
	u8_t devname[] = {'E','I','S','L','A','B',' ','0','0','0',0};
	u8_t n1, n2, n3;
	static u8_t btctrl = 0;

	switch(ogf) {
		case HCI_INFO_PARAM:
			switch(ocf) {
				case HCI_READ_BUFFER_SIZE:
					if(result == HCI_SUCCESS) {
						LWIP_DEBUGF(BT_IP_DEBUG, ("successful HCI_READ_BUFFER_SIZE.\n"));
						//hci_read_bd_addr(read_bdaddr_complete);
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
				case HCI_WRITE_SCAN_ENABLE:
					if(result == HCI_SUCCESS) {
						LWIP_DEBUGF(BT_IP_DEBUG, ("successful HCI_WRITE_SCAN_ENABLE.\n")); 
						hci_cmd_complete(NULL); /* Initialization done, don't come back */
					} else {
						LWIP_DEBUGF(BT_IP_DEBUG, ("Unsuccessful HCI_WRITE_SCAN_ENABLE.\n"));
						return ERR_CONN;
					}
					break;
				case HCI_SET_EVENT_FILTER:
					if(result == HCI_SUCCESS) {
						LWIP_DEBUGF(BT_IP_DEBUG, ("successful HCI_SET_EVENT_FILTER.\n"));
						if(btctrl == 0) {
							hci_write_cod(cod_lap); /*  */
							btctrl = 1;
						} else {
							hci_write_scan_enable(0x03); /* Inquiry and page scan enabled */
						}
					} else {
						LWIP_DEBUGF(BT_IP_DEBUG, ("Unsuccessful HCI_SET_EVENT_FILTER.\n"));
						return ERR_CONN;
					}
					break;
				case HCI_CHANGE_LOCAL_NAME:
					if(result == HCI_SUCCESS) {
						LWIP_DEBUGF(BT_IP_DEBUG, ("Successful HCI_CHANGE_LOCAL_NAME.\n"));
						hci_write_page_timeout(0x4000); /* 10.24s */
					} else {
						LWIP_DEBUGF(BT_IP_DEBUG, ("Unsuccessful HCI_CHANGE_LOCAL_NAME.\n"));
						return ERR_CONN;
					}
					break;
				case HCI_WRITE_COD:
					if(result == HCI_SUCCESS) {
						LWIP_DEBUGF(BT_IP_DEBUG, ("Successful HCI_WRITE_COD.\n"));
						/*n1 = (u8_t)(bt_ip_state.bdaddr.addr[0] / 100);
						n2 = (u8_t)(bt_ip_state.bdaddr.addr[0] / 10) - n1 * 10;
						n3 = bt_ip_state.bdaddr.addr[0] - n1 * 100 - n2 * 10;
						devname[7] = '0' + n1;
						devname[8] = '0' + n2;
						devname[9] = '0' + n3;*/
						hci_change_local_name(devname, sizeof(devname));
					} else {
						LWIP_DEBUGF(BT_IP_DEBUG, ("Unsuccessful HCI_WRITE_COD.\n"));
						return ERR_CONN;
					}
					break;
				case HCI_WRITE_PAGE_TIMEOUT:
					if(result == HCI_SUCCESS) {
						LWIP_DEBUGF(BT_IP_DEBUG, ("successful HCI_WRITE_PAGE_TIMEOUT.\n"));
						hci_cmd_complete(NULL); /* Initialization done, don't come back */
						hci_connection_complete(NULL);//acl_conn_complete);
						LWIP_DEBUGF(BT_IP_DEBUG, ("Initialization done.\n"));
						LWIP_DEBUGF(BT_IP_DEBUG, ("Discover other Bluetooth devices.\n"));
						hci_inquiry(0x009E8B33, 0x04, 0x01, NULL);//inquiry_complete); //FAILED????
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


//	netif_init();
//	ip_init();
//	udp_init();
//	tcp_init();
//	printf("TCP/IP initialized.\n");
	lwbt_memp_init();
	phybusif_init(argv[1]);
	if(hci_init() != ERR_OK) {
		printf("HCI initialization failed!");
		exit(-1);
	}
	l2cap_init();

	hci_cmd_complete(command_complete);
	hci_reset();
//	sdp_init();
//	rfcomm_init();
//	ppp_init();
	printf("Bluetooth initialized.\n");

	//echo_init();
//	httpd_init();
//	printf("Applications started.\n");

	cb = malloc(sizeof(struct phybusif_cb));
	phybusif_reset(cb);

	gettimeofday(&bttv, &tz); /* Initialize Bluetooth timer (1s) */
	gettimeofday(&tcptv, &tz); /* Initialize TCP timer (TCP_TMR_INTERVAL) */

	/* Host controller initialization for DTs according to LAN access point (LAP) and dial up networking (DUN) profile */
	//bt_ip_start(NULL);

	while(1) {
		phybusif_input(cb); /* Check for input */

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
			//rfcomm_tmr();
			//ppp_tmr();
			//nat_tmr();
			if(++btiptmr == 240) { /* Akes server special */
			//	bt_ip_tmr();
				btiptmr = 0;
			}
		}
	}

	return 0;
}
/*-----------------------------------------------------------------------------------*/








