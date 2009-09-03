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


/* uartif.c
 *
 * Implementation of the HCI UART transport layer for Linux
 */


#include "arch/lwbtopts.h"
#include "lwbt/phybusif.h"
#include "lwbt/hci.h"
#include "lwip/debug.h"
#include "lwip/mem.h"

#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#define BAUDRATE B115200
#define _POSIX_SOURCE 1 /* POSIX compliant source */

int fd;

/* Initializes the physical bus interface
*/
int set_speed(int fd, struct termios *ti, int speed)
{
	cfsetospeed(ti, speed);
	cfsetispeed(ti, speed);
	return tcsetattr(fd, TCSANOW, ti);
}

void phybusif_init(const char * port)
{
	struct termios tio;
	int i;

	/* Open the device to be non-blocking (read will return immediatly) */
	printf("opening port \"%s\"\n", port);
	fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd <0) {
		perror(port);
		exit(-1);
	}

	/* Set new port settings */
	tcgetattr(fd,&tio); /* Save current port settings */
	tio.c_cflag = CS8 | CREAD | CLOCAL | CRTSCTS; 
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 0; /* Block until at least this many chars have been received */
	tio.c_cc[VTIME] = 0; /* Timeout value */

	cfsetospeed(&tio, BAUDRATE);
	cfsetispeed(&tio, BAUDRATE);

	if(tcsetattr(fd,TCSANOW,&tio) < 0)
		exit(-1);
	tcflush(fd, TCIOFLUSH);
}

err_t phybusif_reset(struct phybusif_cb *cb) 
{
	/* Init new ctrl block */
	/* Alloc new pbuf. lwIP will handle dealloc */
	if((cb->p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL)) == NULL) {
		LWIP_DEBUGF(PHYBUSIF_DEBUG, ("phybusif_reset: Could not allocate memory for pbuf\n"));
		return ERR_MEM; /* Could not allocate memory for pbuf */
	}
	cb->q = cb->p; /* Make p the pointer to the head of the pbuf chain and q to the tail */

	cb->tot_recvd = 0;
	cb->recvd = 0; 

	cb->state = W4_PACKET_TYPE;
	return ERR_OK;
}

err_t phybusif_input(struct phybusif_cb *cb) 
{
	unsigned char c;
	unsigned char n;

	while((n = read(fd,&c,1))) {
		LWIP_DEBUGF(PHYBUSIF_DEBUG, ("got char: 0x%02x", c));

		if(c < 127 && c > 32)
			LWIP_DEBUGF(PHYBUSIF_DEBUG, ("(%c)\n", c));
		else
			LWIP_DEBUGF(PHYBUSIF_DEBUG, ("\n"));

		switch(cb->state) {
			case W4_PACKET_TYPE:
				switch(c) {
					case HCI_ACL_DATA_PACKET:
						cb->state = W4_ACL_HDR;
						break;
					case HCI_EVENT_PACKET:
						cb->state = W4_EVENT_HDR;
						break;
					default:
						LWIP_DEBUGF(PHYBUSIF_DEBUG, ("phybusif_input: Unknown packet type\n"));
						break;
				}
				break;
			case W4_EVENT_HDR:
				((u8_t *)cb->q->payload)[cb->recvd] = c;
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
						return ERR_OK; /* Since there most likley won't be any more data in the input buffer */
					}
				}
				break;
			case W4_EVENT_PARAM:
				((u8_t *)cb->q->payload)[cb->recvd] = c;
				cb->tot_recvd++;
				cb->recvd++;
				if(cb->recvd == cb->q->len) { /* Pbuf full. alloc and add new tail to chain */
					cb->recvd = 0;
					if((cb->q = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL)) == NULL) {
						LWIP_DEBUGF(PHYBUSIF_DEBUG, ("phybusif_input: Could not allocate memory for event parameter pbuf\n"));
						return ERR_MEM; /* Could not allocate memory for pbuf */
					}
					pbuf_chain(cb->p, cb->q);
					pbuf_free(cb->q);
				}
				if(cb->tot_recvd == cb->evhdr->len) {
					hci_event_input(cb->p); /* Handle incoming event */
					pbuf_free(cb->p);
					phybusif_reset(cb);
					return ERR_OK; /* Since there most likley won't be any more data in the input buffer */
				}
				break;
			case W4_ACL_HDR:
				((u8_t *)cb->q->payload)[cb->recvd] = c;
				cb->tot_recvd++;
				cb->recvd++;
				if(cb->recvd == HCI_ACL_HDR_LEN) {
					cb->aclhdr = cb->p->payload;
					pbuf_header(cb->p, -HCI_ACL_HDR_LEN);
					cb->recvd = cb->tot_recvd = 0;
					if(cb->aclhdr->len > 0) {
						cb->state = W4_ACL_DATA;
					} else {
						LWIP_DEBUGF(PHYBUSIF_DEBUG, ("phybusif_reset: Forward Empty ACL packet to higher layer\n"));
						hci_acl_input(cb->p); /* Handle incoming ACL data */
						pbuf_free(cb->p);
						phybusif_reset(cb);
						return ERR_OK; /* Since there most likley won't be any more data in the input buffer */
					}
				}
				break;
			case W4_ACL_DATA:
				((u8_t *)cb->q->payload)[cb->recvd] = c;
				cb->tot_recvd++;
				cb->recvd++;
				if(cb->recvd == cb->q->len) { /* Pbuf full. alloc and add new tail to chain */
					cb->recvd = 0;
					if((cb->q = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL)) == NULL) {
						LWIP_DEBUGF(PHYBUSIF_DEBUG, ("phybusif_input: Could not allocate memory for ACL data pbuf\n"));
						return ERR_MEM; /* Could not allocate memory for pbuf */
					}
					pbuf_chain(cb->p, cb->q);
					pbuf_free(cb->q);
				}
				if(cb->tot_recvd == cb->aclhdr->len) {
					LWIP_DEBUGF(PHYBUSIF_DEBUG, ("phybusif_input: Forward ACL packet to higher layer\n"));
					hci_acl_input(cb->p); /* Handle incoming ACL data */
					pbuf_free(cb->p);
					phybusif_reset(cb);
					return ERR_OK; /* Since there most likley won't be any more data in the input buffer */
				}
				break;
			default:
				LWIP_DEBUGF(PHYBUSIF_DEBUG, ("phybusif_input: Unknown state\n\n"));
				break;
		}
	}
	return ERR_OK;
}

void phybusif_output(struct pbuf *p, u16_t len) 
{
	static struct pbuf *q;
	static int i;
	static unsigned char *ptr;
	unsigned char c;
	unsigned int bailout;

	/* Send pbuf on UART */
	LWIP_DEBUGF(PHYBUSIF_DEBUG, ("phybusif_output: Send pbuf on UART\n"));
	for(q = p; q != NULL; q = q->next) {
		ptr = q->payload;
		for(i = 0; i < q->len && len; i++) {
			c = *ptr++;
				LWIP_DEBUGF(PHYBUSIF_DEBUG, ("sending %02x\n", c));

			bailout = 0;
			while(write(fd, &c, 1) < 0) //keep trying until it's sent
				if(++bailout == 0xfff)
				{
					perror("Failed to send byte");
					exit(-1);
					break;
				}

			--len;
		}
	}
}

