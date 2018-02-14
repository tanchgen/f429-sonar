/*
 * eth.h
 *
 *  Created on: 11 февр. 2018 г.
 *      Author: jet
 */

#ifndef ETH_H_
#define ETH_H_

#include "lwip/udp.h"

#define OUTBUF_SIZE     1432

extern struct pbuf *outp;
extern struct udp_pcb *upcb;

void udpServerInit(void);
void udpTransfer( uint32_t * data, uint16_t len);

#endif /* ETH_H_ */
