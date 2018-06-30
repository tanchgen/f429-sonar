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

typedef struct
{
  u8 hw_id[4];
  u32 hw_addr;
  u16 hw_tcp_port;
} BCST_MSG;

extern BCST_MSG bcst_msg;

extern struct pbuf *outp;
extern struct udp_pcb *upcb;

void ethInit( void );
void udpInit(void);
void udpServerInit(void);
void udpTransfer( uint32_t * data, uint16_t len);
int8_t bcstSend( void );

#endif /* ETH_H_ */
