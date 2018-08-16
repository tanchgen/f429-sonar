/**
  ******************************************************************************
  * @file    udp_echoserver.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   UDP echo server
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "eth.h"

#include "dac_adc.h"
#include "my_time.h"

#include "stm32f4x7_eth_bsp.h"
#include "tcp_srv.h"
#include "netconf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define UDP_SERVER_PORT    7      /* define the UDP local connection port */
#define UDP_CLIENT_PORT    8000   /* define the UDP remote connection port */

extern tNetCfg  localNetCfg;

uint8_t * myMemPbuf;
struct pbuf_custom myPbuf;
struct udp_pcb *upcb;
struct pbuf *outp;

BCST_MSG bcst_msg;

/* Private function prototypes -----------------------------------------------*/
void udpRecvCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port);
void udpDmaInit( void );
struct pbuf * outBufInit( void );

/* Private functions ---------------------------------------------------------*/

void ethInit( void ){
  /* configure ethernet */
  ETH_BSP_Config();
  /* Initilaize the LwIP stack */
  LwIP_Init();

}

/**
  * @brief  Initialize the server application.
  * @param  None
  * @retval None
  */
void udpInit(void){
   err_t err;
   
   /* Create a new UDP control block  */
   upcb = udp_new();
   
   if (upcb == NULL ) {
//     /* Bind the upcb to the UDP_PORT port */
//     /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
//      err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);
//
//      if(err == ERR_OK) {
//        /* Set a receive callback for the upcb */
//        udp_recv(upcb, udpRecvCallback, NULL);
//      }
//      else {
//        printf("can not bind pcb");
//      }
//   }
     printf("can not create pcb");
   } 
   else {
      err = udp_bind(upcb, (ip_addr_t *)&(localNetCfg.net_hw_ip), UDP_SERVER_PORT);

      if(err == ERR_OK) {
        /* Set a receive callback for the upcb */
        udp_recv(upcb, udpRecvCallback, NULL);
      }
      else {
        printf("can not bind pcb");
      }
     udp_connect( upcb, (ip_addr_t *)&(localNetCfg.net_dst_ip), localNetCfg.net_dst_udp_port );
     outp = outBufInit();
     udpDmaInit();
   }
}

/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
void udpRecvCallback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
  // TODO: Обработка принятого пакета: Парсинг команд и выполнение их


  (void)arg;
  (void)upcb;
  (void)addr;
  (void)port;
#if 0
  uint32_t tout = LocalTime + 1000;

  if( strstr((p->payload), "oo") == NULL ){
    return;
  }

  // Connect to the remote client
  udp_connect(upcb, addr, port);

  // Tell the client that we have accepted it
  while( tout > LocalTime){
    struct pbuf *outp;

    outp = pbuf_alloced_custom( PBUF_TRANSPORT, OUTBUF_SIZE, PBUF_RAW,
                             &myPbuf, myMemPbuf, OUTBUF_SIZE+58 );
    DMA2_Stream3->M0AR = (uint32_t)outp->payload;
    DMA2_Stream3->NDTR = 1432/4;
    DMA2_Stream3->CR |= DMA_SxCR_EN;
    while( (DMA2->LISR & DMA_LISR_TCIF3) == 0 )
    {}
    DMA2->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3;
    udp_send(upcb, outp);
    pbuf_free(outp);
  }

  // free the UDP connection, so we can accept new clients
  udp_disconnect(upcb);
#endif
  // Free the p buffer
  pbuf_free(p);
   
}

void udpAdcDataSend( uint8_t * data, uint16_t len ){
  struct pbuf *outp;

  outp = pbuf_alloced_custom( PBUF_TRANSPORT, OUTBUF_SIZE, PBUF_RAW,
                           &myPbuf, myMemPbuf, OUTBUF_SIZE+58 );
  DMA2_Stream3->PAR = (uint32_t)data;
  DMA2_Stream3->M0AR = (uint32_t)outp->payload;
  DMA2_Stream3->NDTR = len/4;
  DMA2_Stream3->CR |= DMA_SxCR_EN;
  while( (DMA2->LISR & DMA_LISR_TCIF3) == 0 )
  {}
  DMA2->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3;
  udp_send(upcb, outp);
  pbuf_free(outp);

}

void udpDmaInit( void ){
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  DMA2_Stream3->CR = 0;
  DMA2_Stream3->CR |= DMA_SxCR_MBURST_0 | DMA_SxCR_PBURST_0 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;
  DMA2_Stream3->CR |= DMA_SxCR_DIR_1 | DMA_SxCR_PINC | DMA_SxCR_MINC | DMA_SxCR_TCIE | DMA_SxCR_TEIE;
  // Адрес данных АЦП как источник данных для DMA для отправки
  DMA2_Stream3->PAR = (uint32_t)adcData;
  DMA2_Stream3->FCR = DMA_SxFCR_FTH;

  NVIC_EnableIRQ( DMA2_Stream3_IRQn );
  NVIC_SetPriority( DMA2_Stream3_IRQn, 1 );
}

struct pbuf * outBufInit( void ){
  struct pbuf *p;
  if( (myMemPbuf = malloc( OUTBUF_SIZE + 58 )) == NULL){
    p = NULL;
  }
  else {
    myPbuf.custom_free_function = (pbuf_free_custom_fn)pbuf_free;
    p = pbuf_alloced_custom( PBUF_TRANSPORT, OUTBUF_SIZE, PBUF_RAW,
                             &myPbuf, myMemPbuf, OUTBUF_SIZE+58 );
    if( p == NULL){
      free(myMemPbuf);
    }
  }

  return p;
}

void udpTransfer( uint32_t * data, uint16_t len){
  // Данные  отправляем в pbuf UDP-сервера

  outp = pbuf_alloced_custom( PBUF_TRANSPORT, OUTBUF_SIZE, PBUF_RAW,
                           &myPbuf, myMemPbuf, OUTBUF_SIZE+58 );
  if(outp == NULL){
    return;
  }
  DMA2_Stream3->PAR = (uint32_t)data;
  DMA2_Stream3->M0AR = (uint32_t)outp->payload;
  DMA2_Stream3->NDTR = len/2;   // len - длина буфера в 32-х битных словах
  DMA2_Stream3->CR |= DMA_SxCR_EN;

}

int8_t bcstSend( void ){
  int8_t rec;
  struct udp_pcb * upcb_bcst;
  struct pbuf * p;

  bcst_msg.hw_addr = localNetCfg.net_hw_ip;
  bcst_msg.hw_tcp_port = localNetCfg.net_hw_tcp_port;
  bcst_msg.hw_id[0]  = 'A';
  bcst_msg.hw_id[1]  = 'Q';
  bcst_msg.hw_id[2]  = 'U';
  bcst_msg.hw_id[3]  = 'A';

  upcb_bcst = udp_new();
  p = pbuf_alloc(PBUF_TRANSPORT,sizeof(BCST_MSG),PBUF_RAM);

  p->payload = &bcst_msg;

  rec = udp_sendto(upcb_bcst,p,IP_ADDR_BROADCAST,9999);
  for(uint8_t i = 0; i < 4; i++ ){
    mDelay(1000);
    rec = udp_sendto(upcb_bcst,p,IP_ADDR_BROADCAST,9999);
  }
  pbuf_free(p);

  return rec;
}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
