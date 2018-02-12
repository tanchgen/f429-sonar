/*
 * main.h
 *
 *  Created on: 01 февр. 2018 г.
 *      Author: jet
 */

#ifndef MAIN_H_
#define MAIN_H_

#if defined (__GNUC__)
  #ifndef __weak
    #define __weak          __attribute__((weak))
  #endif  /* Weak attribute */
  #ifndef __packed
    #define __packed        __attribute__((__packed__))
  #endif  /* Packed attribute */
#endif

#include <stdint.h>


#define OUTBUF_SIZE     1432

extern uint8_t rxBuf[];
extern uint8_t * myMemPbuf;
extern struct pbuf_custom myPbuf;
extern volatile uint32_t LocalTime;

#endif /* MAIN_H_ */
