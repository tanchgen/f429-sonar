/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   Main program body
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
#include <stddef.h>
#include <stdlib.h>
#include "stm32f4x7_eth.h"
#include "stm32f4x7_eth_bsp.h"
#include "netconf.h"
#include "lwip/pbuf.h"
#include "main.h"
#include "udp_echoserver.h"
#include "serial_debug.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

__attribute__((__aligned__(4))) uint8_t rxBuf[OUTBUF_SIZE+1];
uint8_t * myMemPbuf;
struct pbuf_custom myPbuf;

__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;

RCC_ClocksTypeDef RCC_Clocks;

void dmaInit( void );
struct pbuf * outBufInit( void );

void SetSYSCLK_180(void);

/* Private function prototypes -----------------------------------------------*/
void bufInit( void );

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void){
  __attribute__((__aligned__(4))) uint8_t tmpBuf[1433];
  struct pbuf *outp;

  SetSYSCLK_180();
  SysTick_Config( SystemCoreClock/1000 );
//  SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

#ifdef SERIAL_DEBUG
  DebugComPort_Init();
#endif
  
  bufInit();
  dmaInit();

  DMA2_Stream3->M0AR = (uint32_t)tmpBuf;
  DMA2_Stream3->NDTR = 1432/4;
  DMA2_Stream3->CR |= DMA_SxCR_EN;
  while( (DMA2->LISR & DMA_LISR_TCIF3) == 0 )
  {}
  DMA2->LIFCR |= DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3;
  // configure ethernet
  ETH_BSP_Config();

  // Initilaize the LwIP stack
  LwIP_Init();

  // UDP echoserver
  udp_echoserver_init();
  
  outp = outBufInit();
  if( outp == NULL ){
    while(1)
    {}
  }
    
  /* Infinite loop */
  while (1)
  {  
    /* check if any packet received */
    if (ETH_CheckFrameReceived())
    { 
      /* process received ethernet packet */
      LwIP_Pkt_Handle();
    }
    /* handle periodic timers for LwIP */
    LwIP_Periodic_Handle(LocalTime);
  }   
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: number of 10ms periods to wait for.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Capture the current local time */
  timingdelay = LocalTime + nCount;  

  /* wait until the desired delay finish */  
  while(timingdelay > LocalTime)
  {     
  }
}

void SetSYSCLK_180(void) {

  RCC->APB1ENR |= RCC_APB1Periph_PWR;

  FLASH->ACR |= 0x5;

  PWR->CR |= PWR_CR_VOS;

  PWR->CR |= PWR_CR_ODEN;
  while( (PWR->CSR & PWR_CSR_ODRDY) == 0 )
  {}

  PWR->CR |= PWR_CR_ODSWEN;
  while( (PWR->CSR & PWR_CSR_ODSWRDY) == 0 )
  {}

  RCC->CR |= (0x10 << 3);  // RCC_CR_HSITRIM = 0x10
//  RCC->CR |= RCC_CR_HSION;

   /* Wait till HSI is ready */
//  while( (RCC->CR & RCC_CR_HSIRDY) == 0)
//  {}

  RCC->CR |= RCC_CR_HSEON;
  // Wait till HSE is ready
  while( (RCC->CR & RCC_CR_HSERDY) == 0)
  {}

  RCC->CR &= ~RCC_CR_PLLON;
   /* Wait till PLL is ready */
  while( (RCC->CR & RCC_CR_PLLRDY) != 0)
  {}

  RCC->CR &= ~RCC_CR_PLLI2SON;
   /* Wait till PLL is ready */
  while( (RCC->CR & RCC_CR_PLLI2SRDY) != 0)
  {}
  /* Configure PLL clock to have:
  PLL_VCO = (HSE_VALUE / PLL_M) * PLL_N = 360 MHz
  SYSCLK = PLL_VCO / PLL_P = 180 MHz
  USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ = ~51 MHz (USB is not used)
  */
  //PLLM = 16, PLLN = 360, PLLP = 2, PLLQ = 7
  RCC->PLLCFGR = 50 | (360L << 6) | (((2 >> 1) -1) << 16) | 7 << 24 | (RCC_PLLSource_HSE);

  /* Configure PLL clock to have:
  PLLI2S_VCO = (HSI_VALUE / PLL_M) * PLLI2S_N = 200 MHz
  PLLI2S_CLK = PLLI2S_VCO / PLL_R = 50 MHz
  USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ = ~51 MHz (USB is not used)
  */
  //PLL_M = 16, PLLI2S_R = 4, PLLI2S_N = 200
  RCC->PLLI2SCFGR = (200L << 6) | (4 << 28) | (4 << 24);

  RCC->CR |= RCC_CR_PLLON;
   /* Wait till PLL is ready */
  while( (RCC->CR & RCC_CR_PLLRDY) == 0)
  {}

  RCC->CR |= RCC_CR_PLLI2SON;
   /* Wait till PLL is ready */
  while( (RCC->CR & RCC_CR_PLLI2SRDY) == 0)
  {}

  // HCLK = SYSCLK/1
  RCC->CFGR &= ~RCC_CFGR_HPRE;
  // PCLK1 = HCLK/4
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV4;
  // PCLK2 = HCLK/4\2
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE2_DIV2;

  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
  while( (RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL )
  {}

  // MCO2 source - PLLI2S
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_MCO2PRE);// | RCC_MCO2Div_4;
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_MCO2) | RCC_MCO2Source_PLLI2SCLK;
//  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_MCO2) | RCC_MCO2Source_HSE;

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));

  // Update SystemCoreClock global variable value
  SystemCoreClock = 180000000;
  RCC_GetClocksFreq( &RCC_Clocks );
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  (void)line;
  (void)file;
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif


void bufInit( void ){
  uint16_t i;
  uint8_t ch;

  for( i = 0, ch = 0x21; i < 1431; i++, ch++ ){
    rxBuf[i] = ch;
    if(ch == 0x7E){
      ch = 0x20;
    }
  }
  rxBuf[i++] = '\n';
  rxBuf[i] = '\0';
}

void dmaInit( void ){
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
  DMA2_Stream3->CR = 0;
  DMA2_Stream3->CR |= DMA_SxCR_MBURST_0 | DMA_SxCR_PBURST_0 | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1;
  DMA2_Stream3->CR |= DMA_SxCR_DIR_1 | DMA_SxCR_PINC | DMA_SxCR_MINC;
  DMA2_Stream3->PAR = (uint32_t)rxBuf;
  DMA2_Stream3->FCR = DMA_SxFCR_FTH;
}

struct pbuf * outBufInit( void ){
  uint8_t * tmpMyp;
  struct pbuf *p;
  if( (myMemPbuf = malloc( OUTBUF_SIZE + 58 )) == NULL){
    p = NULL;
  }
  else {
    myPbuf.custom_free_function = (pbuf_free_custom_fn)pbuf_free;
    p = pbuf_alloced_custom( PBUF_TRANSPORT, OUTBUF_SIZE, PBUF_RAW,
                             &myPbuf, myMemPbuf, OUTBUF_SIZE+58 );
    if( p == NULL){
      free(tmpMyp);
    }
  }

  return p;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
