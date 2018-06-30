/*
 * tcp_srv.h
 *
 *  Created on: 25 апр. 2018 г.
 *      Author: jet
 */

#include "errors.h"
#include "lwip/err.h"

#ifndef TCP_SRV_H_
#define TCP_SRV_H_

#define MAX_CHANNELS_NUM  1
#define DPM_SECTIONS_MAX  128

#define USART_TX_MSG_MAX_SIZE 512
#define UART_INTERFACE_NUM_MAX  1

#define VARU_COEFFS_NUM   2000

enum {
  CMD_DIR_READ,
  CMD_DIR_WRITE,
};

enum {
  CMD_CHANNEL_CFG = 0,
  CMD_VARU_CFG = 1,
  CMD_PWR_LEVEL = 2,
  CMD_CHANNEL_MODE = 3,
  CMD_STOP_ALL = 4,
  CMD_GET_SW_VER = 5,
  CMD_EXT_SYNC = 6,
  CMD_USART_ACTIVATE = 7,
  CMD_USART_DEACTIVATE = 8,
  CMD_USART_TX = 9,
  CMD_EXT_SYNC_INPUT = 10,
  CMD_GAIN_CFG = 11,
  CMD_PERFORM_SINGLE_SHOT = 12,
  CMD_PRESSURE = 20,
  CMD_NET_CFG_SOCKET_HW = 30,
  CMD_SET_FREQ_SAMPLING = 40
};

//!CMD_NET_CFG
// #define CMD_NET_CFG_SOCKET_HW 30
// #define CMD_SET_FREQ_SAMPLING 40

#define EXT_SYNC_OFF  0
#define EXT_SYNC_ON   1

typedef enum {
  DDS_MODE_STOP = 0,
  DDS_MODE_OP_CTRL = 1,
  DDS_MODE_RX_CTRL = 2,
} eOpMode;

typedef struct
{
  u32 net_hw_ip;
  u32 net_hw_gw;
  u32 net_hw_mask;
  u32 net_dst_ip;
  u16 net_hw_tcp_port;
  u16 net_dst_udp_port;
  uint8_t net_hw_mac[6];
  u16 reserved;
} tNetCfg;


//typedef struct {
//  uint32_t mainMode;    // Режим работы: 0 - Работа циклическая, 1- разовый цикл от внешнего запуска
//  uint32_t T1Main;      // Период первого таймера главного цикла сонара
//  uint32_t tGen;        // Длительность генерации импульсов излучателя
//  uint32_t fgen;        // Частота излучателя
//  uint32_t tDt;         // Длительность задержек Dead-time ШИМ и tPower-ЩИМ
//  uint32_t kpd;         // Мощность передатчика излучателя
//  uint32_t fAdc;        // Частота преобразования ADC
//  uint32_t TDac;        // Период DAC
//  eOpMode opMode;       // Основной режим работы
//} tTims;
//

typedef struct {
  u16 channel:14;
  u16 mode:2;             // 0- все выключено, 1- все включено, 2- передатчик выкл., приемник - вкл.
  u16 period;             // T1Main - Период первого таймера главного цикла сонара
  struct {
    u8 impulse_type;
    u8 unused;
    u32 freq_carrier;
    union {
      struct {
        u32 tau_d;
        u32 freq_start;     // fgen - Частота излучателя
        u32 freq_stop;

      }fm;
      struct {
        u32 freq;
        u8 nSections;
        struct
        {
          u8 phase;
          u8 periods;
        }section[DPM_SECTIONS_MAX];
      } dpm;
    } impulse;
  } dds;
}DDS_CFG;

typedef struct
{
  u8 channel;
  u8 mode;
}DDS_MODE;

typedef struct
{
  u8 channel;
  u8 level;         // kpd - Мощность передатчика излучателя
}TXR_PWR_CFG;

typedef struct
{
  u8 channel;
  u8 level;  // ВАРУ - Значение = 0-14
}GAIN_CFG;

typedef struct
{
  u8 iVaru;
  u16 tab[VARU_COEFFS_NUM];
}VARU_CFG;

typedef struct
{
  u8 state;
  u16 period_ms;
}EXT_SYNC_CFG;

typedef struct
{
  u16 USART_InterfaceNumber;
  u32 USART_BaudRate;
  u16 USART_WordLength;
  u16 USART_StopBits;
  u16 USART_Parity;
  u16 USART_HardwareFlowControl;
}USART_PHY_CFG;

typedef struct
{
  u16 USART_InterfaceNumber;
  u16 msg_len;
  u8 msg_buf[USART_TX_MSG_MAX_SIZE];
}USART_TX_MSG;


typedef struct
{
  struct
  {
    uint32_t  dir:1;
    uint32_t  reserved1:7;
    uint32_t  cmd:8;
    uint32_t  err:8;
    uint32_t  reserved2:8;
  }head;
  union
  {
    DDS_CFG dds_cfg;
    VARU_CFG varu_cfg;
    GAIN_CFG gain_cfg;
    TXR_PWR_CFG pwr_cfg;
    DDS_MODE dds_mode;
    u8 byte;
    u16 word;
    u32 dword;
    float fpValue;
    EXT_SYNC_CFG ext_sync_cfg;
    USART_PHY_CFG usart_hw_cfg;
    USART_TX_MSG usart_msg;
    tNetCfg net_cfg;
    u32 freq_sampling;
  }data;
}CFG_DATA;

extern  tNetCfg  localNetCfg;

void localNetAddressInit(tNetCfg * pNetCfg);
err_t serverSetup(void);

#endif /* TCP_SRV_H_ */
