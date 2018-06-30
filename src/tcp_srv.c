/*
 * tcp_srv.c
 *
 *  Created on: 25 апр. 2018 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */
#include <string.h>
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "netconf.h"

#include "main.h"
#include "dac_adc.h"
#include "tim.h"

#include "defines.h"
#include "tcp_srv.h"

extern tTims tims;
extern uint16_t dacData[];

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static char szArg[] = "void arg";
static const u16 s_wSoftVersion = 0x8103;

static DDS_CFG dds_cfg_stored;
static DDS_MODE dds_mode_stored;
static TXR_PWR_CFG dds_pwrCfg_stored;
static GAIN_CFG dds_gain_stored;
static u32 sampling_freq_stored;

u16 GetVaruValueFromGainValue(u16 gain_value);
uint8_t WriteNetCfgToFlash( tNetCfg * netcfg );
uint8_t ReadNetCfgFromFlash( tNetCfg * netCfg);
//uint8_t WriteNetCfgToFlash( tNetCfg * netcfg );

static err_t OnAccept(void *arg, struct tcp_pcb *pcb,err_t err);
static err_t OnReceive(void *arg, struct tcp_pcb *pcb, struct pbuf *p,err_t err);
static void Server(struct tcp_pcb *pcb, uint16_t byteNum );

//static u8 GetDDSCfg( u8 channel,DDS_CFG* pCfg ) {
//  if( channel > MAX_CHANNELS_NUM )
//    return ERR_HW_CHANNEL_NUM;
//  memcpy(&pCfg, &dds_cfg_stored, sizeof(DDS_CFG));
//  return ERR_SUCCESS;
//}
//
//static u8 SaveDDSCfg(u8 channel, DDS_CFG* pCfg, uint16_t bn){
//  uint16_t sz;
//
//  sz = (bn < sizeof(DDS_CFG))? bn: sizeof(DDS_CFG);
//  if( channel > MAX_CHANNELS_NUM )
//    return ERR_HW_CHANNEL_NUM;
//  memcpy(&dds_cfg_stored, &pCfg, sz);
//  return ERR_SUCCESS;
//}
//
//static u8 GetDDSMode(u8 channel,DDS_MODE* pMode ) {
//  if( channel > MAX_CHANNELS_NUM )
//    return ERR_HW_CHANNEL_NUM;
//  memcpy( &pMode, &dds_mode_stored, sizeof(DDS_MODE) );
//  return ERR_SUCCESS;
//}
//
//static u8 SaveDDSMode(u8 channel, DDS_MODE* pMode, uint16_t bn) {
//  uint8_t rec = ERR_SUCCESS;
//
//  if( channel > MAX_CHANNELS_NUM ){
//    rec = ERR_HW_CHANNEL_NUM;
//  }
//  else if( (bn < 2) || (pMode->mode > 2) ){
//    rec = ERR_HW_MODE;
//    dds_mode_stored.mode = DDS_MODE_STOP;
//  }
//  else {
//    dds_mode_stored = *pMode;
//  }
//  return rec;
//}
//
//static u8 GetPwrCfg(u8 channel,TXR_PWR_CFG* pCfg)
//{
//  if( channel > MAX_CHANNELS_NUM )
//    return ERR_HW_CHANNEL_NUM;
//  memcpy( &pCfg, &dds_pwrCfg_stored, sizeof(TXR_PWR_CFG) );
//  return ERR_SUCCESS;
//}
//
//static u8 SavePwrCfg(u8 channel, TXR_PWR_CFG* pCfg ) {
//  uint8_t rec = ERR_SUCCESS;
//
//  if( channel > MAX_CHANNELS_NUM ){
//    rec = ERR_HW_CHANNEL_NUM;
//  }
//  dds_pwrCfg_stored = *pCfg;
//
//  return rec;
//}
//
//static u8 GetSamplingFrequency(u32* freq)
//{
//  *freq = sampling_freq_stored;
//  return ERR_SUCCESS;
//}

static u8 GetGainCfg(u8 channel, GAIN_CFG* pGain) {
  if( channel > MAX_CHANNELS_NUM )
    return ERR_HW_CHANNEL_NUM;
  memcpy( &pGain, &dds_gain_stored, sizeof(GAIN_CFG) );
  return ERR_SUCCESS;
}

static u8 SaveGainCfg(u8 channel, GAIN_CFG* pGain, uint16_t bn) {
  uint16_t sz;

  sz = (sizeof(DDS_CFG) > bn)? bn: sizeof(DDS_CFG);
  if( channel > MAX_CHANNELS_NUM )
    return ERR_HW_CHANNEL_NUM;
  memcpy( &dds_gain_stored, &pGain, sz );
  return ERR_SUCCESS;
}

err_t ServerSetup(void)
{
  struct tcp_pcb* pcb = tcp_new();
  if(pcb == NULL)
    return -1;
  if( tcp_bind(pcb,IP_ADDR_ANY,localNetCfg.net_hw_tcp_port) != ERR_OK)   //network config hardware
    return -1;
  tcp_arg(pcb,szArg);
  pcb = tcp_listen(pcb);
  pcb->so_options |= SOF_KEEPALIVE;
  tcp_accept(pcb,OnAccept);
  memset( &dds_cfg_stored, 0, sizeof(DDS_CFG) );
  return ERR_OK;
}

void localNetAddressInit(tNetCfg * pNetCfg) {
  struct ip_addr tmp_addr;
  uint8_t mac[6] = {0x3D,0x41,0x5A,0x1,0x11,0x1};

  IP4_ADDR(&tmp_addr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
//  IP4_ADDR(&tmp_addr, 192,168,10,99);
  pNetCfg->net_hw_ip = tmp_addr.addr;

  IP4_ADDR(&tmp_addr, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
//  IP4_ADDR(&tmp_addr, 192, 168, 10, 1);
  pNetCfg->net_hw_gw = tmp_addr.addr;

  IP4_ADDR(&tmp_addr, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
//  IP4_ADDR(&tmp_addr, 255, 255, 255, 0);
  pNetCfg->net_hw_mask = tmp_addr.addr;

  IP4_ADDR(&tmp_addr, DST_ADDR0, DST_ADDR1, DST_ADDR2, DST_ADDR3);
//  IP4_ADDR(&tmp_addr, 192, 168, 10, 1);
  pNetCfg->net_dst_ip = tmp_addr.addr;

  pNetCfg->net_hw_tcp_port = 7001;
  pNetCfg->net_dst_udp_port = 8000;

  memcpy(pNetCfg->net_hw_mac,&mac[0],sizeof(mac)*sizeof(mac[0]));
}

err_t serverSetup(void) {
  struct tcp_pcb* pcb = tcp_new();
  if(pcb == NULL)
    return -1;
  if( tcp_bind(pcb,IP_ADDR_ANY,localNetCfg.net_hw_tcp_port) != ERR_OK)   //network config hardware
    return -1;
  tcp_arg(pcb,szArg);
  pcb = tcp_listen(pcb);
  pcb->so_options |= SOF_KEEPALIVE;
  tcp_accept(pcb,OnAccept);
  memset(&dds_cfg_stored,0,sizeof(DDS_CFG));
  return ERR_OK;
}

static uint8_t buf[4100];
static u16 nBytesRcvd = 0;
static err_t OnReceive(void *arg, struct tcp_pcb *pcb, struct pbuf *p,err_t err) {
  (void)arg;
  struct pbuf *ptr;
  if(err == ERR_OK && p != NULL)
  {
    tcp_recved(pcb, p->tot_len);
    for(ptr = p;ptr != NULL;ptr=ptr->next)
    {
      if( (nBytesRcvd + ptr->len) > sizeof(buf) )
      {
        nBytesRcvd = 0;
      }
      else
      {
        memcpy(buf+nBytesRcvd,ptr->payload,ptr->len);
        nBytesRcvd += ptr->len;
      }
      if( (ptr->flags & PBUF_FLAG_PUSH) && nBytesRcvd  ) // tcp psh flag
      {
        Server(pcb, nBytesRcvd);
        nBytesRcvd = 0;
      }
    }
    pbuf_free(p);
  }
  else if(err == ERR_OK && p == NULL)
  {
    tcp_close(pcb);
  }
  else
  {
    pbuf_free(p);
  }
  return ERR_OK;
}

static err_t OnAccept(void *arg, struct tcp_pcb *pcb, err_t err){
  (void)arg;
  (void)err;

  tcp_recv(pcb,OnReceive);
  return ERR_OK;
}

EXT_SYNC_CFG ext_sync_cfg = {EXT_SYNC_ON,100};
u8 needToReset = 0;
static void Server( struct tcp_pcb *pcb, uint16_t byteNum ) {
  CFG_DATA* pcd = (CFG_DATA*)buf;
  uint16_t to_send = sizeof(pcd->head);
  switch(pcd->head.cmd){
    case CMD_EXT_SYNC_INPUT:
      // Управление режимом внешней синхронизации
      pcd->head.err = ERR_SUCCESS;
      if(pcd->head.dir == CMD_DIR_READ) {
        // Возвращаем наверх текущий режим синхронизации
        pcd->data.byte = (uint8_t)tims.mainMode;
        to_send += sizeof(u8);
      }
      else {
        if( pcd->data.byte > 1 ){
          pcd->head.err = ERR_HW_MODE;
        }
        else {
          pcd->head.err = ERR_SUCCESS;
          // tims.mainMode = 1 - внешняя синхронизация
          // tims.mainMode = 0 - внутренняя синхронизация
          tims.mainMode = pcd->data.byte;
        }
      }
      break;
    case CMD_CHANNEL_CFG:
      // Параметры излучения для заданного канала (в случае альтиметра канал единственный)
      if(pcd->head.dir == CMD_DIR_READ) {
        // Возвращаем наверх текущие параметры
        if( pcd->data.byte > MAX_CHANNELS_NUM ){
          pcd->head.err = ERR_HW_CHANNEL_NUM;
        }
        else{
          pcd->data.dds_cfg = dds_cfg_stored;
          to_send += sizeof(DDS_CFG);
        }
      }
      else
      {
        if( byteNum != sizeof(DDS_CFG) ){
          pcd->head.err = ERR_HW_MODE;
        }
        else if( pcd->data.dds_cfg.mode > 2){
          pcd->head.err = ERR_HW_MODE;
        }
        else if( (pcd->data.dds_cfg.period < 500) ||
                  (pcd->data.dds_cfg.period > 2000) ){
          // Период меньше минимального или больше максимального
          pcd->head.err = ERR_HW_PERIOD;
        }
        else {
          pcd->head.err = ERR_SUCCESS;
          dds_cfg_stored = pcd->data.dds_cfg;
          dds_mode_stored.mode = dds_cfg_stored.mode;
          tims.opMode = dds_mode_stored.mode;
          tims.T1Main = dds_cfg_stored.period;
          tims.fgen = dds_cfg_stored.dds.impulse.fm.freq_start;
        }
      }
      break;

    case CMD_VARU_CFG:
      break;

    case CMD_CHANNEL_MODE:
      if( pcd->head.dir == CMD_DIR_READ ){
        if( pcd->data.byte > MAX_CHANNELS_NUM ){
          pcd->head.err = ERR_HW_CHANNEL_NUM;
        }
        else {
          // Возвращаем наверх текущий режим работы канала
          pcd->data.dds_mode = dds_mode_stored;
          to_send += sizeof(DDS_MODE);
        }
      }
      else {
        if( pcd->data.dds_mode.channel > MAX_CHANNELS_NUM ){
          pcd->head.err = ERR_HW_CHANNEL_NUM;
        }
        else if( (byteNum != sizeof(DDS_MODE)) ||
                  (pcd->data.dds_mode.mode > 2) ){
          pcd->head.err = ERR_HW_MODE;
        }
        else {
          pcd->head.err = ERR_SUCCESS;
          dds_mode_stored = pcd->data.dds_mode;
          dds_cfg_stored.mode = dds_mode_stored.mode;
          tims.opMode = dds_mode_stored.mode;
        }
      }
      break;

    case CMD_PWR_LEVEL:
      if( pcd->head.dir == CMD_DIR_READ ) {
        if( pcd->data.byte > MAX_CHANNELS_NUM ){
          pcd->head.err = ERR_HW_CHANNEL_NUM;
        }
        else {
          // Возвращаем наверх текущее значение мощности излучения для заданного канала
          pcd->head.err = ERR_SUCCESS;
          pcd->data.pwr_cfg = dds_pwrCfg_stored;
          to_send += sizeof(TXR_PWR_CFG);
        }
      }
      else {

        if( pcd->data.pwr_cfg.channel > MAX_CHANNELS_NUM ){
          pcd->head.err = ERR_HW_CHANNEL_NUM;
        }
        else if( (byteNum != sizeof(DDS_MODE)) ||
                  (pcd->data.dds_mode.mode > 2) ){
          pcd->head.err = ERR_HW_MODE;
        }
        else {
          // Устанавливаем новое значение мощности излучения для канала
          dds_pwrCfg_stored = pcd->data.pwr_cfg;
          tims.kpd = dds_pwrCfg_stored.level;
          pcd->head.err = ERR_SUCCESS;
        }
      }
      break;

    case CMD_PRESSURE:
      break;

    case CMD_USART_ACTIVATE:
      break;

    case CMD_USART_DEACTIVATE:
      break;

    case CMD_USART_TX:
      break;

    case CMD_GET_SW_VER:
      // Возвращаем наверх версию ПО микроконтроллера
      pcd->data.word = s_wSoftVersion;
      pcd->head.err = ERR_SUCCESS;
      to_send += sizeof(u16);
      break;

    case CMD_STOP_ALL:
      // Остановка всех каналов
      pcd->head.err = ERR_SUCCESS;
      dds_mode_stored.mode = DDS_MODE_STOP;
      dds_cfg_stored.mode = DDS_MODE_STOP;
      tims.opMode = DDS_MODE_STOP;
      break;
    case  CMD_NET_CFG_SOCKET_HW:
      if( pcd->head.dir == CMD_DIR_READ ){
        //read net config
        pcd->head.err = ReadNetCfgFromFlash(&pcd->data.net_cfg);
        pcd->data.net_cfg = localNetCfg;
        if( pcd->head.err == ERR_SUCCESS )
        {
          to_send += sizeof(tNetCfg );
        }
      }
      else {
        if( (byteNum != sizeof( tNetCfg )) ){
          pcd->head.err = ERR_PARAM_NET_CFG;
        }
        else {

          u8 hw_ip1 = (ntohl(pcd->data.net_cfg.net_hw_ip)>>24)&0xff;
          u8 hw_ip4 = (ntohl(pcd->data.net_cfg.net_hw_ip)&0xff);
          u8 hw_gw1 = (ntohl(pcd->data.net_cfg.net_hw_gw)>>24)&0xff;
          u8 hw_gw4 = (ntohl(pcd->data.net_cfg.net_hw_gw)&0xff);
          u8 dst_ip1 = (ntohl(pcd->data.net_cfg.net_dst_ip)>>24)&0xff;
          u8 dst_ip4 = (ntohl(pcd->data.net_cfg.net_dst_ip)&0xff);
          u16 tcp_port = pcd->data.net_cfg.net_hw_tcp_port;
          u16 udp_port = pcd->data.net_cfg.net_dst_udp_port;

          u32 msk = ntohl(pcd->data.net_cfg.net_hw_mask);
  //        u8 result = msk&0xFFFF0000;
          if( (msk==0xFFFFFFFC) || (msk==0xFFFFFFF8) || (msk==0xFFFFFFF0) || (msk==0xFFFFFFE0) || (msk==0xFFFFFFC0) ||
              (msk==0xFFFFFF80) || (msk==0xFFFFFF00) || (msk==0xFFFFFE00) || (msk==0xFFFFFC00) || (msk==0xFFFFF800) ||
              (msk==0xFFFFF000) || (msk==0xFFFFE000) || (msk==0xFFFFC000) || (msk==0xFFFF8000) )
          {

            if( (hw_ip1!=0x00 && hw_ip1<0xFF) && (hw_ip4!=0x00 && hw_ip4<0xFF) &&
                (hw_gw1!=0x00 && hw_gw1<0xFF) && (hw_gw4!=0x00 && hw_gw4<0xFF) &&
                (dst_ip1!=0x00 && dst_ip1<0xFF) && (dst_ip4!=0x00 && dst_ip4<0xFF) &&
                (udp_port>=1024 && udp_port<=16383) && (tcp_port>=1024 && tcp_port<=16383))
            {
                //write net cfg
                pcd->head.err = WriteNetCfgToFlash(&pcd->data.net_cfg);
                needToReset = 1;
            }
            else {
              pcd->head.err = ERR_PARAM_NET_CFG;
            }
          }
          else
            pcd->head.err = ERR_PARAM_NET_CFG;
        }
      }

      break;
    case CMD_SET_FREQ_SAMPLING:
      if( pcd->head.dir == CMD_DIR_READ ) {
        // Возвращаем наверх установленное значение частоты дискретизации
        pcd->data.freq_sampling = sampling_freq_stored;
        pcd->head.err = ERR_SUCCESS;
        to_send += sizeof(u32);
      }
      else {
        // Уставливаем новое значение частоты дискретизации
        if( (pcd->data.freq_sampling > (FADC*1.5)) ||
            (pcd->data.freq_sampling < (FADC/1.5))){
          pcd->head.err = ERR_PARAM_FREQ;
          tims.fAdc = FADC;
        }
        else {
          pcd->head.err = ERR_SUCCESS;
          tims.fAdc = pcd->data.freq_sampling;
        }
        sampling_freq_stored = tims.fAdc;
      }
      break;
    case CMD_GAIN_CFG:
      if(pcd->head.dir == CMD_DIR_READ )
      {
        // Возвращаем наверх текущее значение усиления в приеме для заданного канала
        pcd->head.err = GetGainCfg( pcd->data.byte,  &pcd->data.gain_cfg );
        if( pcd->head.err == ERR_SUCCESS )
          to_send += sizeof(GAIN_CFG);
      }
      else
      {
        // Устанавливаем новое значение усиления для канала
        u8 status = ERR_SUCCESS;

        // Значение которым необходимо заполнить таблицу вару
        u16 varuTableValue = GetVaruValueFromGainValue(pcd->data.gain_cfg.level);
        // Записываем неизменное значение ВАРУ
        for(uint16_t i = 0; i < DAC_SAMPLE_NUM; i++ ){
          dacData[i] = varuTableValue;
        }

        // @insert Добавляем необходимые действия по формированию и установке новой таблицы ВАРУ
        //         переменную status устанавливаем кодом ошибки при обнаружении такой

        pcd->head.err = status;
        if(pcd->head.err == ERR_SUCCESS)
          SaveGainCfg(pcd->data.gain_cfg.channel, &pcd->data.gain_cfg, byteNum );
      }
      break;
    case CMD_PERFORM_SINGLE_SHOT:
      break;
    default:
      pcd->head.err = ERR_CMD_UNKNOWN;
      break;
  }
  tcp_write(pcb,(const void*)pcd,to_send,0);
  // Пытаемся применить новые настройки
  if( (TIM3->CR1 & TIM_CR1_CEN) == 0 ){
    // Главный таймер остановлен - можно применять новае настройки
    firstSwProcess();
  }
}

u16 GetVaruValueFromGainValue(u16 gain_value) {
  u16 varu_value;
  if( (gain_value > 0) && (gain_value < 14) ) {
      varu_value = 307 * gain_value + (gain_value - 1);
  }
  else if( gain_value == 14 ){
      varu_value = 4095;
  }
  else {
      varu_value = 1;
  }

  return varu_value;
}

u16 GetGainValueFromVaruValue(u16 varu_value){
  u16 gain_value;

  if(varu_value == 4095) {
    gain_value = 14;
  }
  else if( (varu_value <= 1) || (varu_value > 4095)){
    gain_value = 0;
  }
  else {
    gain_value = varu_value / 307;
  }
  return gain_value;
}
