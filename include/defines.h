/**
 *  Defines for your entire project at one place
 * 
 *	@author 	Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@version 	v1.0
 *	@ide		Keil uVision 5
 *	@license	GNU GPL v3
 *	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#ifndef DEFINES_H
#define DEFINES_H

/* Put your global defines for all libraries here used in your project */
#define PLL_M		15
#define PLL_N		216
#define PLL_P		2
#define PLL_Q		7

/* MAC ADDRESS: MAC_ADDR0:MAC_ADDR1:MAC_ADDR2:MAC_ADDR3:MAC_ADDR4:MAC_ADDR5 */
/* In case you want to use custom MAC, use parameter in init function */
/**
 * @brief  Default MAC address for your device
 * @note   This MAC is used in case you set mac_addr param in @ref TM_ETHERNET_Init() function to "NULL"
 */
#ifndef MAC_ADDR0
#define MAC_ADDR0             0x06
#define MAC_ADDR1             0x05
#define MAC_ADDR2             0x04
#define MAC_ADDR3             0x03
#define MAC_ADDR4             0x02
#define MAC_ADDR5             0x01
#endif

/* Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
/* Used in case DHCP is not used or response failed */
/* In case you want to use custom IP, use parameter in init function */
/**
 * @brief  Default IP address
 * @note   Used in case DHCP is not used or response failed
 * @note   In case you want to use custom IP, use parameter in @ref TM_ETHERNET_Init() function
 */
#ifndef IP_ADDR0
#define IP_ADDR0              192
#define IP_ADDR1              168
#define IP_ADDR2              1
#define IP_ADDR3              15
#endif

/**
 * @brief  Netmask address
 * @note   In case you want to use custom netmask, use parameter in @ref TM_ETHERNET_Init() function
 */
#ifndef NETMASK_ADDR0
#define NETMASK_ADDR0           255
#define NETMASK_ADDR1           255
#define NETMASK_ADDR2           255
#define NETMASK_ADDR3           0
#endif

/**
 * @brief  Gateway address
 * @note   In case you want to use custom gateway, use parameter in @ref TM_ETHERNET_Init() function
 */
#ifndef GW_ADDR0
#define GW_ADDR0              192
#define GW_ADDR1              168
#define GW_ADDR2              1
#define GW_ADDR3              1
#endif

#ifndef DST_ADDR0
#define DST_ADDR0              192
#define DST_ADDR1              168
#define DST_ADDR2              1
#define DST_ADDR3              1
#endif

#ifndef HW_TCP_PORT
#define HW_TCP_PORT            7001
#endif

#ifndef UDP_PORT
#define UDP_POST               8000
#endif


#endif
