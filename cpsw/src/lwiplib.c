/**
*  \file lwiplib.c
*
*  \brief lwip related initializations
*/
/*
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
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
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
*/

/*
** Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
** ALL RIGHTS RESERVED
*/
#include "lwiplib.h"
#include "beaglebone.h"
#include "cpsw.h"
#include "netif/cpswif.h"
#include "delay.h"
#include "lwip/netif.h"
#include "lwip/prot/dhcp.h"
#include "lwip/dhcp.h"
#include "lwip/sys.h"
#include "lwip/autoip.h"
#include <sched.h>

/* when the link is down, show a message, wait for a while and reboot */
#define REBOOT_DELAY_SECONDS 5 

#define DEFAULT_INST_NUM 0
#define DEFAULT_PORT_NUMBER 1

#define LWIP_NOT_INITIALIZED 0
#define LWIP_INITIALIZED 1
//#define NUM_DHCP_TRIES 20

#define SECONDS_TO_MILISECONDS 1000

/******************************************************************************
**                       INTERNAL FUNCTION PROTOTYPES
******************************************************************************/
static bool lwIPDHCPComplete(unsigned int ifNum);
static void CPSWCore0RxIsr(void*);
static void CPSWCore0TxIsr(void*);
static void interruptSetup(void);


/******************************************************************************
**                       INTERNAL VARIABLE DEFINITIONS
******************************************************************************/
/*
** The lwIP network interface structure for CPSW ports.
*/
#ifdef CPSW_DUAL_MAC_MODE
static struct netif cpswNetIF[MAX_CPSW_INST * MAX_SLAVEPORT_PER_INST];
#else
static struct netif cpswNetIF[MAX_CPSW_INST];
#endif

/*
** Helper to identify ports
*/
static struct cpswportif cpswPortIf[MAX_CPSW_INST * MAX_SLAVEPORT_PER_INST];

/******************************************************************************
**                          FUNCTION DEFINITIONS
******************************************************************************/
/**
 * @brief checks if the dhcp has changed its state for 
 * (LWIP_DHCP_TIMEOUT * 10) ms
 *  
 * @param state A pointer to the volatile state to check.
 */
static inline void dhcpCheck(volatile unsigned char * const state)
{
    unsigned int cnt = LWIP_DHCP_TIMEOUT;

    /* Check for DHCP completion for 'cnt' number of times, each 10ms */
    while( cnt-- && *state != DHCP_STATE_BOUND)
    {
	    if(cnt % 2 == 0)
               sched_yield();
        delay(10);
    }
}

/**
 * @brief   This function waits for DHCP completion with a timeout
 *
 * @param   ifNum  The netif number for the interface
 *
 * @return  True if IP acquired succesfully, false otherwise.
*/
static bool lwIPDHCPComplete(const unsigned int ifNum)
{
    bool ret = true;
    unsigned int dhcpTries = 0;
    volatile unsigned char * state;
    struct netif* const netif = &cpswNetIF[ifNum];

    do
    {
        ++dhcpTries;
        dhcp_start(netif);
        state = (volatile unsigned char *) &(netif->state);
        LWIP_PRINTF("\n\rDHCP Trial %d ", (dhcpTries));
        dhcpCheck(state);
    }while((dhcpTries < NUM_DHCP_TRIES) && (*state != DHCP_STATE_BOUND));

    if (*state != DHCP_STATE_BOUND)
    {
        LWIP_PRINTF("\n\rUnable to complete DHCP! \n\r");
        ret = false;
    }
    return ret;
}

/**
 * @brief Starts the interface in Lwip according to its mode.
 * 
 * @param lwipIf The addresses and modes to set.
 * @param ifNum The interface to set.
 * 
 * @return True if mode ok, false otherwise.
 */
static bool netifStart(LWIP_IF* const lwipIf, const unsigned int ifNum)
{
    bool ret = true;
    switch(lwipIf->ipMode)
    {
        case IPADDR_USE_DHCP:
            ret = lwIPDHCPComplete(ifNum);
            break;
        case IPADDR_USE_AUTOIP:
            autoip_start(&cpswNetIF[ifNum]);
            /*This has no break on purpose, since both autoip and static must do netif_set_up*/
        case IPADDR_USE_STATIC:
            netif_set_up(&cpswNetIF[ifNum]);
            break;
        default:
            ret = false;
    }
    return ret;
}

/**
 * @brief Resets Addresses if the mode is not static.
 * 
 * @param lwipIf the addresses to configure.
 */
static inline void startLwipIf(LWIP_IF* const  lwipIf)
{
    if(lwipIf->ipMode != IPADDR_USE_STATIC)
    {
        lwipIf->ipAddr.addr = 0;
        lwipIf->netMask.addr = 0;
        lwipIf->gwAddr.addr = 0;
    }
}

/**
 * @brief Configures cpswPortIf for de desired interface, setting its
 * instance number, port number and mac address.
 * 
 * @param lwipIf The lwip interface that has the mac address
 * @param ifNum The interface to configure
 */
static inline void setPortIf(LWIP_IF* const lwipIf, const unsigned int ifNum)
{
    unsigned int macIndex;
    cpswPortIf[ifNum].inst_num = ifNum;
    cpswPortIf[ifNum].port_num = DEFAULT_PORT_NUMBER;

    /* set MAC hardware address */
    for(macIndex = 0; macIndex < LEN_MAC_ADDRESS; ++macIndex)
    {
        cpswPortIf[ifNum].eth_addr[macIndex] =
                         lwipIf->macArray[(LEN_MAC_ADDRESS - 1) - macIndex];
    }
}

/**
 *
 * @brief Initializes the lwIP TCP/IP stack.
 *
 * @param lwipIf  The interface structure for lwIP
 *
 * @return true if everything ok, false otherwise.
*/
static bool lwIPInit(LWIP_IF *const lwipIf, const unsigned int ifNum)
{
    static unsigned int lwipInitFlag = LWIP_NOT_INITIALIZED;
    bool ret = true;

    /* do lwip library init only once */
    if(LWIP_NOT_INITIALIZED == lwipInitFlag)
    {
        tcpip_init(NULL, NULL);
    }
    setPortIf(lwipIf, ifNum);

    if(netif_add(&cpswNetIF[ifNum], &lwipIf->ipAddr, &lwipIf->netMask, 
            &lwipIf->gwAddr,&cpswPortIf[ifNum], cpswif_init, tcpip_input) != NULL)
    {
        if(LWIP_NOT_INITIALIZED == lwipInitFlag)
        {
            netif_set_default(&cpswNetIF[ifNum]);
            lwipInitFlag = LWIP_INITIALIZED;
        }
	//lwipIf->ipMode = IPADDR_USE_DHCP;
        ret = netifStart(lwipIf, ifNum);
    }
    else
    {
        LWIP_PRINTF("\n\rUnable to add interface for interface %d", ifNum);
        ret = false;
    }
    return ret;
}

/**
 * @brief   Interrupt handler for Receive Interrupt. Directly calls the
 *          cpsw interface receive interrupt handler.
 *
 * @param   instNum  The instance number of CPSW module for which receive
 *                   interrupt happened
*/
static inline void lwIPRxIntHandler(const unsigned int instNum)
{
    cpswif_rx_inthandler(instNum);
}

/**
 * @brief   Interrupt handler for Transmit Interrupt. Directly calls the
 *          cpsw interface transmit interrupt handler.
 *
 * @param   instNum  The instance number of CPSW module for which transmit
 *                   interrupt happened
*/
static inline void lwIPTxIntHandler(const unsigned int instNum)
{
    cpswif_tx_inthandler(instNum);
}

/**
 * @brief Initializes low level device.
 *
 */
static inline void cpswInit(void)
{
    CPSWPinMuxSetup();
    CPSWClkEnable();
}

/**
 * @brief Initializes Phy and returns MAC Address.
 * 
 * @param lwipIfPort The addresses, to return the MAC address.
 */
static inline void phyInit(LWIP_IF* const lwipIfPort)
{
    /* Chip configuration RGMII selection */
    EVMPortMIIModeSelect();

    /* Get the MAC address */
    EVMMACAddrGet(0, lwipIfPort->macArray);
}

/**
 * @brief Prints a message wait a delay time and reboot the board(calling exit).
 */
static inline void ipFailed(void)
{
    printk("\n\r\n\rIP Address Acquisition Failed. Please Verify Your Cable/Link Status...");
    printk("\n\r\n\rSystem Will Automatically Reboot In %d Seconds.\r\n", REBOOT_DELAY_SECONDS);
    delay(REBOOT_DELAY_SECONDS * SECONDS_TO_MILISECONDS);
    exit(1);
}

/**
 * @brief Interrupt Handler for Core Receive interrupt
 * 
 * @param instNum The device Instance number
 */
static void CPSWCore0RxIsr(void* instNum)
{
    lwIPRxIntHandler((unsigned int)instNum);
}

/**
 * @brief Interrupt Handler for Core Transmit interrupt
 * 
 * @param instNum The device Instance number
 */
static void CPSWCore0TxIsr(void* instNum)
{
    lwIPTxIntHandler((unsigned int)instNum);
}

/**
 * @brief Displays the IP address on the Console
 * 
 * @param ipAddr The desired IP address to display.
 */
static inline void IpAddrDisplay(const ip4_addr_t* const ipAddr)
{
    printk("\n\r\n\rIP Address Assigned: %s\r\n", ip4addr_ntoa(ipAddr));
}

/**
 * @brief Sets up the ARM Interrupt Controller for generating timer interrupt.
 */
static void interruptSetup(void)
{
rtems_status_code sc = RTEMS_SUCCESSFUL;

  sc = rtems_interrupt_handler_install(
    AM335X_INT_3PGSWRXINT0,
    "EthRX",
    RTEMS_INTERRUPT_UNIQUE,
    CPSWCore0RxIsr,
    (void*)DEFAULT_INST_NUM
  );
  LWIP_ASSERT("Install interrupt handler rx", sc == RTEMS_SUCCESSFUL);

  sc = rtems_interrupt_handler_install(
    AM335X_INT_3PGSWTXINT0,
    "EthTX",
    RTEMS_INTERRUPT_UNIQUE,
    CPSWCore0TxIsr,
    (void*)DEFAULT_INST_NUM
  );
    LWIP_ASSERT("Install interrupt handler tx", sc == RTEMS_SUCCESSFUL);
}

bool startLwip(LWIP_IF* const lwipIfPort)
{
    const unsigned int ifNum = DEFAULT_INST_NUM;
    bool ret;

    /*Initialization of low level device*/
    cpswInit();
    /*Initialization of the PHY and getting of MAC Address.*/
    phyInit(lwipIfPort);
    /*Set ISR for the device*/
    interruptSetup();
    printk("Acquiring IP Address... \n\r" );
    /*Set up lwipIfPort properly for the desired mode*/
    startLwipIf(lwipIfPort);
    /*Start Lwip stack with the desired address*/
    ret = lwIPInit(lwipIfPort, ifNum);

    if(ret)
    {       
        IpAddrDisplay((ip4_addr_t *)&cpswNetIF[ifNum].ip_addr);
    }
    else /*Failed lwipinit, Print a message and reset the board*/
    {
        ipFailed();
    }
    return ret;
}
