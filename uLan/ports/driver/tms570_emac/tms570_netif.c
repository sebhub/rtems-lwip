/*
 * Copyright (c) 2013, 2015 Czech Technical University in Prague
 * Czech Republic
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
 * Author: Premysl Houdek <houdepre@fel.cvut.cz>
 * Mentor: Pavel Pisa <pisa@cmp.felk.cvut.cz>
 * Industrial Informatics Group, FEE, Czech Technical University in Prague
 *
 * Based on work of Carlos Jenkins, Rostislav Lisovy, Jan Dolezal
 */

/* lwIP headers */
#include "lwip/init.h"
#if LWIP_VERSION_MAJOR >= 2
#include "lwip/timeouts.h"
#else /*LWIP_VERSION_MAJOR*/
#include "lwip/timers.h" /* for DHCP binding in NO_SYS mode */
#endif /*LWIP_VERSION_MAJOR*/
#include "lwip/sys.h" /* includes - lwip/opt.h, lwip/err.h, arch/sys_arch.h */
#include "lwip/tcpip.h" /* includes - lwip/opt.h, lwip/api_msg.h, lwip/netifapi.h, lwip/pbuf.h, lwip/api.h, lwip/sys.h, lwip/timers.h, lwip/netif.h */
#include "lwip/stats.h" /* includes - lwip/mem.h, lwip/memp.h, lwip/opt.h */
#include "lwip/snmp.h"
#include "netif/etharp.h" /* includes - lwip/ip.h, lwip/netif.h, lwip/ip_addr.h, lwip/pbuf.h */
#include <lwip/netifapi.h>
/* end - lwIP headers */

//--------moje
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <bsp/irq.h>
#include <bsp/tms570.h>
#include <bsp/tms570-pinmux.h>
#include "arch/cc.h"
#include "eth_lwip.h"
#include "tms570_netif.h"
#include "ti_drv_emac.h"
#include "phy_dp83848h.h"
#include "tms570_emac.h"

/* take in account oversized frames */
#define MAX_TRANSFER_UNIT           1500

RTEMS_STATIC_ASSERT((EMAC_RX_BD_COUNT & (EMAC_RX_BD_COUNT - 1)) == 0, EMAC_RX_BD_COUNT_POWER_OF_TWO);
RTEMS_STATIC_ASSERT((EMAC_TX_BD_COUNT & (EMAC_TX_BD_COUNT - 1)) == 0, EMAC_TX_BD_COUNT_POWER_OF_TWO);
RTEMS_STATIC_ASSERT(sizeof(struct cppi_ram) <= 8192, EMAC_CPPI_RAM_SIZE);
RTEMS_STATIC_ASSERT(PBUF_POOL_BUFSIZE >= MAX_TRANSFER_UNIT + PBUF_LINK_HLEN, PBUF_POOL_BUFSIZE);

#define LINK_SPEED_OF_YOUR_NETIF_IN_BPS 10000000

/* Number of EMAC Instances */

#define DEFAULT_PHY_ADDR            0x1
#define FIND_FIRST_PHY_ALIVE        1 /* or use default (phy_address: 1) */
#define NUM_OF_PHYs             32

/* Channel number used for for RX, TX, unicast, broadcast or damaged frames;
 * there are different channels for rx and tx operations (i.e. RXCH0 != TXCH0) */
#define CHANNEL                 0

#ifndef TMS570_BALL_K19_MII_RXCLK
  #define TMS570_BALL_K19_MII_RXCLK TMS570_BALL_K19_MII_RX_CLK
#endif

/* Define those to better describe the network interface. */
#define IFNAME0                 'e'
#define IFNAME1                 'n'

/* Time to wait for autonegotiation in ticks. */
#define TICKS_PHY_AUTONEG           4000

/* startup init indicator */
static bool initialized = false;

/*private?*/
static void tms570_eth_hw_set_RX_HDP(struct tms570_netif_state *nf_state, volatile struct emac_rx_bd *new_head);
static void tms570_eth_hw_set_TX_HDP(struct tms570_netif_state *nf_state, volatile struct emac_tx_bd *new_head);
static void tms570_eth_hw_set_hwaddr(struct tms570_netif_state *nf_state, uint8_t *mac_addr);
static SYS_IRQ_HANDLER_FNC(tms570_eth_process_irq_rx);
static SYS_IRQ_HANDLER_FNC(tms570_eth_process_irq_tx);
struct netif *tms570_eth_get_netif(uint32_t instance_number);
static err_t tms570_eth_send(struct netif *netif, struct pbuf *p);
static err_t tms570_eth_send_raw(struct netif *netif, struct pbuf *pbuf);
static err_t tms570_eth_init_hw(struct tms570_netif_state *nf_state);
static err_t tms570_eth_init_hw_post_init(struct tms570_netif_state *nf_state);
static err_t tms570_eth_init_interrupt(struct netif *netif);
#ifdef EMAC_ENABLE_MDIO_SUPPORT
static err_t tms570_eth_init_find_PHY(struct tms570_netif_state *nf_state);
#endif
static void tms570_eth_init_netif_fill(struct netif *netif);
static void tms570_eth_init_buffer_descriptors(struct tms570_netif_state *nf_state);
static void tms570_eth_init_set_pinmux();

static inline uint32_t tms570_eth_swap(uint32_t word)
{
#if TMS570_VARIANT == 4357
  return __builtin_bswap32(word);
#else
  return word;
#endif
}

static inline uint8_t *tms570_eth_swap_bufptr(volatile void *p)
{
  return (uint8_t *)tms570_eth_swap((uint32_t)p);
}

static inline struct emac_tx_bd *tms570_eth_swap_txp(volatile struct emac_tx_bd *p)
{
  return (struct emac_tx_bd *)tms570_eth_swap((uint32_t)p);
}

static inline struct emac_rx_bd *tms570_eth_swap_rxp(volatile struct emac_rx_bd *p)
{
  return (struct emac_rx_bd *)tms570_eth_swap((uint32_t)p);
}

/***** initializing functions **********************************************/


struct tms570_netif_state *
tms570_eth_init_state(void)
{
  struct tms570_netif_state *nf_state = (struct tms570_netif_state *)malloc(sizeof(struct tms570_netif_state));

  nf_state->emac_base = &TMS570_EMACM;
  nf_state->emac_ctrl_base = &TMS570_EMACC;
  nf_state->emac_ctrl_ram = (struct cppi_ram *) EMAC_CTRL_RAM_BASE;
#ifdef EMAC_ENABLE_MDIO_SUPPORT
  nf_state->mdio.base.init = TIMDIOInit;
  nf_state->mdio.base.phyAliveStatusGet = TIMDIOPhyAliveStatusGet;
  nf_state->mdio.base.phyLinkStatusGet = TIMDIOPhyLinkStatusGet;
  nf_state->mdio.base.phyRegRead = TIMDIOPhyRegRead;
  nf_state->mdio.base.phyRegWrite = TIMDIOPhyRegWrite;
  nf_state->mdio.baseAddr = (uintptr_t) &TMS570_MDIO;
  nf_state->phy_addr = DEFAULT_PHY_ADDR;
#if !NO_SYS
  nf_state->waitTicksForPHYAneg = TICKS_PHY_AUTONEG;
#endif
#endif
  return nf_state;
}

static void
tms570_eth_init_netif_fill(struct netif *netif)
{
#if LWIP_NETIF_HOSTNAME
  netif->hostname = "tms570";
#endif

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  NETIF_INIT_SNMP(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  /* We directly use etharp_output() here to save a function call.
   * You can instead declare yo_SKIP_TO_HWur own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...)
   */
  netif->output = etharp_output;
  netif->linkoutput = tms570_eth_send;

  /* maximum transfer unit */
  netif->mtu = MAX_TRANSFER_UNIT;

  /* device capabilities */
  /* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
  netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_LINK_UP;
}

err_t
tms570_eth_init_netif(struct netif *netif)
{
  err_t retVal;
  struct tms570_netif_state *nf_state = (struct tms570_netif_state *)netif->state;

  if (initialized)
    return ERR_IF;

  if (nf_state == NULL) {
    /* nf_state needs to be freed */
    if ((nf_state = tms570_eth_init_state()) == 0) {
      return ERR_IF;
    }
    netif->state = nf_state;
  }

  tms570_eth_init_netif_fill(netif);

  if ((retVal = tms570_eth_init_hw(nf_state)) != ERR_OK) {
    tms570_eth_debug_printf("tms570_eth_init_hw: %d", retVal);
    return retVal;
  }
  tms570_eth_init_buffer_descriptors(nf_state);
  tms570_eth_hw_set_hwaddr(nf_state, netif->hwaddr);
  tms570_eth_init_interrupt(netif);
  if ((retVal = tms570_eth_init_hw_post_init(nf_state)) != ERR_OK) {
    tms570_eth_debug_printf("tms570_eth_init_hw_post_init: %d", retVal);
    return retVal;
  }
  initialized = true;
#if TMS570_NETIF_DEBUG
  tms570_eth_debug_print_HDP(nf_state);
#endif
  return ERR_OK;
}

static err_t
tms570_eth_init_interrupt(struct netif *netif)
{
  int res;

  res = sys_request_irq(TMS570_IRQ_EMAC_TX, tms570_eth_process_irq_tx,
                        0, "emac_tx", netif);
  if (res < 0) {
    sys_arch_printk("Failed to install tx handler\n");
    return ERR_IF;
  }

  res = sys_request_irq(TMS570_IRQ_EMAC_RX, tms570_eth_process_irq_rx,
                        0, "emac_rx", netif);
  if (res < 0) {
    sys_arch_printk("Failed to install rx handler\n");
    return ERR_IF;
  }
  return ERR_OK;
}

#ifdef EMAC_ENABLE_MDIO_SUPPORT
static err_t
tms570_eth_init_find_PHY(struct tms570_netif_state *nf_state)
{
  uint8_t index;
  uint16_t regContent;
  uint32_t physAlive;

  MDIOPhyRegRead(&nf_state->mdio.base, nf_state->phy_addr, PHY_BMSR, &regContent);
  physAlive = MDIOPhyAliveStatusGet(&nf_state->mdio.base);
  /* Find first alive PHY -- or use default if alive */
  if (!(physAlive & (1 << nf_state->phy_addr))) {
    for (index = 0; index < NUM_OF_PHYs; index++) {
      if (physAlive & (1 << index)) {
        nf_state->phy_addr = index;
        break;
      } else {
        /*
         * Try to 'wake up' PHY on 'index' address by
         * reading random register, making MDIO set
         * alive bit for current PHY
         */
        MDIOPhyRegRead(&nf_state->mdio.base, index,
                       PHY_BMCR, &regContent);

        /* Get updated register */
        physAlive = MDIOPhyAliveStatusGet(&nf_state->mdio.base);
        if (physAlive & (1 << index)) {
          nf_state->phy_addr = index;
          break;
        }
      }
    }

    if (!physAlive) {             /* FIXME je to ok? */
      tms570_eth_debug_printf("no phy found, phys: %d\n", physAlive);
      return NO_PHY_ALIVE;
    }
  }
  return ERR_OK;
}
#endif

static const uint32_t tms570_eth_pin_config[] = {
  TMS570_MMR_SELECT_MII_MODE,
  TMS570_BALL_V5_MDCLK,
  TMS570_BALL_G3_MDIO,
  TMS570_BALL_H19_MII_TXEN,
  TMS570_BALL_E18_MII_TXD_3,
  TMS570_BALL_R2_MII_TXD_2,
  TMS570_BALL_J19_MII_TXD_1,
  TMS570_BALL_J18_MII_TXD_0,
  TMS570_BALL_D19_MII_TX_CLK,
  TMS570_BALL_H18_MII_RXD_3,
  TMS570_BALL_G19_MII_RXD_2,
  TMS570_BALL_A14_MII_RXD_1,
  TMS570_BALL_P1_MII_RXD_0,
  TMS570_BALL_K19_MII_RXCLK,
  TMS570_BALL_N19_MII_RX_ER,
  TMS570_BALL_B11_MII_RX_DV,
  TMS570_BALL_B4_MII_CRS,
  TMS570_BALL_F3_MII_COL
};

static void
tms570_eth_init_set_pinmux(void)
{
  tms570_pin_config_prepare();
  tms570_pin_config_array_apply(tms570_eth_pin_config,
                                RTEMS_ARRAY_SIZE(tms570_eth_pin_config));
  tms570_pin_config_complete();
}

static err_t
tms570_eth_init_hw(struct tms570_netif_state *nf_state)
{
  tms570_eth_init_set_pinmux();

  /* Initialize EMAC control module and EMAC module */
  EMACInit(nf_state->emac_ctrl_base, nf_state->emac_base);

#ifdef EMAC_ENABLE_MDIO_SUPPORT
  /* Initialize MDIO module (reset) */
  MDIOInit(&nf_state->mdio.base, 0x0, 0x0);

  err_t retVal = tms570_eth_init_find_PHY(nf_state);
  if (retVal != ERR_OK) {
    tms570_eth_debug_printf("tms570_eth_init_find_PHY: %d", retVal);
    return retVal;
  }
  /*
   * Start autonegotiation and check on completion later or
   * when complete link register will be updated
   */
  PHY_auto_negotiate(&nf_state->mdio.base, nf_state->phy_addr,
                     PHY_100BASETXDUPL_m | PHY_100BASETX_m |
                     PHY_10BASETDUPL_m | PHY_10BASET_m);
  tms570_eth_debug_printf("autoneg started -- check on cable if it's connected!\r\n");
#endif

  /*
   * TODO: you can implement init of receive flow control somewhere
   * here if desired - set RXBUFFERFLOWEN in MACCONTROL
   */

  /* Acknowledge EMAC control module RX, TX and MISC interrupts */
  EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_RX);
  EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_TX);
  EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_MISC);

  /* Sets which channel will receive broadcasts */
  EMACRxBroadCastEnable(nf_state->emac_base, CHANNEL);

  /*
   * Sets channel where all frames will be copied to --
   * either with MAC address different from local device address,
   * either packets with error will be copied; appropriate error
   * will be set in the frame EOP buffer descriptor
   */
  EMACRxPromiscEnable(nf_state->emac_base, CHANNEL);

  /* Enable unicast */
  EMACRxUnicastSet(nf_state->emac_base, CHANNEL);

  /* Enable TX and RX interrupts in both EMAC module and EMAC control module */
  EMACTxIntPulseEnable(nf_state->emac_base, nf_state->emac_ctrl_base, 0, CHANNEL);
  EMACRxIntPulseEnable(nf_state->emac_base, nf_state->emac_ctrl_base, 0, CHANNEL);

  return ERR_OK;
}
static void
tms570_eth_hw_set_hwaddr(struct tms570_netif_state *nf_state, uint8_t *mac_addr)
{
  uint8_t mac[ETHARP_HWADDR_LEN];
  int i;

  for (i = 0; i < ETHARP_HWADDR_LEN; i++) {
    mac[i] = mac_addr[ETHARP_HWADDR_LEN - i - 1];
  }
  /* for flow control frames */
  EMACMACSrcAddrSet(nf_state->emac_base, mac);

  /*  Be sure to program all eight MAC address registers -
   *  whether the receive channel is to be enabled or not.
   */
  for (i = 0; i < 8; i++) {
    EMACMACAddrSet(nf_state->emac_base, i, mac, 0);
  }
}

static err_t
tms570_eth_init_hw_post_init(struct tms570_netif_state *nf_state)
{
#ifdef EMAC_ENABLE_MDIO_SUPPORT
  /* 0x3FFFFFFF is for 80MHz aproximately 13s */
  uint16_t regContent;

  /* wait for autonegotiation to be done or continue, when delay was reached */
  uint32_t timeToWake = nf_state->waitTicksForPHYAneg + sys_jiffies();

  while (PHY_is_done_auto_negotiate(&nf_state->mdio.base, nf_state->phy_addr) == false &&
         timeToWake > sys_jiffies())
    sys_arch_delay(20);
  /* XXX: if init is not done at the startup,
   * but couple days later, this might cause troubles */

  if (PHY_is_done_auto_negotiate(&nf_state->mdio.base, nf_state->phy_addr) != false)
    tms570_eth_debug_printf("autoneg finished\n");
  else
    tms570_eth_debug_printf("autoneg timeout\n");

  /* provide informations retrieved from autoneg to EMAC module */
  PHY_partner_ability_get(&nf_state->mdio.base, nf_state->phy_addr, &regContent);
  if (regContent & (PHY_100BASETXDUPL_m | PHY_10BASETDUPL_m)) {
    EMACDuplexSet(nf_state->emac_base, 1);
    /* this is right place to implement transmit flow control if desired --
     * set TXFLOWEN in MACCONTROL
     */
  } else if (regContent & (PHY_100BASETX_m | PHY_10BASET_m)) {
    EMACDuplexSet(nf_state->emac_base, 0);
  } else {
    tms570_eth_debug_printf("Unknown duplex mode\r\n");
    return UNKN_DUPLEX_MODE;
  }

  if (regContent & (PHY_100BASETX_m | PHY_100BASETXDUPL_m)) {
    EMACUse100Mbps(nf_state->emac_base, true);
  } else {
    EMACUse100Mbps(nf_state->emac_base, false);
  }
#else
  EMACDuplexSet(nf_state->emac_base, 1);
  EMACUse100Mbps(nf_state->emac_base, true);
#endif

  /* enable hostpend interrupts in emac module */
  nf_state->emac_base->MACINTMASKSET |= TMS570_EMACM_MACINTMASKSET_HOSTMASK;

  /* enable hostpend interrupts in emac control module */
  nf_state->emac_ctrl_base->C0MISCEN |= TMS570_EMACC_C0MISCEN_HOSTPENDEN;

  EMACMIIEnable(nf_state->emac_base);
  EMACTxEnable(nf_state->emac_base);
  EMACRxEnable(nf_state->emac_base);

  return ERR_OK;
}
static void
tms570_eth_init_buffer_descriptors(struct tms570_netif_state *nf_state)
{
  memset(nf_state->emac_ctrl_ram, 0x0, sizeof(*nf_state->emac_ctrl_ram));

  /*
   * Initialize the Descriptor Memory For TX and RX
   * Only Channel 0 is supported for both TX and RX
   */

  /* Initialize all the TX buffer Descriptors */
  struct txch *txch = &(nf_state->txch);
  txch->head = 0;
  txch->tail = 0;
  txch->bds = &nf_state->emac_ctrl_ram->tx_bds[0];

  /* Initialize the descriptors for the RX channel */
  struct rxch *rxch = &(nf_state->rxch);
  rxch->head = 0;
  rxch->bds = &nf_state->emac_ctrl_ram->rx_bds[0];

  for (size_t i = 0; i < EMAC_RX_BD_COUNT; ++i) {
    struct pbuf *p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
    LWIP_ASSERT("no rx pbuf for init", p != NULL);
    LWIP_ASSERT("rx pbuf has multiple buffers", p->next == NULL);

    rtems_cache_invalidate_multiple_data_lines(p->payload, p->len);

    volatile struct emac_rx_bd *bd = &rxch->bds[i];
    bd->next = tms570_eth_swap_rxp(&rxch->bds[(i + 1) % EMAC_RX_BD_COUNT]);
    bd->bufptr = tms570_eth_swap_bufptr(p->payload);
    bd->bufoff_len = tms570_eth_swap(p->len);
    bd->flags_pktlen = tms570_eth_swap(EMAC_DSC_FLAG_OWNER);
    bd->pbuf = p;
  }

  sys_arch_data_sync_barier();
  tms570_eth_hw_set_RX_HDP(nf_state, &rxch->bds[0]);

#if TMS570_NETIF_DEBUG
  tms570_eth_debug_show_rx(nf_state);
  tms570_eth_debug_show_tx(nf_state);
#endif

}

/* send and receive functions / ISRs ******************************************/

static err_t
tms570_eth_send(struct netif *netif, struct pbuf *p)
{
  err_t retVal = ERR_OK;

  SYS_ARCH_DECL_PROTECT(lev);

  /**
   * This entire function must be protected to preserve
   * the integrity of the transmit pbuf queue.
   */
  SYS_ARCH_PROTECT(lev);
#if !SYS_LIGHTWEIGHT_PROT
  sys_prot_t prevProt = sys_arch_protect()
                        /*
                              uint32_t prevProt = (uint32_t) _get_CPSR() & 0x80;
                              _disable_IRQ();
                        */
#endif

  /**
   * Bump the reference count on the pbuf to prevent it from being
   * freed until we are done with it.
   */
  pbuf_ref(p);

  /* call the actual transmit function */
  retVal = tms570_eth_send_raw(netif, p);

  /* Return to prior interrupt state and return. */
  SYS_ARCH_UNPROTECT(lev);
#if !SYS_LIGHTWEIGHT_PROT
  sys_arch_unprotect(prevProt);
  /*
        if (!prevProt)
                _enable_IRQ();
  */
#endif

  return retVal;
}

/**
 * When called from tms570_eth_send(), the 'lev' lock is held
 */
static err_t
tms570_eth_send_raw(struct netif *netif, struct pbuf *pbuf)
{
  struct tms570_netif_state *nf_state = (struct tms570_netif_state *)netif->state;

  /* Adjust the packet length if less than minimum required */
  size_t pktlen = pbuf->tot_len;
  size_t padlen;
  if (pktlen < EMAC_MIN_PKT_LEN) {
    padlen = EMAC_MIN_PKT_LEN - pktlen;
    pktlen = EMAC_MIN_PKT_LEN;
  } else {
    padlen = 0;
  }

  size_t head = nf_state->txch.head;
  size_t sop = head;
  size_t next = (head + 1) % EMAC_TX_BD_COUNT;
  size_t tail = nf_state->txch.tail;
  struct pbuf *q = pbuf;
  u32_t flags_pktlen = pktlen | EMAC_DSC_FLAG_SOP | EMAC_DSC_FLAG_OWNER;
  volatile struct emac_tx_bd *bd = &nf_state->txch.bds[head];
  volatile struct emac_tx_bd *bd_sop = bd;
  volatile struct emac_tx_bd *bd_next;
  struct pbuf *bd_pbuf = pbuf;

  while (true) {
    bd_next = &nf_state->txch.bds[next];

    rtems_cache_flush_multiple_data_lines(q->payload, q->len);
    bd->flags_pktlen = tms570_eth_swap(flags_pktlen);
    bd->bufptr = tms570_eth_swap_bufptr(q->payload);
    bd->bufoff_len = tms570_eth_swap(q->len);
    bd->next = tms570_eth_swap_txp(bd_next);
    bd->pbuf = bd_pbuf;

    head = next;
    next = (next + 1) % EMAC_TX_BD_COUNT;

    if (next == tail) {
      break;
    }

    q = q->next;

    if (q == NULL) {
      break;
    }

    bd = bd_next;
    bd_pbuf = NULL;
    flags_pktlen = EMAC_DSC_FLAG_OWNER;
  }

  if (padlen > 0 && next != tail) {
    flags_pktlen = EMAC_DSC_FLAG_OWNER;

    bd = bd_next;
    bd->bufptr = tms570_eth_swap_bufptr(&nf_state->emac_ctrl_ram->padding[0]);
    bd->bufoff_len = tms570_eth_swap(padlen);
    bd->pbuf = NULL;

    head = next;
    next = (next + 1) % EMAC_TX_BD_COUNT;
  }

  /* Enough descriptors? */
  if (next == tail) {
    pbuf_free(pbuf);
    return ERR_IF;
  }

  bd->flags_pktlen = tms570_eth_swap(flags_pktlen | EMAC_DSC_FLAG_EOP);
  bd->next = NULL;
  bd->next;
  sys_arch_data_sync_barier();

  bd_sop->eop_increment = ((head - sop) % EMAC_TX_BD_COUNT) - 1;

  if (sop != tail) {
    volatile struct emac_tx_bd *bd_tail =
      &nf_state->txch.bds[(sop - 1) % EMAC_TX_BD_COUNT];
    bd_tail->next = tms570_eth_swap_txp(bd_sop);
    sys_arch_data_sync_barier();

    flags_pktlen = tms570_eth_swap(bd_tail->flags_pktlen);
    if ((flags_pktlen & (EMAC_DSC_FLAG_EOQ | EMAC_DSC_FLAG_OWNER)) == EMAC_DSC_FLAG_EOQ) {
      bd_tail->flags_pktlen = tms570_eth_swap(flags_pktlen & ~EMAC_DSC_FLAG_EOQ);
      tms570_eth_hw_set_TX_HDP(nf_state, bd_sop);
    }
  } else {
    tms570_eth_hw_set_TX_HDP(nf_state, bd_sop);
  }

  nf_state->txch.head = head;
  return ERR_OK;
}

/* EMAC Packet Buffer Sizes and Placement */
#ifdef CONFIG_EMAC_PKT_FRAG_SIZE
  #define EMAC_FRAG_SIZE    CONFIG_EMAC_PKT_FRAG_SIZE
#else
  #define EMAC_FRAG_SIZE      1536
#endif

#ifdef CONFIG_EMAC_ETH_FRAME_SIZE
  #define EMAC_MAX_FLEN    CONFIG_EMAC_ETH_FRAME_SIZE
#else
  #define EMAC_MAX_FLEN       1536
#endif

#ifdef CONFIG_EMAC_NUM_RX_FRAGS
  #define EMAC_NUM_RX_FRAG    CONFIG_EMAC_NUM_RX_FRAGS
#else
  #define EMAC_NUM_RX_FRAG    4
#endif

#ifdef CONFIG_EMAC_NUM_TX_FRAGS
  #define EMAC_NUM_TX_FRAG    CONFIG_EMAC_NUM_TX_FRAGS
#else
  #define EMAC_NUM_TX_FRAG    2
#endif

static
SYS_IRQ_HANDLER_FNC(tms570_eth_process_irq_rx)
{
  struct netif *netif = (struct netif *)sys_irq_handler_get_context();
  struct tms570_netif_state *nf_state = netif->state;
  size_t head = nf_state->rxch.head;

  while (true) {
    volatile struct emac_rx_bd *bd = &nf_state->rxch.bds[head];
    u32_t flags_pktlen = tms570_eth_swap(bd->flags_pktlen);

    if ((flags_pktlen & EMAC_DSC_FLAG_OWNER) != 0) {
      break;
    }

    LINK_STATS_INC(link.recv);
    u32_t full_packet = EMAC_DSC_FLAG_SOP | EMAC_DSC_FLAG_EOP;
    bool recycle;

    if ((flags_pktlen & full_packet) == full_packet) {
      struct pbuf *p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);

      if (p != NULL) {
        u32_t len = tms570_eth_swap(bd->bufoff_len) & 0xFFFF;
        struct pbuf *q = bd->pbuf;
        q->len = len;
        q->tot_len = len;

        if ((*netif->input)(q, netif) != ERR_OK) {
          pbuf_free(q);
        }

        rtems_cache_invalidate_multiple_data_lines(p->payload, p->len);
        bd->bufptr = tms570_eth_swap_bufptr(p->payload);
        bd->bufoff_len = tms570_eth_swap(p->len);
        bd->flags_pktlen = tms570_eth_swap(EMAC_DSC_FLAG_OWNER);
        bd->pbuf = p;
        recycle = false;
      } else {
        LINK_STATS_INC(link.memerr);
        recycle = true;
      }
    } else {
      LINK_STATS_INC(link.err);
      recycle = true;
    }

    if (recycle) {
      LINK_STATS_INC(link.drop);
      struct pbuf *q = bd->pbuf;
      bd->bufptr = tms570_eth_swap_bufptr(q->payload);
      bd->bufoff_len = tms570_eth_swap(q->len);
      bd->flags_pktlen = tms570_eth_swap(EMAC_DSC_FLAG_OWNER);
    }

    bd->next = NULL;
    sys_arch_data_sync_barier();

    /* Acknowledge that this packet is processed */
    EMACRxCPWrite(nf_state->emac_base, 0, (unsigned int)bd);

    volatile struct emac_rx_bd *prev_bd =
      &nf_state->rxch.bds[(head - 1) % EMAC_RX_BD_COUNT];
    prev_bd->next = tms570_eth_swap_rxp(bd);

    if ((tms570_eth_swap(prev_bd->flags_pktlen) & EMAC_DSC_FLAG_EOQ) != 0) {
      tms570_eth_hw_set_RX_HDP(nf_state, bd);
    }

    head = (head + 1) % EMAC_RX_BD_COUNT;
  }

  nf_state->rxch.head = head;
  EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_RX);
}

static
SYS_IRQ_HANDLER_FNC(tms570_eth_process_irq_tx)
{
  struct netif *netif = (struct netif *)sys_irq_handler_get_context();
  struct tms570_netif_state *nf_state = netif->state;

  size_t head = nf_state->txch.head;
  size_t tail = nf_state->txch.tail;
  volatile struct emac_tx_bd *bd = &nf_state->txch.bds[tail];
  volatile struct emac_tx_bd *bd_sof = bd;

  while (tail != head) {
    u32_t flags_pktlen = tms570_eth_swap(bd->flags_pktlen);

    if ((flags_pktlen & EMAC_DSC_FLAG_OWNER) != 0) {
      break;
    }

    tail = (tail + bd->eop_increment) % EMAC_TX_BD_COUNT;
    bd = &nf_state->txch.bds[tail];
    flags_pktlen = tms570_eth_swap(bd->flags_pktlen);

    EMACTxCPWrite(nf_state->emac_base, CHANNEL, (uint32_t)bd);
    pbuf_free(bd_sof->pbuf);
    LINK_STATS_INC(link.xmit);

    if ((flags_pktlen & EMAC_DSC_FLAG_EOQ) != 0) {
      bd = &nf_state->txch.bds[(tail + 1) % EMAC_TX_BD_COUNT];
      flags_pktlen = tms570_eth_swap(bd->flags_pktlen);

      if ((flags_pktlen & EMAC_DSC_FLAG_OWNER) != 0) {
        tms570_eth_hw_set_TX_HDP(nf_state, bd);
      }
    }

    tail = (tail + 1) % EMAC_TX_BD_COUNT;
    bd = &nf_state->txch.bds[tail];
    bd_sof = bd;
  }

  nf_state->txch.tail = tail;

  EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_TX);
}

static void
tms570_eth_hw_set_RX_HDP(struct tms570_netif_state *nf_state, volatile struct emac_rx_bd *new_head)
{
  /* Writes to RX HDP are allowed
                                                 * only when it is 0
                                                 */
  while (nf_state->emac_base->RXHDP[CHANNEL] != 0) {
    tms570_eth_debug_printf("HW -RX- is slacking!!!\n");
    sys_arch_delay(10);
  }
  tms570_eth_debug_printf("setting RX HDP");
  EMACRxHdrDescPtrWrite(
    nf_state->emac_base,
    (uint32_t)new_head,
    CHANNEL);
}
static void
tms570_eth_hw_set_TX_HDP(struct tms570_netif_state *nf_state, volatile struct emac_tx_bd *new_head)
{
  /* Writes to RX HDP are allowed
                                                 * only when it is 0
                                                 */
  while (nf_state->emac_base->TXHDP[CHANNEL] != 0) {
    tms570_eth_debug_printf("HW -TX- is slacking!!!\n");
  }
  tms570_eth_debug_printf("setting TX HDP");
  EMACTxHdrDescPtrWrite(
    nf_state->emac_base,
    (uint32_t)new_head,
    CHANNEL);
}

#if TMS570_NETIF_DEBUG

void
tms570_eth_debug_print_info(struct netif *netif)
{
  struct tms570_netif_state *nf_state = netif->state;

  tms570_eth_debug_show_rx(nf_state);
  tms570_eth_debug_show_tx(nf_state);
  tms570_eth_debug_print_HDP(nf_state);
}

void
tms570_eth_debug_print_HDP(struct tms570_netif_state *nf_state)
{
  tms570_eth_debug_printf("RX HDP = %d\n", tms570_eth_debug_get_BD_num(
                            (void *)nf_state->emac_base->RXHDP[0], nf_state));
  tms570_eth_debug_printf("TX HDP = %d\n", tms570_eth_debug_get_BD_num(
                            (void *)nf_state->emac_base->TXHDP[0], nf_state));
}

int
tms570_eth_debug_get_BD_num(volatile void *ptr, struct tms570_netif_state *nf_state)
{
  uintptr_t i = (struct emac_rx_bd *) ptr - &nf_state->emac_ctrl_ram->rx_bds[0];

  if (i < EMAC_RX_BD_COUNT) {
    return (int) i;
  }

  i = (struct emac_tx_bd *) ptr - &nf_state->emac_ctrl_ram->tx_bds[0];

  if (i < EMAC_TX_BD_COUNT) {
    return (int) i;
  }

  return -1;
}

void
tms570_eth_debug_print_rxch(struct tms570_netif_state *nf_state)
{
  tms570_eth_debug_printf("head = %zu\n", nf_state->rxch.head);
}
void
tms570_eth_debug_print_txch(struct tms570_netif_state *nf_state)
{
  tms570_eth_debug_printf("head = %zu, tail %zu\n", nf_state->txch.head, nf_state->txch.tail);
}
void
tms570_eth_debug_show_BD_chain_rx(volatile struct emac_rx_bd *curr_bd,
                                  struct tms570_netif_state *nf_state)
{
  int count = 0;

  while (curr_bd != NULL) {
    tms570_eth_debug_printf("%d ", tms570_eth_debug_get_BD_num(curr_bd, nf_state));
    curr_bd = tms570_eth_swap_rxp(curr_bd->next);
    count++;
  }
  tms570_eth_debug_printf(" count = %d\n", count);
}

void
tms570_eth_debug_show_BD_chain_tx(volatile struct emac_tx_bd *curr_bd,
                                  struct tms570_netif_state *nf_state)
{
  int count = 0;

  while (curr_bd != NULL) {
    tms570_eth_debug_printf("%d ", tms570_eth_debug_get_BD_num(curr_bd, nf_state));
    curr_bd = tms570_eth_swap_txp(curr_bd->next);
    count++;
  }
  tms570_eth_debug_printf(" count = %d\n", count);
}

void
tms570_eth_debug_show_rx(struct tms570_netif_state *nf_state)
{
  tms570_eth_debug_printf("!!!RX!!!\n");
  tms570_eth_debug_print_rxch(nf_state);
  tms570_eth_debug_printf("active chain\n");
  tms570_eth_debug_show_BD_chain_rx(&nf_state->rxch.bds[nf_state->rxch.head], nf_state);
}
void
tms570_eth_debug_show_tx(struct tms570_netif_state *nf_state)
{

  tms570_eth_debug_printf("!!!TX!!!\n");
  tms570_eth_debug_print_txch(nf_state);
  tms570_eth_debug_printf("active chain\n");
  tms570_eth_debug_show_BD_chain_tx(&nf_state->txch.bds[nf_state->txch.head], nf_state);
}
#endif /* if TMS570_NETIF_DEBUG */
