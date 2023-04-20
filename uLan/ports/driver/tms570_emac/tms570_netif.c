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
#include <assert.h>
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

#define LINK_SPEED_OF_YOUR_NETIF_IN_BPS 10000000

/* Number of EMAC Instances */

#define DEFAULT_PHY_ADDR            0x1
#define FIND_FIRST_PHY_ALIVE        1 /* or use default (phy_address: 1) */
#define NUM_OF_PHYs             32

/* Size of the Buffer descriptor defined by the EMAC in bytes */
#define SIZE_OF_DESC                16

/* Channel number used for for RX, TX, unicast, broadcast or damaged frames;
 * there are different channels for rx and tx operations (i.e. RXCH0 != TXCH0) */
#define CHANNEL                 0

/* take in account oversized frames */
#define MAX_TRANSFER_UNIT           1500

#ifndef TMS570_BALL_K19_MII_RXCLK
  #define TMS570_BALL_K19_MII_RXCLK TMS570_BALL_K19_MII_RX_CLK
#endif

/* WARNING!
 * Be very carefull when setting this value. We have to keep in mind
 * that pbuf_alloc(..., PBUF_POOL) will usualy return a chain of PBUFs
 * pointing to the statically preallocated buffers (of the same size).
 * The problem is that if we ask to allocate 300 bytes whereby the size
 * of the statically preallocated PBUFs (PBUF_POOL_BUFSIZE) is 256, we
 * will get a chain containing two PBUFs -- one *reporting* its size to
 * be 256 bytes, the other one 44 bytes.
 * Everything seems to be just fine however during RX, after we call
 * netif->input(pbuf, netif) we have to newly allocate the PBUF(s) and
 * properly set the apropriate BDs. This will however work only if the
 * number of the freed BDs is the same as the number of the BDs newly
 * initialized. One possible situation when this may fail is when multiple
 * non-256 byte sized PBUFs will move near to each other, i.e. 3 BDs:
 * 256 B, 44 B, 44 B -- 344 bytes will be freed (3 BDs) but the new call
 * to pbuf_alloc(...) will return a chain comprising only two PBUFs
 * (256 B, 88 B).
 * This is the implementation limitation. The PBUF_LEN_MAX should therefore
 * be multiple of PBUF_POOL_BUFSIZE
 */
#define PBUF_LEN_MAX                (PBUF_POOL_BUFSIZE * 6)

/* Maximum number of PBUFs preallocated in the driver
 * init function to be used for the RX
 */
#define MAX_RX_PBUF_ALLOC           10
#define MIN_PKT_LEN             60

/* Define those to better describe the network interface. */
#define IFNAME0                 'e'
#define IFNAME1                 'n'

/* Time to wait for autonegotiation in ticks. */
#define TICKS_PHY_AUTONEG           4000

/* startup init indicator */
static bool initialized = false;

#define pk(...) do { rtems_interrupt_level level; rtems_interrupt_disable(level); printk(__VA_ARGS__); rtems_interrupt_enable(level); } while (0)

/*private?*/
#if !defined(__TI_COMPILER_VERSION__)
static
#endif /*__TI_COMPILER_VERSION__*/
SYS_IRQ_HANDLER_FNC(tms570_eth_irq);
static void tms570_eth_hw_set_RX_HDP(struct tms570_netif_state *nf_state, volatile struct emac_rx_bd *new_head);
static void tms570_eth_hw_set_TX_HDP(struct tms570_netif_state *nf_state, volatile struct emac_tx_bd *new_head);
static void tms570_eth_hw_set_hwaddr(struct tms570_netif_state *nf_state, uint8_t *mac_addr);
static void tms570_eth_process_irq_rx(void *arg);
static void tms570_eth_process_irq_tx(void *arg);
static void tms570_eth_process_irq_request(void *argument);
static void tms570_eth_process_irq(void *argument);
struct netif *tms570_eth_get_netif(uint32_t instance_number);
static err_t tms570_eth_send(struct netif *netif, struct pbuf *p);
static err_t tms570_eth_send_raw(struct netif *netif, struct pbuf *pbuf);
static err_t tms570_eth_init_hw(struct tms570_netif_state *nf_state);
static err_t tms570_eth_init_hw_post_init(struct tms570_netif_state *nf_state);
static err_t tms570_eth_init_interrupt(struct tms570_netif_state *nf_state);
static err_t tms570_eth_init_find_PHY(struct tms570_netif_state *nf_state);
static err_t tms570_eth_init_control_structures(struct netif *netif);
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
  nf_state->emac_ctrl_ram = EMAC_CTRL_RAM_BASE;
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

static err_t
tms570_eth_init_control_structures(struct netif *netif)
{
  err_t res;
  sys_thread_t tx_thread_id;
  struct tms570_netif_state *nf_state;

  nf_state = netif->state;

  res = sys_sem_new(&nf_state->intPend_sem, 0);
  if (res != ERR_OK) {
    sys_arch_printk("ERROR! semaphore creation error - 0x%08lx\n", (long)res);
  }
  tx_thread_id = sys_thread_new(0, tms570_eth_process_irq_request, netif, 1024, 3); //zkontrolovat priorita 0
  if (tx_thread_id == 0) {
    sys_arch_printk("ERROR! lwip interrupt thread not created");
    res = !ERR_OK;
  }
  return res;
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
  if ((retVal = tms570_eth_init_control_structures(netif)) != ERR_OK) {
    tms570_eth_debug_printf("tms570_eth_init_control_structures: %d", retVal);
    return retVal;
  }
  tms570_eth_init_buffer_descriptors(nf_state);
  tms570_eth_hw_set_hwaddr(nf_state, netif->hwaddr);
  tms570_eth_init_interrupt(nf_state);
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
tms570_eth_init_interrupt(struct tms570_netif_state *nf_state)
{
  int res;

  res = sys_request_irq(TMS570_IRQ_EMAC_TX, tms570_eth_irq,
                        0, "emac_tx", nf_state);
  if (res < 0) {
    sys_arch_printk("Failed to install tx handler\n");
    return ERR_IF;
  }

  res = sys_request_irq(TMS570_IRQ_EMAC_RX, tms570_eth_irq,
                        0, "emac_rx", nf_state);
  if (res < 0) {
    sys_arch_printk("Failed to install rx handler\n");
    return ERR_IF;
  }
  return ERR_OK;
}
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
  err_t retVal;

  tms570_eth_init_set_pinmux();

  /* Initialize EMAC control module and EMAC module */
  EMACInit(nf_state->emac_ctrl_base, nf_state->emac_base);
  /* Initialize MDIO module (reset) */
  MDIOInit(&nf_state->mdio.base, 0x0, 0x0);

  if ((retVal = tms570_eth_init_find_PHY(nf_state)) != ERR_OK) {
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
  uint32_t num_bd;
  volatile struct emac_rx_bd *curr_rxbd;
  volatile struct emac_rx_bd *prev_rxbd;
  volatile struct emac_rx_bd *head_rxbd;

  struct rxch *rxch;
  struct txch *txch;

  rxch = &(nf_state->rxch);
  txch = &(nf_state->txch);


  /*
   * Use the CPPI RAM to store RX/TX Buffer Descriptors (= BD).
   * 1/2 of CPPI RAM is used for TX BDs, another 1/2 for RX BDs.
   * All the TX BDs are 'owned' by the software. They are initialized
   * as a linked-list forming a ring. They are awaiting the application
   * to append pbuf payload to the them and correctly configure for
   * actual transmission.
   * Only such number of RX BDs is configured that the pbufs can be
   * allocated for (i.e. MAX_RX_PBUF_ALLOC). Pbufs are allocated from
   * the PBUF_POOL (thus the allocated pbufs might be chained).
   * Each payload part of a payload is them used to initialize single
   * RX BD. The RX BDs are then configured to be 'owned' bythe EMAC
   */

  /*
   * Initialize the Descriptor Memory For TX and RX
   * Only Channel 0 is supported for both TX and RX
   */

  /* Initialize all the TX buffer Descriptors */
  txch->head = 0;
  txch->tail = 0;
  txch->bds = (volatile struct emac_tx_bd *)nf_state->emac_ctrl_ram;
  memset(RTEMS_DEVOLATILE(struct emac_tx_bd *, txch->bds), 0x0, sizeof(*txch->bds) * EMAC_TX_BD_COUNT);

  /* Initialize the descriptors for the RX channel */
  curr_rxbd = (volatile struct emac_rx_bd *)(&txch->bds[EMAC_TX_BD_COUNT]);
  head_rxbd = curr_rxbd;
  rxch->active_head = head_rxbd;

  num_bd = ((SIZE_EMAC_CTRL_RAM >> 1) / sizeof(struct emac_rx_bd))-1;
  num_bd = LWIP_MIN(PBUF_POOL_SIZE / 2, num_bd);

  while (num_bd > 0) {
    struct pbuf *p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);
    assert(p != NULL);
    assert(p->next == NULL);

    rtems_cache_invalidate_multiple_data_lines(p->payload, p->len);
    curr_rxbd->bufptr = tms570_eth_swap_bufptr(p->payload);
    curr_rxbd->bufoff_len = tms570_eth_swap(p->len);
    curr_rxbd->flags_pktlen = tms570_eth_swap(EMAC_DSC_FLAG_OWNER);
    curr_rxbd->pbuf = p;

    volatile struct emac_rx_bd *next_rxbd = curr_rxbd + 1;
    curr_rxbd->next = tms570_eth_swap_rxp(next_rxbd);
    curr_rxbd->ring_next = next_rxbd;
    curr_rxbd->ring_prev = prev_rxbd;
    prev_rxbd = curr_rxbd;
    curr_rxbd = next_rxbd;

    --num_bd;
  }

  prev_rxbd->next = NULL;
  prev_rxbd->ring_next = head_rxbd;
  head_rxbd->ring_prev = prev_rxbd;

  sys_arch_data_sync_barier();
  tms570_eth_hw_set_RX_HDP(nf_state, head_rxbd);

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

#if 0
  struct tms570_netif_state *nf_state = (struct tms570_netif_state *)netif->state;
  struct txch *txch = &(nf_state->txch);
  while (txch->head != txch->tail) {
    RTEMS_COMPILER_MEMORY_BARRIER();
    rtems_task_wake_after(1);
  }
#endif

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
  struct txch *txch = &(nf_state->txch);

  /* Adjust the packet length if less than minimum required */
  size_t pktlen = pbuf->tot_len;
  size_t padlen;
  if (pktlen < MIN_PKT_LEN) {
    padlen = MIN_PKT_LEN - pktlen;
    pktlen = MIN_PKT_LEN;
  } else {
    padlen = 0;
  }

  size_t head = txch->head;
  size_t sop = head;
  size_t next = (head + 1) % EMAC_TX_BD_COUNT;
  size_t tail = txch->tail;
  struct pbuf *q = pbuf;
  u32_t flags_pktlen = pktlen | EMAC_DSC_FLAG_SOP | EMAC_DSC_FLAG_OWNER;
  volatile struct emac_tx_bd *bd = &txch->bds[head];
  volatile struct emac_tx_bd *bd_sop = bd;
  volatile struct emac_tx_bd *bd_next;
  struct pbuf *bd_pbuf = pbuf;

  while (true) {
    bd_next = &txch->bds[next];

    rtems_cache_flush_multiple_data_lines(q->payload, q->len);
    bd->flags_pktlen = tms570_eth_swap(flags_pktlen);
    bd->bufptr = tms570_eth_swap_bufptr(q->payload);
    bd->bufoff_len = tms570_eth_swap(q->len & 0xFFFF);
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
    bd->bufptr = tms570_eth_swap_bufptr(pbuf->payload);
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
    volatile struct emac_tx_bd *bd_tail = &txch->bds[(sop - 1) % EMAC_TX_BD_COUNT];
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

  txch->head = head;
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

static void
tms570_eth_process_irq_rx(void *arg)
{
  struct netif *netif = (struct netif *)arg;
  struct tms570_netif_state *nf_state = netif->state;

  rtems_interrupt_level level;
  rtems_interrupt_disable(level);

  volatile struct emac_rx_bd *curr_bd = nf_state->rxch.active_head;

  while (true) {
    u32_t flags_pktlen = tms570_eth_swap(curr_bd->flags_pktlen);

    if ((flags_pktlen & EMAC_DSC_FLAG_OWNER) != 0) {
      break;
    }

    LINK_STATS_INC(link.recv);
    u32_t full_packet = EMAC_DSC_FLAG_SOP | EMAC_DSC_FLAG_EOP;
    bool recycle;

    if ((flags_pktlen & full_packet) == full_packet) {
      struct pbuf *p = pbuf_alloc(PBUF_RAW, PBUF_POOL_BUFSIZE, PBUF_POOL);

      if (p != NULL) {
        u32_t len = tms570_eth_swap(curr_bd->bufoff_len) & 0xFFFF;
        struct pbuf *q = curr_bd->pbuf;
        q->len = len;
        q->tot_len = len;
        //pk("R %08x %08x %u %u\n", q, q->payload, q->len, ntohl(*(uint32_t*)((uint8_t*)q->payload + 0x26)));

        if ((*netif->input)(q, netif) != ERR_OK) {
          pbuf_free(q);
        }

        rtems_cache_invalidate_multiple_data_lines(p->payload, p->len);
        curr_bd->bufptr = tms570_eth_swap_bufptr(p->payload);
        curr_bd->bufoff_len = tms570_eth_swap(p->len);
        curr_bd->flags_pktlen = tms570_eth_swap(EMAC_DSC_FLAG_OWNER);
        curr_bd->pbuf = p;
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
      struct pbuf *q = curr_bd->pbuf;
      curr_bd->bufptr = tms570_eth_swap_bufptr(q->payload);
      curr_bd->bufoff_len = tms570_eth_swap(q->len);
      curr_bd->flags_pktlen = tms570_eth_swap(EMAC_DSC_FLAG_OWNER);
    }

    curr_bd->next = NULL;
    sys_arch_data_sync_barier();

    /* Acknowledge that this packet is processed */
    EMACRxCPWrite(nf_state->emac_base, 0, (unsigned int)curr_bd);

    volatile struct emac_rx_bd *prev_bd = curr_bd->ring_prev;
    prev_bd->next = tms570_eth_swap_rxp(curr_bd);

    if ((tms570_eth_swap(prev_bd->flags_pktlen) & EMAC_DSC_FLAG_EOQ) != 0) {
      tms570_eth_hw_set_RX_HDP(nf_state, curr_bd);
    }

    curr_bd = curr_bd->ring_next;
  }

  nf_state->rxch.active_head = curr_bd;
  rtems_interrupt_enable(level);
}

static void
tms570_eth_process_irq_tx(void *arg)
{
  struct netif *netif = (struct netif *)arg;
  struct tms570_netif_state *nf_state = netif->state;
  struct txch *txch = &(nf_state->txch);

  size_t head = txch->head;
  size_t tail = txch->tail;
  volatile struct emac_tx_bd *bd = &txch->bds[tail];
  volatile struct emac_tx_bd *bd_sof = bd;

  while (tail != head) {
    u32_t flags_pktlen = tms570_eth_swap(bd->flags_pktlen);

    if ((flags_pktlen & EMAC_DSC_FLAG_OWNER) != 0) {
      break;
    }

    tail = (tail + bd->eop_increment) % EMAC_TX_BD_COUNT;
    bd = &txch->bds[tail];
    flags_pktlen = tms570_eth_swap(bd->flags_pktlen);

    EMACTxCPWrite(nf_state->emac_base, CHANNEL, (uint32_t)bd);
    pbuf_free(bd_sof->pbuf);
    LINK_STATS_INC(link.xmit);

    if ((flags_pktlen & EMAC_DSC_FLAG_EOQ) != 0) {
      bd = &txch->bds[(tail + 1) % EMAC_TX_BD_COUNT];
      flags_pktlen = tms570_eth_swap(bd->flags_pktlen);

      if ((flags_pktlen & EMAC_DSC_FLAG_OWNER) != 0) {
        tms570_eth_hw_set_TX_HDP(nf_state, bd);
      }
    }

    tail = (tail + 1) % EMAC_TX_BD_COUNT;
    bd = &txch->bds[tail];
    bd_sof = bd;
  }

  txch->tail = tail;
}

static void
tms570_eth_process_irq(void *argument)
{
  struct netif *netif = (struct netif *)argument;
  struct tms570_netif_state *nf_state;
  uint32_t macints;

  nf_state = netif->state;

  if (nf_state == NULL)
    return;

      rtems_interrupt_level level;
      rtems_interrupt_disable(level);
  while (1) {
    macints = nf_state->emac_base->MACINVECTOR;
    if ((macints & 0xffff) == 0) {
      break;
    }
    if (macints & (0xff<<16)) { //TX interrupt
      tms570_eth_process_irq_tx(netif);
      EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_TX);
    }
    if (macints & (0xff<<0)) { //RX interrupt
      tms570_eth_process_irq_rx(netif);
      EMACCoreIntAck(nf_state->emac_base, EMAC_INT_CORE0_RX);
    }
  }
  sys_arch_unmask_interrupt_source(TMS570_IRQ_EMAC_RX);
  sys_arch_unmask_interrupt_source(TMS570_IRQ_EMAC_TX);
      rtems_interrupt_enable(level);
}

void static
tms570_eth_process_irq_request(void *argument)
{
  struct netif *netif = (struct netif *)argument;
  struct tms570_netif_state *nf_state;

  nf_state = netif->state;

  for (;; ) {
    sys_arch_sem_wait(&nf_state->intPend_sem, 0);
    tcpip_callback((tcpip_callback_fn)tms570_eth_process_irq, netif);
  }
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

#if !defined(__TI_COMPILER_VERSION__)
static
#endif /*__TI_COMPILER_VERSION__*/
SYS_IRQ_HANDLER_FNC(tms570_eth_irq){
  struct tms570_netif_state *nf_state = (struct tms570_netif_state *)sys_irq_handler_get_context();

  sys_arch_mask_interrupt_source(TMS570_IRQ_EMAC_RX);
  sys_arch_mask_interrupt_source(TMS570_IRQ_EMAC_TX);
  if (nf_state != NULL)
    sys_sem_signal_from_ISR(&nf_state->intPend_sem);
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
  if (ptr == NULL)
    return -1;
  return ((uintptr_t)ptr-nf_state->emac_ctrl_ram)/sizeof(struct emac_rx_bd);
}

void
tms570_eth_debug_print_rxch(struct tms570_netif_state *nf_state)
{
  tms570_eth_debug_printf("inactive_head = %d, inactive_tail = %d, active_head = %d, active_tail = %d, freed_pbuf_len = %d\n",
                          tms570_eth_debug_get_BD_num(nf_state->rxch.inactive_head, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->rxch.inactive_tail, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->rxch.active_head, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->rxch.active_tail, nf_state),
                          nf_state->rxch.freed_pbuf_len);
}
void
tms570_eth_debug_print_txch(struct tms570_netif_state *nf_state)
{
  tms570_eth_debug_printf("inactive_head = %d, inactive_tail = %d, active_head = %d, active_tail = %d\n",
                          tms570_eth_debug_get_BD_num(nf_state->txch.inactive_head, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->txch.inactive_tail, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->txch.active_head, nf_state),
                          tms570_eth_debug_get_BD_num(nf_state->txch.active_tail, nf_state));
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
  tms570_eth_debug_printf("inactive chain\n");
  tms570_eth_debug_show_BD_chain_rx(nf_state->rxch.inactive_head, nf_state);
  tms570_eth_debug_printf("active chain\n");
  tms570_eth_debug_show_BD_chain_rx(nf_state->rxch.active_head, nf_state);
}
void
tms570_eth_debug_show_tx(struct tms570_netif_state *nf_state)
{

  tms570_eth_debug_printf("!!!TX!!!\n");
  tms570_eth_debug_print_txch(nf_state);
  tms570_eth_debug_printf("inactive chain\n");
  tms570_eth_debug_show_BD_chain_tx(nf_state->txch.inactive_head, nf_state);
  tms570_eth_debug_printf("active chain\n");
  tms570_eth_debug_show_BD_chain_tx(nf_state->txch.active_head, nf_state);
}
#endif /* if TMS570_NETIF_DEBUG */
