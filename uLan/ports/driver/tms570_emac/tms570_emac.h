#ifndef EMAC_H_
#define EMAC_H_

#include <stdbool.h>
#include "netif/etharp.h"
#include "lwip/sys.h"
#include "bsp/tms570.h"
#include "hw_mdio.h"

/*to rtems*/
#define EMAC_CTRL_RAM_BASE      (0xFC520000U)
#define SIZE_EMAC_CTRL_RAM        0x2000U




/* Packet Flags EMAC_Desc - tx and rx */
#define EMAC_DSC_FLAG_SOP         0x80000000u
#define EMAC_DSC_FLAG_EOP         0x40000000u
#define EMAC_DSC_FLAG_OWNER       0x20000000u
#define EMAC_DSC_FLAG_EOQ         0x10000000u
#define EMAC_DSC_FLAG_TDOWNCMPLT  0x08000000u
#define EMAC_DSC_FLAG_PASSCRC     0x04000000u
/* Packet Flags - in addition to rx */
#define EMAC_DSC_FLAG_JABBER      0x02000000u
#define EMAC_DSC_FLAG_OVERSIZE    0x01000000u
#define EMAC_DSC_FLAG_FRAGMENT    0x00800000u
#define EMAC_DSC_FLAG_UNDERSIZED  0x00400000u
#define EMAC_DSC_FLAG_CONTROL     0x00200000u
#define EMAC_DSC_FLAG_OVERRUN     0x00100000u
#define EMAC_DSC_FLAG_CODEERROR   0x00080000u
#define EMAC_DSC_FLAG_ALIGNERROR  0x00040000u
#define EMAC_DSC_FLAG_CRCERROR    0x00020000u
#define EMAC_DSC_FLAG_NOMATCH     0x00010000u

/********** TI structs ***********/

/* EMAC TX Buffer descriptor data structure */
struct emac_tx_bd {
  volatile struct emac_tx_bd *next;
  volatile u8_t *bufptr;
  volatile u32_t bufoff_len;
  volatile u32_t flags_pktlen;

  /* helper to know which pbuf this tx bd corresponds to */
  struct pbuf *pbuf;

  /* The EOP increment is used to get from the SOP BD to the EOP BD */
  size_t eop_increment;
};

/* EMAC RX Buffer descriptor data structure */
struct emac_rx_bd {
  volatile struct emac_rx_bd *next;
  volatile u8_t *bufptr;
  volatile u32_t bufoff_len;
  volatile u32_t flags_pktlen;

  /* helper to know which pbuf this rx bd corresponds to */
  struct pbuf *pbuf;
};

/**
 * Helper struct to hold the data used to operate on a particular
 * receive channel
 */
struct rxch {
  /*
   * This is the index of the BD containing the least recently received
   * Ethernet frame not handed over to the stack.
   */
  size_t head;

  volatile struct emac_rx_bd *bds;
};

/**
 * Helper struct to hold the data used to operate on a particular
 * transmit channel
 */
struct txch {
  /*
   * This is the index of the BD for the start of the next Ethernet frame to
   * transmit.
   */
  size_t head;

  /*
   * This is the index of the BD containing the Ethernet frame least recently
   * handed over to the EMAC.
   */
  size_t tail;

  volatile struct emac_tx_bd *bds;
};

#define EMAC_RX_BD_COUNT LWIP_MIN((PBUF_POOL_SIZE / 2), 128)

#define EMAC_TX_BD_COUNT 128

#define EMAC_MIN_PKT_LEN 60

struct cppi_ram {
  struct emac_rx_bd rx_bds[EMAC_RX_BD_COUNT];
  struct emac_tx_bd tx_bds[EMAC_TX_BD_COUNT];
  uint8_t padding[EMAC_MIN_PKT_LEN];
};

/**
 * Helper struct to hold private data used to operate the ethernet interface.
 */
struct tms570_netif_state {
  /* emac instance number */
  u32_t inst_num;

  /* EMAC configuration register-window base address */
  volatile tms570_emacm_t *emac_base;

  /* EMAC controller base address */
  volatile tms570_emacc_t *emac_ctrl_base;
  struct cppi_ram *emac_ctrl_ram;

  /* The RX/TX channel 0 state description
   * (keeps track of used/freed Buffer Descriptors)
   */
  struct txch txch;
  struct rxch rxch;

  /* MDIO module control */
  tiMDIOControl mdio;

  u32_t phy_addr;
  uint32_t waitTicksForPHYAneg;
};

/********************************************** End of Statistics **********************************************/

#endif /* EMAC_H_ */
