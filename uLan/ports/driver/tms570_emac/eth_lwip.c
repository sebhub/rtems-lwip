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

//#define DEBUG 1
#include "lwip/tcpip.h" /* includes - lwip/opt.h, lwip/api_msg.h, lwip/netifapi.h, lwip/pbuf.h, lwip/api.h, lwip/sys.h, lwip/timers.h, lwip/netif.h */
#include "lwip/stats.h"
#include "lwip/dhcp.h"
#include "lwip/netifapi.h"
#include "netif/etharp.h" /* includes - lwip/ip.h, lwip/netif.h, lwip/ip_addr.h, lwip/pbuf.h */
#include "eth_lwip.h"
#include "tms570_netif.h"
#include "lwip/prot/ethernet.h"
#include <stdio.h>
#include <inttypes.h>

#define SUCCESS ERR_OK
#define FAILURE ERR_IF

static void eth_lwip_conv_IP_decimal_Str(ip_addr_t ip, uint8_t *ipStr);


void
eth_lwip_get_dhcp_info(struct netif *netif)
{
  if (dhcp_supplied_address(netif)) {
    uint8_t ipString[16]; // FIXME change the functions to use char
    eth_lwip_conv_IP_decimal_Str(netif->ip_addr, ipString);
    printf("Address: %s\n", ipString);
    eth_lwip_conv_IP_decimal_Str(netif->netmask, ipString);
    printf("Netmask: %s\n", ipString);
    eth_lwip_conv_IP_decimal_Str(netif->gw, ipString);
    printf("Gateway: %s\n", ipString);
  } else {
    printf("dhcp not bound\n");
  }
}

int start_networking(
  struct netif  *netif,
  ip_addr_t     *ipaddr,
  ip_addr_t     *netmask,
  ip_addr_t     *gateway,
  unsigned char *mac_addr
)
{
  tcpip_init(NULL, NULL);

  eth_lwip_set_hwaddr(netif, mac_addr);
  netif = netif_add(netif, (const ip4_addr_t *) ipaddr,
    (const ip4_addr_t *) netmask, (const ip4_addr_t *) gateway,
     NULL, tms570_eth_init_netif, tcpip_input);

  if (netif == NULL) {
    return NETIF_ADD_ERR;
  }

  netif_set_default(netif);
  netif_set_up(netif);
  return 0;
}

int
eth_lwip_get_netif_status_cmd(int argc, char *arg[])
{
  stats_display();
  return 0;
}

static void
eth_lwip_conv_IP_decimal_Str(ip_addr_t ip, uint8_t *ipStr)
{
  uint32_t addr;
 #if LWIP_IPV6
  addr = ip.u_addr.ip4.addr;
 #else
  addr = ip.addr;
 #endif

  snprintf((char *)ipStr, 16, "%" PRIu32 ".%" PRIu32 ".%" PRIu32 ".%" PRIu32 "",
           (addr >> 24), ((addr >> 16) & 0xff), ((addr >> 8) & 0xff), (addr & 0xff));
}

/*
* Function to set the MAC address to the interface
* @param   inst_num the instance number
*
* @note    mac_addr[0] is considered MSB
*/
void
eth_lwip_set_hwaddr(struct netif *netif, uint8_t *mac_addr)
{
  int i;

  /* set MAC hardware address */
  for (i = 0; i < ETH_HWADDR_LEN; i++) {
    netif->hwaddr[i] = mac_addr[i];
  }
  netif->hwaddr_len = ETH_HWADDR_LEN;

#ifdef DEBUG
  uint8_t macStr[18];
  eth_lwip_get_hwaddr_str(netif, macStr);
  printf("Setting MAC... %s\r\n", macStr);
#endif
}

void
eth_lwip_get_hwaddr_str(struct netif *netif, uint8_t *macStr)
{
  uint8_t index, outindex = 0;
  char ch;

  for (index = 0; index < netif->hwaddr_len; index++) {
    if (index)
      macStr[outindex++] = ':';
    ch = (netif->hwaddr[index] >> 4);
    macStr[outindex++] = (ch < 10) ? (ch + '0') : (ch - 10 + 'A');
    ch = (netif->hwaddr[index] & 0xf);
    macStr[outindex++] = (ch < 10) ? (ch + '0') : (ch - 10 + 'A');
  }
  macStr[outindex] = 0;
}
