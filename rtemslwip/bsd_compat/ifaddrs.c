/*
 * Copyright (C) 2022 On-Line Applications Research Corporation (OAR)
 * Written by Kinsey Moore <kinsey.moore@oarcorp.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <lwip/netif.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
static struct ifaddrs *create_ifaddr_from_netif(struct netif *netif)
{
  struct ifaddrs *ifaddr = malloc(sizeof(struct ifaddrs));
  if ( ifaddr == NULL ) {
    return NULL;
  }
  ifaddr->ifa_next = NULL;
  ifaddr->ifa_name = netif->name;

  ifaddr->ifa_flags = 0;
  if ( netif->flags & NETIF_FLAG_UP ) {
    ifaddr->ifa_flags |= IFF_RUNNING;
  }
  if ( netif->flags & NETIF_FLAG_LINK_UP ) {
    ifaddr->ifa_flags |= IFF_UP;
  }
  if ( netif->flags & NETIF_FLAG_BROADCAST ) {
    ifaddr->ifa_flags |= IFF_BROADCAST;
  }

  ifaddr->ifa_addr = malloc(sizeof(struct sockaddr));
  ifaddr->ifa_netmask = malloc(sizeof(struct sockaddr));
  ifaddr->ifa_dstaddr = malloc(sizeof(struct sockaddr));
  if ( ifaddr->ifa_addr == NULL
       || ifaddr->ifa_netmask == NULL
       || ifaddr->ifa_dstaddr == NULL ) {
    freeifaddrs(ifaddr);
    return NULL;
  }

  ifaddr->ifa_addr->sa_family = AF_INET;
  ifaddr->ifa_netmask->sa_family = AF_INET;
  ifaddr->ifa_dstaddr->sa_family = AF_INET;
#define inet_addr_from_ip4addr(target_inaddr, source_ipaddr) ((target_inaddr)->s_addr = ip4_addr_get_u32(source_ipaddr))
  inet_addr_from_ip4addr(&((struct sockaddr_in *)ifaddr->ifa_addr)->sin_addr, &netif->ip_addr.u_addr.ip4);
  inet_addr_from_ip4addr(&((struct sockaddr_in *)ifaddr->ifa_netmask)->sin_addr, &netif->netmask.u_addr.ip4);
  ((struct sockaddr_in *)ifaddr->ifa_netmask)->sin_addr.s_addr = netif->ip_addr.u_addr.ip4.addr & netif->netmask.u_addr.ip4.addr;
  return ifaddr;
}

int getifaddrs(struct ifaddrs **ifaddrs)
{
  struct netif *netif;
  *ifaddrs = NULL;

  LWIP_ASSERT_CORE_LOCKED();

  NETIF_FOREACH(netif) {
    /* create ifaddr from netif */
    struct ifaddrs *ifaddr = create_ifaddr_from_netif(netif);
    if ( ifaddr == NULL ) {
      freeifaddrs(*ifaddrs);
      *ifaddrs = NULL;
      errno = ENOMEM;
      return -1;
    }
    ifaddr->ifa_next = *ifaddrs;
    *ifaddrs = ifaddr;
  }

  return 0;
}

void freeifaddrs(struct ifaddrs *ifaddrs)
{
  struct ifaddrs *ifaddr;
  while ( ifaddrs != NULL ) {
    ifaddr = ifaddrs;
    ifaddrs = ifaddrs->ifa_next;
    free(ifaddr->ifa_addr);
    free(ifaddr->ifa_netmask);
    free(ifaddr->ifa_dstaddr);
    free(ifaddr);
  }
}
