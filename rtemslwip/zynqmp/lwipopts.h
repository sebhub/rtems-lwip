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

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define SYS_LIGHTWEIGHT_PROT 1

#define NO_SYS 0
#define LWIP_SOCKET                     1
#define LWIP_COMPAT_SOCKETS 1
#define LWIP_NETCONN 1

#define LWIP_ARP 1
#define LWIP_NETIF_API 1
/* Required for socketpair implementation */
#define LWIP_NETIF_LOOPBACK 1
#define LWIP_IPV6                       1
#define LWIP_IPV4                       1
#define LWIP_TIMEVAL_PRIVATE 0
#define LWIP_DNS                       1

#define LWIP_CALLBACK_API 1

#define MEM_ALIGNMENT           64
#define MEM_SIZE                2 * 1024 * 1024
#define MEMP_NUM_PBUF           32
#define MEMP_NUM_UDP_PCB        4
#define MEMP_NUM_TCP_PCB        32
#define MEMP_NUM_TCP_PCB_LISTEN 8
#define MEMP_NUM_TCP_SEG        256

#define PBUF_POOL_SIZE          256
#define PBUF_POOL_BUFSIZE       1600
#define PBUF_LINK_HLEN          16

#define ARP_TABLE_SIZE 10
#define ARP_QUEUEING 1

#define ICMP_TTL                255

#define IP_OPTIONS              1
#define IP_FORWARD              0
#define IP_REASSEMBLY 1
#define IP_FRAG 1
#define IP_REASS_BUFSIZE 5760
#define IP_FRAG_MAX_MTU 1500
#define IP_DEFAULT_TTL 255
#define LWIP_CHKSUM_ALGORITHM 3

#define LWIP_UDP                1
#define UDP_TTL                 255

#define LWIP_TCP                1
#define TCP_MSS                 1576
#define TCP_SND_BUF             16 * 1024
#define TCP_WND                 6 * 1024
#define TCP_TTL                 255
#define TCP_MAXRTX              12
#define TCP_SYNMAXRTX           4
#define TCP_QUEUE_OOSEQ         1
#define TCP_SND_QUEUELEN        16 * TCP_SND_BUF / TCP_MSS
#define CHECKSUM_GEN_TCP  1
#define CHECKSUM_GEN_UDP  1
#define CHECKSUM_GEN_IP   1
#define CHECKSUM_CHECK_TCP  1
#define CHECKSUM_CHECK_UDP  1
#define CHECKSUM_CHECK_IP   1

#define TCP_TMR_INTERVAL       250
#define TCP_FAST_INTERVAL      250
#define TCP_SLOW_INTERVAL      500

#define NO_SYS_NO_TIMERS 1
#define MEMP_SEPARATE_POOLS 1
#define MEMP_NUM_FRAG_PBUF 256
#define IP_OPTIONS_ALLOWED 0
#define TCP_OVERSIZE TCP_MSS
#define LWIP_COMPAT_MUTEX 0
#define LWIP_ALLOW_MEM_FREE_FROM_OTHER_CONTEXT 1

#define LWIP_DHCP               1
#define DHCP_DOES_ARP_CHECK     1

#define DBG_TYPES_ON DBG_LEVEL_WARNING

#define LWIP_STATS                      0
#define LWIP_STATS_DISPLAY              0
#define LWIP_STATS_POSIX                0

#define CONFIG_LINKSPEED_AUTODETECT 1
#define TCPIP_MBOX_SIZE                 20
#define DEFAULT_TCP_RECVMBOX_SIZE       20
#define DEFAULT_ACCEPTMBOX_SIZE         5

#define tskIDLE_PRIORITY RTEMS_MAXIMUM_PRIORITY
#define portTICK_RATE_MS ( rtems_clock_get_ticks_per_second() * 1000 )
#define vTaskDelay( x ) sys_arch_delay( x )

#endif /* __LWIPOPTS_H__ */
