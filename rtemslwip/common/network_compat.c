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

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <lwip/ip4_addr.h>
#include <lwip/ip6_addr.h>
#include <lwip/sockets.h>
#include <lwip/netdb.h>

in_addr_t inet_addr(const char *cp)
{
  return ipaddr_addr(cp);
}

int inet_aton(const char *cp, struct in_addr *addr)
{
  return ip4addr_aton(cp, (ip4_addr_t*)addr);
}

char *inet_ntoa(struct in_addr addr)
{
  return ip4addr_ntoa((const ip4_addr_t*)&(addr));
}

char *inet_ntoa_r(struct in_addr addr, char *buf, socklen_t buflen)
{
  return ip4addr_ntoa_r((const ip4_addr_t*)&(addr), buf, buflen);
}

#undef htons
uint16_t htons(uint16_t x)
{
  return lwip_htons(x);
}

int
getnameinfo(const struct sockaddr *sa, socklen_t salen, char *node,
    size_t nodelen, char *service, size_t servicelen, int flags)
{
    int af;
    const struct sockaddr_in *sa_in = (const struct sockaddr_in *)sa;

    (void) salen;

    af = sa->sa_family;
    if (af != AF_INET) {
        return EAI_FAMILY;
    }

    if ((flags & NI_NAMEREQD) != 0) {
        return EAI_NONAME;
    }

    /* FIXME: This return just the address value. Try resolving instead. */
    if (node != NULL && nodelen > 0) {
        if (lwip_inet_ntop(af, &sa_in->sin_addr, node, nodelen) == NULL) {
            return EAI_FAIL;
        }
    }

    if (service != NULL && servicelen > 0) {
        in_port_t port = ntohs(sa_in->sin_port);
        int rv;

        rv = snprintf(service, servicelen, "%u", port);
        if (rv <= 0) {
            return EAI_FAIL;
        } else if ((unsigned)rv >= servicelen) {
            return EAI_OVERFLOW;
        }
    }

    return 0;
}
