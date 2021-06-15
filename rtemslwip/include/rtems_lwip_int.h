/*
 *
 * RTEMS Project (https://www.rtems.org/)
 *
 * Copyright (c) 2021 Vijay Kumar Banerjee <vijay@rtems.org>.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RTEMS_LWIP_INT_H
#define RTEMS_LWIP_INT_H

#include <stdint.h>

extern int rtems_lwip_sysdefs_AF_UNSPEC;
extern int rtems_lwip_sysdefs_AF_UNIX;
extern int rtems_lwip_sysdefs_AF_INET;
extern int rtems_lwip_sysdefs_AF_INET6;
extern int rtems_lwip_sysdefs_PF_UNSPEC;
extern int rtems_lwip_sysdefs_PF_UNIX;
extern int rtems_lwip_sysdefs_PF_INET;
extern int rtems_lwip_sysdefs_PF_INET6;
extern int rtems_lwip_sysdefs_SOCK_STREAM;
extern int rtems_lwip_sysdefs_SOCK_DGRAM;
extern int rtems_lwip_sysdefs_SOCK_RAW;
extern int rtems_lwip_sysdefs_sockaddr_in_size;
extern int rtems_lwip_sysdefs_sockaddr_in6_size;

int rtems_lwip_sysdefs_sockaddr_get_len( const void *sockaddr );
int rtems_lwip_sysdefs_sockaddr_get_family( const void *sockaddr );
uint16_t rtems_lwip_sysdefs_sockaddr_in_get_sin_port( const void *sockaddr );
uint32_t rtems_lwip_sysdefs_sockaddr_in_get_sin_addr( const void *sockaddr );
uint16_t rtems_lwip_sysdefs_sockaddr_in6_get_sin6_port( const void *sockaddr );
const uint8_t *rtems_lwip_sysdefs_sockaddr_in6_get_sin6_addr_ptr_const(
  const void *sockaddr );
uint8_t *rtems_lwip_sysdefs_sockaddr_in6_get_sin6_addr_ptr(
  void *sockaddr );
uint32_t rtems_lwip_sysdefs_sockaddr_in6_get_sin6_flowinfo(
  const void *sockaddr );
uint32_t rtems_lwip_sysdefs_sockaddr_in6_get_sin6_scope_id(
  const void *sockaddr );

void rtems_lwip_sysdefs_sockaddr_set_len(
  void *sockaddr,
  int   len
);
void rtems_lwip_sysdefs_sockaddr_set_family(
  void *sockaddr,
  int   family
);
void rtems_lwip_sysdefs_sockaddr_in_set_sin_port(
  void    *sockaddr,
  uint16_t port
);
void rtems_lwip_sysdefs_sockaddr_in_set_sin_addr(
  void    *sockaddr,
  uint32_t addr
);
void rtems_lwip_sysdefs_sockaddr_in6_set_sin6_port(
  void    *sockaddr,
  uint16_t port
);
void rtems_lwip_sysdefs_sockaddr_in6_set_sin6_flowinfo(
  void    *sockaddr,
  uint32_t flowinfo
);
void rtems_lwip_sysdefs_sockaddr_in6_set_sin6_scope_id(
  void    *sockaddr,
  uint32_t scope_id
);

#endif /*RTEMS_LWIP_INT_H*/
