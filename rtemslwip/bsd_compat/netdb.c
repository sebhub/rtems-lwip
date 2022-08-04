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

#include <lwip/netdb.h>
#include <stdio.h>

static uint16_t getserviceport(const char *target_service) {
  FILE* file;
  char service[16];
  char protocol[16];
  uint16_t target_port = 0, port;
  int scan_result;

  if ( target_service == NULL ) {
    return target_port;
  }

  file = fopen(_PATH_SERVICES, "r");
  if ( file == NULL ) {
    return target_port;
  }

  while ( 1 ) {
    scan_result = fscanf(file, "%15s%hu/%15s%*s%*[^\n]", service, &port, protocol);

    /* Check for end of file */
    if (scan_result == EOF) {
      break;
    }

    /* Check for bad line */
    if (scan_result < 3) {
      continue;
    }

    if ( strcmp(target_service, service) == 0 ) {
      target_port = port;
      break;
    }
  }

  fclose(file);
  return target_port;
}

#undef getaddrinfo
int getaddrinfo(
  const char *nodename,
  const char *servname,
  const struct addrinfo *hints,
  struct addrinfo **res)
{
  char aport[16];
  itoa(getserviceport(servname), aport, 10);
  return lwip_getaddrinfo(nodename, aport, hints, res);
}

#undef freeaddrinfo
void freeaddrinfo(struct addrinfo *a1)
{
  lwip_freeaddrinfo(a1);
}

const char *gai_strerror(int err)
{
  switch ( err ) {
  case EAI_OVERFLOW: return "EAI_OVERFLOW";
  case EAI_SOCKTYPE: return "EAI_SOCKTYPE";
  case EAI_SYSTEM: return "EAI_SYSTEM";
  case EAI_BADHINTS: return "EAI_BADHINTS";
  case EAI_PROTOCOL: return "EAI_PROTOCOL";
  case EAI_NONAME: return "EAI_NONAME";
  case EAI_SERVICE: return "EAI_SERVICE";
  case EAI_FAIL: return "EAI_FAIL";
  case EAI_MEMORY: return "EAI_MEMORY";
  case EAI_FAMILY: return "EAI_FAMILY";
  default: return "UNKNOWN EAI";
  }
}

#include <lwip/if_api.h>

#undef if_nametoindex
unsigned int if_nametoindex(const char *ifname)
{
  return lwip_if_nametoindex(ifname);
}

int res_init()
{
  return 0;
}
