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

#include "netif/xtopology.h"
#include "xparameters.h"
#include "xparameters_ps.h"
#include <netif/xemacpsif.h>

struct xtopology_t xtopology[] = {
  {
    ZYNQMP_EMACPS_3_BASEADDR,
    xemac_type_emacps,
    0x0,
    0x0,
    0xF8F00100,
    XPS_GEM3_INT_ID,
  },
  {
    ZYNQMP_EMACPS_2_BASEADDR,
    xemac_type_emacps,
    0x0,
    0x0,
    0xF8F00100,
    XPS_GEM2_INT_ID,
  },
  {
    ZYNQMP_EMACPS_1_BASEADDR,
    xemac_type_emacps,
    0x0,
    0x0,
    0xF8F00100,
    XPS_GEM1_INT_ID,
  },
  {
    ZYNQMP_EMACPS_0_BASEADDR,
    xemac_type_emacps,
    0x0,
    0x0,
    0xF8F00100,
    XPS_GEM0_INT_ID,
  },
};

int xtopology_n_emacs = 4;
