/**
 *  \file   mdio.h
 *
 *  \brief  MDIO APIs and macros.
 *
 *   This file contains the driver API prototypes and macro definitions.
 */

/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
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
*
*/

#ifndef __MDIO_H__
#define __MDIO_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct mdioControl {
    void (*init)(struct mdioControl *self, unsigned int mdioInputFreq,
              unsigned int mdioOutputFreq);
    unsigned int (*phyAliveStatusGet)(struct mdioControl *self);
    unsigned int (*phyLinkStatusGet)(struct mdioControl *self);
    unsigned int (*phyRegRead)(struct mdioControl *self, unsigned int phyAddr,
                               unsigned int regNum, unsigned short *dataPtr);
    void (*phyRegWrite)(struct mdioControl *self, unsigned int phyAddr,
                        unsigned int regNum, unsigned short RegVal);
} mdioControl;

/*
** Prototypes for the APIs
*/

static inline void MDIOInit(mdioControl *self, unsigned int mdioInputFreq,
    unsigned int mdioOutputFreq)
{
  return (*self->init)(self, mdioInputFreq, mdioOutputFreq);
}

static inline unsigned int MDIOPhyAliveStatusGet(mdioControl *self)
{
  return (*self->phyAliveStatusGet)(self);
}

static inline unsigned int MDIOPhyLinkStatusGet(mdioControl *self)
{
  return (*self->phyLinkStatusGet)(self);
}

static inline unsigned int MDIOPhyRegRead(mdioControl *self,
    unsigned int phyAddr, unsigned int regNum, unsigned short *dataPtr)
{
  return (*self->phyRegRead)(self, phyAddr, regNum, dataPtr);
}

static inline void MDIOPhyRegWrite(mdioControl *self, unsigned int phyAddr,
    unsigned int regNum, unsigned short regVal)
{
  return (*self->phyRegWrite)(self, phyAddr, regNum, regVal);
}

#ifdef __cplusplus
}
#endif
#endif /* __MDIO_H__ */
