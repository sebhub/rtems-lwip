#ifndef __LWIP_BBB_H__
#define __LWIP_BBB_H__

#include "lwip/ip_addr.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef LEN_MAC_ADDRESS
#define LEN_MAC_ADDRESS                    (6)
#endif

/**
 * @brief The possible IP modes, Static, DHCP or AutoIP
 * For use in the lwipIf struct.
 */
typedef enum IPMode
{
    IPADDR_USE_STATIC,
    IPADDR_USE_DHCP,
    IPADDR_USE_AUTOIP 
} IPMode;

/**
 * @brief Used to pass the addresses for the LWIP
 * initialization routines(Ip address, NetMask, GW, mode and MAC).
 * 
 */
typedef struct lwipIf
{
    ip4_addr_t ipAddr; /* IP Address */
    ip4_addr_t netMask; /* Net Mask */
    ip4_addr_t gwAddr; /* Gate Way Address */
    IPMode ipMode; /* IP Address mode*/
    unsigned char macArray[LEN_MAC_ADDRESS]; /* MAC Address to be used*/
}LWIP_IF;

/**
 * @brief Starts the Lwip stack and initializes low level devices.
 * 
 * @param ipConf The desired addresses and/or modes.
 * @return True if everything went fine, false otherwise.
 */
bool startLwip(LWIP_IF* ipConf);

#ifdef __cplusplus
}
#endif

#endif // __LWIP_BBB_H__
