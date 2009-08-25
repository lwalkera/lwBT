#pragma once

#include "lwip/opt.h"

#ifdef htons
#undef htons
#endif /* htons */
#ifdef htonl
#undef htonl
#endif /* htonl */
#ifdef ntohs
#undef ntohs
#endif /* ntohs */
#ifdef ntohl
#undef ntohl
#endif /* ntohl */

#ifndef LWIP_PLATFORM_BYTESWAP
#define LWIP_PLATFORM_BYTESWAP 0
#endif

#if BYTE_ORDER == BIG_ENDIAN
#define htons(x) (x)
#define ntohs(x) (x)
#define htonl(x) (x)
#define ntohl(x) (x)
#else /* BYTE_ORDER != BIG_ENDIAN */
#ifdef LWIP_PREFIX_BYTEORDER_FUNCS
/* workaround for naming collisions on some platforms */
#define htons lwip_htons
#define ntohs lwip_ntohs
#define htonl lwip_htonl
#define ntohl lwip_ntohl
#endif /* LWIP_PREFIX_BYTEORDER_FUNCS */
#if LWIP_PLATFORM_BYTESWAP
#define htons(x) LWIP_PLATFORM_HTONS(x)
#define ntohs(x) LWIP_PLATFORM_HTONS(x)
#define htonl(x) LWIP_PLATFORM_HTONL(x)
#define ntohl(x) LWIP_PLATFORM_HTONL(x)
#else /* LWIP_PLATFORM_BYTESWAP */
u16_t htons(u16_t x);
u16_t ntohs(u16_t x);
u32_t htonl(u32_t x);
u32_t ntohl(u32_t x);
#endif /* LWIP_PLATFORM_BYTESWAP */

#endif /* BYTE_ORDER == BIG_ENDIAN */
