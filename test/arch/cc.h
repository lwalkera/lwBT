#pragma once

#include <stdint.h>
#include <stdio.h>

//system types
typedef uint8_t		u8_t;
typedef int8_t		s8_t;
typedef uint16_t	u16_t;
typedef int16_t		s16_t;
typedef uint32_t	u32_t;
typedef int32_t		s32_t;
typedef int			mem_ptr_t;

#define PACK_STRUCT_STRUCT attribute((__packed__))

#define LWIP_PLATFORM_DIAG(x) printf(x)
#define LWIP_PLATFORM_ASSERT(x) printf(x)
