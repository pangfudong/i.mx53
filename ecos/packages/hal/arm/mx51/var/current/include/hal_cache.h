#ifndef CYGONCE_HAL_CACHE_H
#define CYGONCE_HAL_CACHE_H

//=============================================================================
//
//      hal_cache.h
//
//      HAL cache control API
//
//=============================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Red Hat, Inc.
//
// eCos is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free
// Software Foundation; either version 2 or (at your option) any later version.
//
// eCos is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
// for more details.
//
// You should have received a copy of the GNU General Public License along
// with eCos; if not, write to the Free Software Foundation, Inc.,
// 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
//
// As a special exception, if other files instantiate templates or use macros
// or inline functions from this file, or you compile this file and link it
// with other works to produce a work based on this file, this file does not
// by itself cause the resulting work to be covered by the GNU General Public
// License. However the source code for this file must still be made available
// in accordance with section (3) of the GNU General Public License.
//
// This exception does not invalidate any other reasons why a work based on
// this file might be covered by the GNU General Public License.
//
// Alternative licenses for eCos may be arranged by contacting Red Hat, Inc.
// at http://sources.redhat.com/ecos/ecos-license/
// -------------------------------------------
//####ECOSGPLCOPYRIGHTEND####
//=============================================================================

#include <cyg/infra/cyg_type.h>
#include <cyg/hal/hal_soc.h>         // Variant specific hardware definitions

//-----------------------------------------------------------------------------
// Cache dimensions

// Data cache
#define HAL_DCACHE_SIZE                 0x4000    // 16KB Size of data cache in bytes
#define HAL_DCACHE_LINE_SIZE            32    // Size of a data cache line
#define HAL_DCACHE_WAYS                 64    // Associativity of the cache

// Instruction cache
#define HAL_ICACHE_SIZE                 0x4000    // Size of cache in bytes
#define HAL_ICACHE_LINE_SIZE            32    // Size of a cache line
#define HAL_ICACHE_WAYS                 64    // Associativity of the cache

#define HAL_DCACHE_SETS (HAL_DCACHE_SIZE / (HAL_DCACHE_LINE_SIZE*HAL_DCACHE_WAYS))
#define HAL_ICACHE_SETS (HAL_ICACHE_SIZE / (HAL_ICACHE_LINE_SIZE*HAL_ICACHE_WAYS))

#define CYGHWR_HAL_ARM_ARM9_CLEAN_DCACHE_INDEX
#define CYGHWR_HAL_ARM_ARM9_CLEAN_DCACHE_INDEX_STEP  0x20
#define CYGHWR_HAL_ARM_ARM9_CLEAN_DCACHE_INDEX_LIMIT 0x100
//-----------------------------------------------------------------------------
// Global control of data cache

// Enable the data cache
#define HAL_DCACHE_ENABLE_L1()                                          \
CYG_MACRO_START                                                         \
    asm volatile (                                                      \
        "mrc p15, 0, r1, c1, c0, 0;"                                    \
        "orr r1, r1, #0x0007;" /* enable DCache (also ensures */        \
                               /* the MMU, alignment faults, and */       \
        "mcr p15, 0, r1, c1, c0, 0"                                     \
        :                                                               \
        :                                                               \
        : "r1" /* Clobber list */                                       \
        );                                                              \
CYG_MACRO_END

// Disable the data cache
#define HAL_DCACHE_DISABLE_L1()                                         \
CYG_MACRO_START                                                         \
    asm volatile (                                                      \
        "mov r1, #0;"                                                   \
        "mcr p15, 0, r1, c7, c6, 0;" /* clear data cache */             \
        "mrc p15, 0, r1, c1, c0, 0;"                                    \
        "bic r1, r1, #0x0004;" /* disable DCache  */                    \
                             /* but not MMU and alignment faults */     \
        "mcr p15, 0, r1, c1, c0, 0"                                     \
        :                                                               \
        :                                                               \
        : "r1" /* Clobber list */                                       \
    );                                                                  \
CYG_MACRO_END

// Invalidate the entire cache
#define HAL_DCACHE_INVALIDATE_ALL_L1()                                  \
CYG_MACRO_START  /* this macro can discard dirty cache lines. */        \
    asm volatile (                                                      \
        "mov r0, #0;"                                                   \
        "mcr p15, 0, r0, c7, c6, 0;" /* flush d-cache */                \
        "mcr p15, 0, r0, c8, c7, 0;" /* flush i+d-TLBs */               \
        :                                                               \
        :                                                               \
        : "r0","memory" /* clobber list */                              \
    );                                                                  \
CYG_MACRO_END

// Synchronize the contents of the cache with memory.
// using ARM9's defined(CYGHWR_HAL_ARM_ARM9_CLEAN_DCACHE_INDEX)
#define HAL_DCACHE_SYNC_L1()                                           \
CYG_MACRO_START                                                        \
    asm volatile (                                                     \
        "nop; "                                                        \
        "nop; "                                                        \
        "nop; "                                                        \
        "nop; "                                                        \
        "nop; "                                                        \
        "nop; "                                                        \
        "mov r0, #0x0;"                                                \
        "mcr p15, 0, r0, c7, c14, 0;" /* clean, invalidate Dcache*/    \
        "mcr p15, 0, r0, c7, c10, 4;" /* drain the write buffer */     \
        "mcr p15, 0, r0, c7, c10, 5;" /* data memory barrier */        \
        :                                                              \
        :                                                              \
        : "r0" /* Clobber list */                                      \
        );                                                             \
CYG_MACRO_END

// Query the state of the data cache
#define HAL_DCACHE_IS_ENABLED(_state_)                                  \
CYG_MACRO_START                                                         \
    register int reg;                                                   \
    asm volatile (                                                      \
        "nop; "                                                         \
        "nop; "                                                         \
        "nop; "                                                         \
        "nop; "                                                         \
        "nop; "                                                         \
        "mrc p15, 0, %0, c1, c0, 0;"                                    \
                  : "=r"(reg)                                           \
                  :                                                     \
        );                                                              \
    (_state_) = (0 != (4 & reg)); /* Bit 2 is DCache enable */          \
CYG_MACRO_END

//-----------------------------------------------------------------------------
// Global control of Instruction cache

// Enable the instruction cache
#define HAL_ICACHE_ENABLE_L1()                                          \
CYG_MACRO_START                                                         \
    asm volatile (                                                      \
        "mrc p15, 0, r1, c1, c0, 0;"                                    \
        "orr r1, r1, #0x1000;"                                          \
        "orr r1, r1, #0x0003;"  /* enable ICache (also ensures   */     \
                                /* that MMU and alignment faults */     \
                                /* are enabled)                  */     \
        "mcr p15, 0, r1, c1, c0, 0"                                     \
        :                                                               \
        :                                                               \
        : "r1" /* Clobber list */                                       \
        );                                                              \
CYG_MACRO_END

// Query the state of the instruction cache
#define HAL_ICACHE_IS_ENABLED(_state_)                                  \
CYG_MACRO_START                                                         \
    register cyg_uint32 reg;                                            \
    asm volatile (                                                      \
        "mrc p15, 0, %0, c1, c0, 0"                                     \
        : "=r"(reg)                                                     \
        :                                                               \
        );                                                              \
                                                                        \
    (_state_) = (0 != (0x1000 & reg)); /* Bit 12 is ICache enable */    \
CYG_MACRO_END

// Disable the instruction cache
#define HAL_ICACHE_DISABLE_L1()                                         \
CYG_MACRO_START                                                         \
    asm volatile (                                                      \
        "mrc p15, 0, r1, c1, c0, 0;"                                    \
        "bic r1, r1, #0x1000;" /* disable ICache (but not MMU, etc) */  \
        "mcr p15, 0, r1, c1, c0, 0;"                                    \
        "mov r1, #0;"                                                   \
        "mcr p15, 0, r1, c7, c5, 0;"  /* flush ICache */                \
        "mcr p15, 0, r1, c7, c5, 4;"  /* flush prefetch buffer */       \
        "nop;" /* next few instructions may be via cache    */          \
        "nop;"                                                          \
        "nop;"                                                          \
        "nop;"                                                          \
        "nop;"                                                          \
        "nop"                                                           \
        :                                                               \
        :                                                               \
        : "r1" /* Clobber list */                                       \
        );                                                              \
CYG_MACRO_END

// Invalidate the entire cache
#define HAL_ICACHE_INVALIDATE_ALL_L1()                                  \
CYG_MACRO_START                                                         \
    /* this macro can discard dirty cache lines (N/A for ICache) */     \
    asm volatile (                                                      \
        "mov r1, #0;"                                                   \
        "mcr p15, 0, r1, c7, c5, 0;"  /* flush ICache */                \
        "mcr p15, 0, r1, c8, c5, 0;"  /* flush ITLB only */             \
        "mcr p15, 0, r1, c7, c5, 4;"  /* flush prefetch buffer */       \
        "nop;" /* next few instructions may be via cache    */          \
        "nop;"                                                          \
        "nop;"                                                          \
        "nop;"                                                          \
        "nop;"                                                          \
        "nop;"                                                          \
        :                                                               \
        :                                                               \
        : "r1" /* Clobber list */                                       \
        );                                                              \
CYG_MACRO_END

// Synchronize the contents of the cache with memory.
// (which includes flushing out pending writes)
#define HAL_ICACHE_SYNC()                                       \
CYG_MACRO_START                                                 \
    HAL_DCACHE_SYNC(); /* ensure data gets to RAM */            \
    HAL_ICACHE_INVALIDATE_ALL(); /* forget all we know */       \
CYG_MACRO_END

// Query the state of the L2 cache
#define HAL_L2CACHE_IS_ENABLED(_state_)                         \
    (_state_ = readl(L2CC_BASE_ADDR + L2_CACHE_CTL_REG) & 1)

#ifdef L2CC_ENABLED

#define HAL_ENABLE_L2()                             \
{                                                   \
    writel(1, L2CC_BASE_ADDR + L2_CACHE_CTL_REG);   \
}

#define HAL_DISABLE_L2()                            \
{                                                   \
    writel(0, L2CC_BASE_ADDR + L2_CACHE_CTL_REG);   \
}

#define HAL_SYNC_L2()                                                           \
{                                                                               \
    if ((readl(L2CC_BASE_ADDR + L2_CACHE_CTL_REG) & 1) != 0) {                  \
        writel(0, L2CC_BASE_ADDR + L2_CACHE_SYNC_REG);                          \
        while ((readl(L2CC_BASE_ADDR + L2_CACHE_SYNC_REG) & 1) == 1);           \
    }                                                                           \
}

#define HAL_INVALIDATE_L2()                                                     \
{                                                                               \
    if ((readl(L2CC_BASE_ADDR + L2_CACHE_CTL_REG) & 1) != 0) {                  \
        writel(0xFF, L2CC_BASE_ADDR + L2_CACHE_INV_WAY_REG);                    \
        while ((readl(L2CC_BASE_ADDR + L2_CACHE_INV_WAY_REG) & 0xFF) != 0);    \
        HAL_SYNC_L2();                                                          \
    }                                                                           \
}
                                                                                \
#define HAL_CLEAN_INVALIDATE_L2()                                               \
{                                                                               \
    if ((readl(L2CC_BASE_ADDR + L2_CACHE_CTL_REG) & 1) != 0) {                  \
        writel(0xFF, L2CC_BASE_ADDR + L2_CACHE_CLEAN_INV_WAY_REG);              \
        while ((readl(L2CC_BASE_ADDR + L2_CACHE_CLEAN_INV_WAY_REG) & 0xFF) != 0);\
        HAL_SYNC_L2();                                                          \
    }                                                                           \
}

#else //L2CC_ENABLED

#define HAL_ENABLE_L2()
#define HAL_DISABLE_L2()
#define HAL_INVALIDATE_L2()
#define HAL_CLEAN_INVALIDATE_L2()
#define HAL_SYNC_L2()
#endif //L2CC_ENABLED

/*********************** Exported macros *******************/

#define HAL_DCACHE_ENABLE() {           \
        HAL_DCACHE_ENABLE_L1();         \
        HAL_ENABLE_L2();                \
}

#define HAL_DCACHE_DISABLE() {          \
        HAL_DCACHE_DISABLE_L1();        \
        HAL_DISABLE_L2();               \
}

#define HAL_DCACHE_INVALIDATE_ALL() {   \
        HAL_DCACHE_INVALIDATE_ALL_L1(); \
        HAL_CLEAN_INVALIDATE_L2();      \
}

#define HAL_DCACHE_SYNC() {             \
        HAL_DCACHE_SYNC_L1();           \
        /* don't just call HAL_SYNC_L2() */ \
        HAL_CLEAN_INVALIDATE_L2();      \
}

#define HAL_ICACHE_INVALIDATE_ALL() {   \
        HAL_ICACHE_INVALIDATE_ALL_L1(); \
        HAL_CLEAN_INVALIDATE_L2();      \
}

#define HAL_ICACHE_DISABLE() {          \
        HAL_ICACHE_DISABLE_L1();        \
}                                       

#define HAL_ICACHE_ENABLE() {           \
        HAL_ICACHE_ENABLE_L1();         \
}

#endif // ifndef CYGONCE_HAL_CACHE_H
// End of hal_cache.h
