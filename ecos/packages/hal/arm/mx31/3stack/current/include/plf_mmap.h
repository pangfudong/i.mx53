#ifndef CYGONCE_HAL_BOARD_PLATFORM_PLF_MMAP_H
#define CYGONCE_HAL_BOARD_PLATFORM_PLF_MMAP_H
//=============================================================================
//
//      plf_mmap.h
//
//      Platform specific memory map support
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
//===========================================================================

#include <cyg/hal/hal_misc.h>

// Get the pagesize for a particular virtual address:

// This does not depend on the vaddr.
#define HAL_MM_PAGESIZE(vaddr, pagesize) CYG_MACRO_START        \
        (pagesize) = SZ_1M;                                         \
CYG_MACRO_END

// Get the physical address from a virtual address:

#define HAL_VIRT_TO_PHYS_ADDRESS( vaddr, paddr ) CYG_MACRO_START           \
        cyg_uint32 _v_ = (cyg_uint32)(vaddr);                                  \
        if ( _v_ < 128 * SZ_1M )          /* SDRAM */                           \
                _v_ += 0x800u * SZ_1M;                                             \
        else                             /* Rest of it */                      \
                /* no change */ ;                                                  \
                (paddr) = _v_;                                                         \
CYG_MACRO_END

//---------------------------------------------------------------------------
#endif // CYGONCE_HAL_BOARD_PLATFORM_PLF_MMAP_H
