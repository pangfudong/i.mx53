//==========================================================================
//
//      sprintf.c
//
//      Light-weight BSP sprintf.
//
//==========================================================================
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
//==========================================================================
//#####DESCRIPTIONBEGIN####
//
// Author(s):    
// Contributors: gthomas
// Date:         1999-10-20
// Purpose:      Light-weight BSP sprintf.
// Description:  
//               
//
//####DESCRIPTIONEND####
//
//=========================================================================


#include <bsp/bsp.h>
#include "bsp_if.h"

static char *str_ptr;

static void
str_putc(char ch)
{
    *str_ptr++ = ch;
}

int
bsp_vsprintf(char *str, const char *fmt, va_list ap)
{
    str_ptr = str;
    __vprintf(str_putc, fmt, ap);
    *str_ptr = '\0';
    return str_ptr - str;
}


void
bsp_sprintf(char *str, const char *fmt, ...)
{
    va_list ap;

    va_start (ap, fmt);
    bsp_vsprintf(str, fmt, ap);
    va_end (ap);
}


