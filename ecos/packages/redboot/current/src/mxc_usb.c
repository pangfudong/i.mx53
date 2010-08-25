//==========================================================================
//
//      mxc_usb.c
//
//      usb download support for RedBoot
//
//==========================================================================
//####ECOSGPLCOPYRIGHTBEGIN####
// -------------------------------------------
// This file is part of eCos, the Embedded Configurable Operating System.
// Copyright (C) 1998, 1999, 2000, 2001, 2002 Red Hat, Inc.
// Copyright (C) 2002, 2003 Gary Thomas
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
// Author(s):    Kevin Zhang
// Contributors: Kevin Zhang
// Date:         2006-09-06
// Purpose:      
// Description:  this code is based on tftp_client.c
//              
// This code is part of RedBoot (tm).
//
//####DESCRIPTIONEND####
//
//==========================================================================

#include <redboot.h>

static struct {
    bool open;
    int  total_timeouts, packets_received;
    unsigned long last_good_block;
    int  avail, actual_len;
//    char data[SEGSIZE+sizeof(struct mxcusbhdr)];
//    char *bufp;
} mxcusb_stream;

extern void mxc_pcd_open(void);
extern void mxc_pcd_exit(void);
extern int mxc_pcd_int_hndlr(long buffer, long length);

int
mxcusb_stream_open(connection_info_t *info,
                 int *err)
{
//    diag_printf("%s()\n", __FUNCTION__);
    
    mxc_pcd_open();
    return 0;
}

void
mxcusb_stream_close(int *err)
{
//    diag_printf("%s()\n", __FUNCTION__);

    mxc_pcd_exit();
}

void
mxcusb_stream_terminate(bool abort,
                      int (*getc)(void))
{
    int err;

//    diag_printf("%s()\n", __FUNCTION__);
}

int
mxcusb_stream_read(char *buf,
                 int len,
                 int *err)
{
//    diag_printf("%s(len=%d buf=%x)\n", __FUNCTION__, len, buf);

	return mxc_pcd_int_hndlr(buf, len);
}

char *
mxcusb_error(int err)
{
    char *errmsg = "Unknown error";

    diag_printf("%s()\n", __FUNCTION__);
#if 0
    switch (err) {
    case MXCUSB_ENOTFOUND:
        return "file not found";
    case MXCUSB_EACCESS:
        return "access violation";
    case MXCUSB_ENOSPACE:
        return "disk full or allocation exceeded";
    case MXCUSB_EBADOP:
        return "illegal MXCUSB operation";
    case MXCUSB_EBADID:
        return "unknown transfer ID";
    case MXCUSB_EEXISTS:
        return "file already exists";
    case MXCUSB_ENOUSER:
        return "no such user";
    case MXCUSB_TIMEOUT:
        return "operation timed out";
    case MXCUSB_INVALID:
        return "invalid parameter";
    case MXCUSB_TOOLARGE:
        return "file is larger than buffer";
    }
#endif
    return errmsg;
}

//
// RedBoot interface
//
GETC_IO_FUNCS(mxcusb_io, mxcusb_stream_open, mxcusb_stream_close,
              mxcusb_stream_terminate, mxcusb_stream_read, mxcusb_error);
RedBoot_load(usb, mxcusb_io, true, false, 0);

