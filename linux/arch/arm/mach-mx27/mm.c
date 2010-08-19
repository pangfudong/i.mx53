/*
 * Copyright 2004-2007 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/mm.h>
#include <linux/init.h>
#include <asm/hardware.h>
#include <asm/pgtable.h>
#include <asm/mach/map.h>

/*!
 * @file mach-mx27/mm.c
 *
 * @brief This file creates static mapping between physical to virtual memory.
 *
 * @ingroup Memory_MX27
 */

/*!
 * This structure defines the MX27 memory map.
 */
static struct map_desc mxc_io_desc[] __initdata = {
	{
	 .virtual = AIPI_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(AIPI_BASE_ADDR),
	 .length = AIPI_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = SAHB1_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(SAHB1_BASE_ADDR),
	 .length = SAHB1_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = X_MEMC_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(X_MEMC_BASE_ADDR),
	 .length = X_MEMC_SIZE,
	 .type = MT_DEVICE},
	{
	 .virtual = CS4_BASE_ADDR_VIRT,
	 .pfn = __phys_to_pfn(CS4_BASE_ADDR),
	 .length = CS4_SIZE,
	 .type = MT_DEVICE}
};

/*!
 * This function initializes the memory map. It is called during the
 * system startup to create static physical to virtual memory map for
 * the IO modules.
 */
void __init mxc_map_io(void)
{
	iotable_init(mxc_io_desc, ARRAY_SIZE(mxc_io_desc));
}
