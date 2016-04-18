/*
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/bootmem.h>
#include <linux/memblock.h>

#include <mach/hardware.h>
#if defined(CONFIG_OF)
#include <linux/of_fdt.h>
#endif


static phys_addr_t s_phy_mem_size_saved;
phys_addr_t owl_get_phy_mem_size(void)
{
	return s_phy_mem_size_saved;
}
EXPORT_SYMBOL(owl_get_phy_mem_size);
extern phys_addr_t arm_lowmem_limit;
void __init owl_reserve(void)
{
	phys_addr_t phy_mem_size, phy_mem_end;
	unsigned int owl_ion0_start = 0;
	unsigned int owl_ion1_start = 0;

	printk( KERN_INFO "Initialising... %s \n", __func__);
	phy_mem_size = memblock_phys_mem_size();
	if (phy_mem_size & (phy_mem_size - 1)) { /* != 2^n ? */
		uint _tmp = __fls(phy_mem_size);
		if (_tmp > 0 && (phy_mem_size & (1U << (_tmp - 1)))) {
			/* close to next boundary */
			_tmp++;
			phy_mem_size =
				(_tmp >= sizeof(phy_mem_size) * 8) ? phy_mem_size : (1U << _tmp);
		} else {
			phy_mem_size = 1U << _tmp;
		}
	}
	s_phy_mem_size_saved = phy_mem_size;
	phy_mem_end = arm_lowmem_limit;
	printk( KERN_INFO  "%s: pyhsical memory size %u bytes, end @0x%x\n",
		__func__, phy_mem_size, phy_mem_end);

	memblock_reserve(0, 0x4000); /* reserve low 16K for DDR dqs training */

	printk("completed Initialising... %s \n", __func__);
}

