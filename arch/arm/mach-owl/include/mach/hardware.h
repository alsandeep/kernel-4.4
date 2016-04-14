/*
 * Copyright (c) 2014 Actions Semiconductor Co., Ltd.
 *
 * Author: Yixun Lan <yixun.lan@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __ASM_ARCH_HARDWARE_H
#define __ASM_ARCH_HARDWARE_H

#include <linux/types.h>

#define OWL_SDRAM_BASE			UL(0x00000000)
#define OWL_IO_DEVICE_BASE		UL(0xB0000000)

#define OWL_IO_ADDR_BASE		0xF8000000

/* macro to get at IO space when running virtually */
#ifdef CONFIG_MMU
#define IO_ADDRESS(x)	(OWL_IO_ADDR_BASE + ((x) & 0x03ffffff))
#else
#define IO_ADDRESS(x)	(x)
#endif

#define __io_address(n)	 __io(IO_ADDRESS(n))

/*
 * Peripheral physical addresses
 */
#define OWL_PA_REG_BASE		(0xB0000000)
#define OWL_PA_REG_SIZE		(6 * SZ_1M)

#define OWL_PA_CORESIGHT	(0xB0000000)
#define OWL_PA_EXTDEBUG		(0xB0008000)

#define OWL_PA_COREPERI		(0xB0020000)
#define OWL_PA_SCU		(0xB0020000)
#define OWL_PA_GIC_CPU		(0xB0020100)
#define OWL_PA_GIC_GP		(0xB0020200)
#define OWL_PA_TWD		(0xB0020600)
#define OWL_PA_GIC_DIST		(0xB0021000)
#define OWL_IO_COREPERI_SIZE	(SZ_8K)

#define OWL_PA_L2CC		(0xB0022000)
#define OWL_IO_L2CC_SIZE	(SZ_4K)

#define OWL_PA_BOOT_RAM		(0xFFFF8000)
#define OWL_VA_BOOT_RAM		(0xFFFF8000)

#endif /* __ASM_ARCH_HARDWARE_H */
