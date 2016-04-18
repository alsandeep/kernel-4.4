/*
 * arch/arm/mach-owl/cpu-owl.c
 *
 * cpu peripheral init for Actions SOC
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <asm/mach/map.h>
#include <linux/irqchip/arm-gic.h>
#include <mach/hardware.h>

#define L2X0_AUX_CTRL_MASK                   0xc0000fff
#define IRQ_LOCALTIMER                          (29)

extern void __init owl_gp_timer_init(void);

static struct map_desc owl_io_desc[] __initdata = {
	{
		.virtual	= IO_ADDRESS(OWL_PA_REG_BASE),
		.pfn		= __phys_to_pfn(OWL_PA_REG_BASE),
		.length		= OWL_PA_REG_SIZE,
		.type		= MT_DEVICE,
	},

   /*for suspend , to load ddr ops code. by jlingzhang*/
	{
		.virtual	= OWL_VA_BOOT_RAM,
		.pfn		= __phys_to_pfn(OWL_PA_BOOT_RAM),
		.length		= SZ_4K,
		.type		= MT_MEMORY_RW,
	},
};

void __init owl_map_io(void)
{
	printk( KERN_INFO "Initialising... %s \n", __func__);
	/* dma coherent allocate buffer: 2 ~ 14MB */
	init_dma_coherent_pool_size(14 << 20);
	iotable_init(owl_io_desc, ARRAY_SIZE(owl_io_desc));
	printk( KERN_INFO "completed Initialising... %s \n", __func__);
}

#ifdef CONFIG_OF
extern int gic_of_init(struct device_node *node, struct device_node *parent);
static const struct of_device_id owl_dt_irq_match[] = {
	{ .compatible = "arm,cortex-a9-gic", .data = gic_of_init },
	{ }
};
#endif

extern void of_irq_init(const struct of_device_id *matches);

static struct device_node *intc_node;

void __init owl_init_irq(void)
{
#ifdef CONFIG_OF
	printk( KERN_INFO "Initialising... of_irq_init in %s \n", __func__);
	of_irq_init(owl_dt_irq_match);
#else
	printk( KERN_INFO "Initialising...  gic_init %s \n", __func__);
	gic_init(0, 29, IO_ADDRESS(OWL_PA_GIC_DIST),
		IO_ADDRESS(OWL_PA_GIC_CPU));
#endif
}

void __init owl_timer_init(void)
{
	printk( KERN_INFO "Initialising... %s \n", __func__);
	owl_gp_timer_init();

	printk( KERN_INFO "completed Initialising... %s \n", __func__);
}

