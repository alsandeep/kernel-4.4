/*
 * arch/arm/mach-owl/timer.c
 *
 * time0 use as clocksource
 * timer1 for time tick at boot stage
 *
 * Copyright 2012 Actions Semi Inc.
 * Author: Actions Semi, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/sched_clock.h>

#include <mach/hardware.h>
#include <mach/irqs.h>

/*
 * clocksource
 */
static cycle_t owl_read_timer(struct clocksource *cs)
{
	return (cycle_t)act_readl(T0_VAL);
}

static struct clocksource owl_clksrc = {
	.name		= "timer0",
	.rating		= 200,
	.read		= owl_read_timer,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

/*
 * Using this local implementation sched_clock which uses timer0
 * to get some better resolution when scheduling the kernel.
 */
static u64 notrace owl_read_sched_clock(void)
{
	return act_readl(T0_VAL);
}
/* Clockevent device: use one-shot mode */
static int owl_clkevt_oneshot(struct clock_event_device *dev)
{
	act_writel(0, T1_CTL);
	act_writel(0, T1_VAL);
	act_writel(0, T1_CMP);
return 0;
}

/* Clockevent device: use one-shot mode */
static int owl_clkevt_set_periodic(struct clock_event_device *dev)
{
	pr_err("%s: periodic mode not supported\n", __func__);
return 0;
}

/* Clockevent device: use one-shot mode */
static int owl_clkevt_shutdown(struct clock_event_device *dev)
{
	/* disable irq */
	act_writel(0, T1_CTL);
return 0;
}


/* Clockevent device: use one-shot mode */
static int owl_clkevt_resume(struct clock_event_device *dev)
{
return 0;
}

static int owl_clkevt_next(unsigned long evt, struct clock_event_device *ev)
{
	/* disable timer */
	act_writel(0x0, T1_CTL);

	/* writing the value has immediate effect */
	act_writel(0, T1_VAL);
	act_writel(evt, T1_CMP);

	/* enable timer & IRQ */
	act_writel(0x6, T1_CTL);

	return 0;
}

static struct clock_event_device owl_clkevt = {
	.name		= "timer1",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating	 = 200,
	.set_state_shutdown     = owl_clkevt_shutdown,
	.set_state_periodic     = owl_clkevt_set_periodic,
	.set_state_oneshot      = owl_clkevt_oneshot,
	.tick_resume            = owl_clkevt_resume,
	.set_next_event	= owl_clkevt_next,
};

/*
 * IRQ Handler for timer 1 of the MTU block.
 */
static irqreturn_t owl_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evdev = dev_id;

	act_writel(1 << 0, T1_CTL); /* Interrupt clear reg */
	evdev->event_handler(evdev);

	return IRQ_HANDLED;
}

static struct irqaction owl_timer_irq = {
	.name		= "timer1_tick",
	.flags		= IRQF_TIMER,
	.handler	= owl_timer_interrupt,
	.dev_id		= &owl_clkevt,
};

void __init owl_gp_timer_init(void)
{
	unsigned long rate;
	printk("Initialising... %s \n", __func__);

	/* enable the clock of timer */
	act_setl(1 << 27, CMU_DEVCLKEN1);

	rate = 24000000;

	/* Timer 0 is the free running clocksource */
	act_writel(0, T0_CTL);
	act_writel(0, T0_VAL);
	act_writel(0, T0_CMP);
	act_writel(4, T0_CTL);

	sched_clock_register(owl_read_sched_clock, 32, rate);
	clocksource_register_hz(&owl_clksrc, rate);

	/* Timer 1 is used for events, fix according to rate */
	act_writel(0, T1_CTL);
	act_writel(0, T1_VAL);
	act_writel(0, T1_CMP);

	setup_irq(OWL_IRQ_TIMER1, &owl_timer_irq);
	owl_clkevt.cpumask = cpumask_of(0);
	clockevents_config_and_register(&owl_clkevt, rate,
					0xf, 0xffffffff);
	printk("Completed Initialising... %s \n", __func__);
}

