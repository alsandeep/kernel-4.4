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


#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>

#define OWL_MAX_UART		6

#define UART_TO_OWL(uart_port) ((struct owl_port *) uart_port)

#define CMU_BASE		0xB0160000
#define CMU_DEVCLKEN1		(CMU_BASE + 0x00A4)
#define CMU_DEVRST1		(CMU_BASE + 0x00AC)

#define UART_CTL		0x0000
#define UART_RXDAT		0x0004
#define UART_TXDAT		0x0008
#define UART_STAT		0x000C

/* UART0_CTL, bit[23:31], bit[13] bit[7:11] bit[3] reserved */
#define UART_CTL_DTCR		(0x1 << 22)	/* DMA TX counter reset */
#define UART_CTL_DRCR		(0x1 << 21)	/* DMA RX counter reset */
#define UART_CTL_LBEN		(0x1 << 20)	/* Loop Back Enable */
#define UART_CTL_TXIE		(0x1 << 19)	/* TX IRQ Enable */
#define UART_CTL_RXIE		(0x1 << 18)	/* RX IRQ Enable */
#define UART_CTL_TXDE		(0x1 << 17)	/* TX DRQ Enable */
#define UART_CTL_RXDE		(0x1 << 16)	/* RX DRQ Enable */
#define UART_CTL_EN		(0x1 << 15)	/* UART Enable */
#define UART_CTL_TRFS		(0x1 << 14)	/* UART TX/RX FIFO Enable */
#define	UART_CTL_TRFS_RX	(0x0 << 14)	/* select RX FIFO */
#define	UART_CTL_TRFS_TX	(0x1 << 14)	/* select TX FIFO */
#define UART_CTL_AFE		(0x1 << 12)	/* Autoflow Enable */
#define UART_CTL_PRS_MASK	(0x7 << 4)
#define UART_CTL_PRS(x)		(((x) & 0x7) << 4)
#define	UART_CTL_PRS_NONE	UART_CTL_PRS(0)
#define	UART_CTL_PRS_ODD	UART_CTL_PRS(4)
#define	UART_CTL_PRS_MARK	UART_CTL_PRS(5)
#define	UART_CTL_PRS_EVEN	UART_CTL_PRS(6)
#define	UART_CTL_PRS_SPACE	UART_CTL_PRS(7)
#define UART_CTL_STPS_MASK	(0x1 << 2)
#define	UART_CTL_STPS_1BITS	(0x0 << 2)
#define	UART_CTL_STPS_2BITS	(0x1 << 2)
#define UART_CTL_DWLS_MASK	(0x3 << 0)
#define UART_CTL_DWLS(x)	(((x) & 0x3) << 0)
#define	UART_CTL_DWLS_5BITS	UART_CTL_DWLS(0)
#define	UART_CTL_DWLS_6BITS	UART_CTL_DWLS(1)
#define	UART_CTL_DWLS_7BITS	UART_CTL_DWLS(2)
#define	UART_CTL_DWLS_8BITS	UART_CTL_DWLS(3)

/* UART0_RXDAT, 32 Bytes fifo depth, bit[8:31] reserved */
#define UART_RXDAT_MASK		(0xFF << 0)	  /* Received Data */

/* UART0_TXDAT, 16 Bytes fifo depth, bit[8:31] reserved */
#define UART_TXDAT_MASK		(0xFF << 0)	  /* Sending Data*/

/* UART_STAT, bit[17:31] reserved */
#define UART_STAT_UTBB		(0x1 << 16)	/* UART0 TX busy bit */
#define UART_STAT_TRFL_MASK	GENMASK(16, 11)	/* TX/RX FIFO Level */
#define UART_STAT_TRFL_SET(x)	(((x) << 11) & UART_STAT_TRFL_MASK)
#define UART_STAT_TFES		(0x1 << 10)	/* TX FIFO Empty Status */
#define UART_STAT_RFFS		(0x1 << 9)	/* RX FIFO full Status */
#define UART_STAT_RTSS		(0x1 << 8)	/* RTS status */
#define UART_STAT_CTSS		(0x1 << 7)	/* CTS status */
#define UART_STAT_TFFU		(0x1 << 6)	/* TX FIFO full Status */
#define UART_STAT_RFEM		(0x1 << 5)	/* RX FIFO Empty Status */
#define UART_STAT_RXST		(0x1 << 4)	/* Receive Status */
#define UART_STAT_TFER		(0x1 << 3)	/* TX FIFO Erro */
#define UART_STAT_RXER		(0x1 << 2)	/* RX FIFO Erro */
#define UART_STAT_TIP		(0x1 << 1)	/* TX IRQ Pending Bit */
#define UART_STAT_RIP		(0x1 << 0)	/* RX IRQ Pending Bit */

struct owl_port {
	struct uart_port uart;
	char name[16];
};

static struct owl_port owl_ports[OWL_MAX_UART];

static inline struct uart_port *get_port_from_line(unsigned int line)
{
	return &owl_ports[line].uart;
}

static inline
void owl_uart_writel(struct uart_port *port, unsigned int val, unsigned int off)
{
	writel(val, port->membase + off);
}

static inline
unsigned int owl_uart_readl(struct uart_port *port, unsigned int off)
{
	return readl(port->membase + off);
}

static void owl_start_tx(struct uart_port *port)
{
	unsigned int data;

	data = owl_uart_readl(port, UART_STAT);
	data |= UART_STAT_TIP;
	owl_uart_writel(port, data, UART_STAT);

	data = owl_uart_readl(port, UART_CTL);
	data |= UART_CTL_TXIE;
	owl_uart_writel(port, data, UART_CTL);
}

static void owl_stop_tx(struct uart_port *port)
{
	unsigned int data;

	data = owl_uart_readl(port, UART_CTL);
	data &= ~UART_CTL_TXIE;
	owl_uart_writel(port, data, UART_CTL);

	data = owl_uart_readl(port, UART_STAT);
	data |= UART_STAT_TIP;
	owl_uart_writel(port, data, UART_STAT);
}

static void owl_stop_rx(struct uart_port *port)
{
	unsigned int data;

	data = owl_uart_readl(port, UART_CTL);
	data &= ~UART_CTL_RXIE;
	owl_uart_writel(port, data, UART_CTL);

	data = owl_uart_readl(port, UART_STAT);
	data |= UART_STAT_RIP;
	owl_uart_writel(port, data, UART_STAT);
}

static void owl_enable_ms(struct uart_port *port)
{
}

/*
 * Characters received (called from interrupt handler)
 */
static void handle_rx(struct uart_port *port)
{
	unsigned int stat, data;

	/* select RX FIFO */
	data = owl_uart_readl(port, UART_CTL);
	data &= ~UART_CTL_TRFS;
	data |= UART_CTL_TRFS_RX;
	owl_uart_writel(port, data, UART_CTL);

	/* and now the main RX loop */
	while (!((stat = owl_uart_readl(port, UART_STAT)) & UART_STAT_RFEM)) {
		unsigned int c;
		char flag = TTY_NORMAL;

		c = owl_uart_readl(port, UART_RXDAT);

		if (stat & UART_STAT_RXER)
			port->icount.overrun++;

		if (stat & UART_STAT_RXST) {
			/* we are not able to distinguish the error type */
			port->icount.brk++;
			port->icount.frame++;

			/* Mask conditions we're ignorning. */
			stat &= port->read_status_mask;
			if (stat & UART_STAT_RXST)
				flag = TTY_PARITY;
		} else {
			port->icount.rx++;
		}

		if (uart_handle_sysrq_char(port, c))
			continue;

		uart_insert_char(port, stat, stat & UART_STAT_RXER, c, flag);
	}

	tty_flip_buffer_push(&port->state->port);
}

/*
 * transmit interrupt handler
 */
static void handle_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int data;

	if (port->x_char) {
		/* wait TX FIFO not full */
		while (owl_uart_readl(port, UART_STAT) & UART_STAT_TFFU)
			;
		owl_uart_writel(port, port->x_char, UART_TXDAT);
		port->icount.tx++;
		port->x_char = 0;
	}

	/* select TX FIFO */
	data = owl_uart_readl(port, UART_CTL);
	data &= ~UART_CTL_TRFS;
	data |= UART_CTL_TRFS_TX;
	owl_uart_writel(port, data, UART_CTL);

	while (!(owl_uart_readl(port, UART_STAT) & UART_STAT_TFFU)) {
		if (uart_circ_empty(xmit))
			break;

		owl_uart_writel(port, xmit->buf[xmit->tail], UART_TXDAT);

		/* wait FIFO empty? */
		while (owl_uart_readl(port, UART_STAT) & UART_STAT_TRFL_MASK)
			;

		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	if (uart_circ_empty(xmit))
		owl_stop_tx(port);
}

/*
 * Interrupt handler
 */
static irqreturn_t owl_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	unsigned int stat;

	spin_lock(&port->lock);
	stat = owl_uart_readl(port, UART_STAT);

	if (stat & UART_STAT_RIP)
		handle_rx(port);

	if (stat & UART_STAT_TIP)
		handle_tx(port);

	stat = owl_uart_readl(port, UART_STAT);
	stat |= UART_STAT_RIP | UART_STAT_TIP;
	owl_uart_writel(port, stat, UART_STAT);

	spin_unlock(&port->lock);

	return IRQ_HANDLED;
}

/*
 * Return TIOCSER_TEMT when transmitter FIFO and Shift register is empty.
 */
static unsigned int owl_tx_empty(struct uart_port *port)
{
	unsigned int data, ret;
	unsigned long flags = 0;

	spin_lock_irqsave(&port->lock, flags);

	/* select TX FIFO */
	data = owl_uart_readl(port, UART_CTL);
	data &= ~UART_CTL_TRFS;
	data |= UART_CTL_TRFS_TX;
	owl_uart_writel(port, data, UART_CTL);

	/* check FIFO level */
	data = owl_uart_readl(port, UART_STAT);
	ret = (data & UART_STAT_TRFL_MASK) ? 0 : TIOCSER_TEMT;

	spin_unlock_irqrestore(&port->lock, flags);

	return ret;
}

static void owl_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int owl_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void owl_break_ctl(struct uart_port *port, int break_ctl)
{
}

static int owl_set_baud_rate(struct uart_port *port, unsigned int baud)
{
	return baud;
}

/*
 * Perform initialization and enable port for reception
 */
static int owl_startup(struct uart_port *port)
{
	struct owl_port *owl_port = UART_TO_OWL(port);
	unsigned int data;
	int ret;

	snprintf(owl_port->name, sizeof(owl_port->name),
		 "owl_serial%d", port->line);

	ret = request_irq(port->irq, owl_irq, IRQF_TRIGGER_HIGH,
			  owl_port->name, port);
	if (unlikely(ret))
		return ret;

	/* clear IRQ pending */
	data = owl_uart_readl(port, UART_STAT);
	data |= UART_STAT_RIP | UART_STAT_TIP;
	owl_uart_writel(port, data, UART_STAT);

	/* enable module/IRQs */
	data = owl_uart_readl(port, UART_CTL);
	data |= UART_CTL_RXIE | UART_CTL_TXIE | UART_CTL_EN;
	owl_uart_writel(port, data, UART_CTL);

	return 0;
}

/*
 * Disable the port
 */
static void owl_shutdown(struct uart_port *port)
{
	unsigned int data;

	/* disable module/IRQs */
	data = owl_uart_readl(port, UART_CTL);
	data &= ~(UART_CTL_RXIE | UART_CTL_TXIE | UART_CTL_EN);
	owl_uart_writel(port, data, UART_CTL);

	free_irq(port->irq, port);
}

/*
 * Change the port parameters
 */
static void owl_set_termios(struct uart_port *port, struct ktermios *termios,
				  struct ktermios *old)
{
	unsigned long flags;
	unsigned int ctl, baud;

	spin_lock_irqsave(&port->lock, flags);

	baud = uart_get_baud_rate(port, termios, old, 0, 115200);
	baud = owl_set_baud_rate(port, baud);
	/*
	 * We don't support modem control lines.
	 */
	termios->c_cflag &= ~(HUPCL | CRTSCTS | CMSPAR);
	termios->c_cflag |= CLOCAL;

	/*
	 * We don't support BREAK character recognition.
	 */
	termios->c_iflag &= ~(IGNBRK | BRKINT);

	ctl = owl_uart_readl(port, UART_CTL);
	ctl &= ~(UART_CTL_DWLS_MASK | UART_CTL_STPS_MASK
			| UART_CTL_PRS_MASK | UART_CTL_AFE);

	ctl &= ~UART_CTL_DWLS_MASK;
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		ctl |= UART_CTL_DWLS_5BITS;
		break;
	case CS6:
		ctl |= UART_CTL_DWLS_6BITS;
		break;
	case CS7:
		ctl |= UART_CTL_DWLS_7BITS;
		break;
	case CS8:
	default:
		ctl |= UART_CTL_DWLS_8BITS;
		break;
	}

	/* stop bits */
	if (termios->c_cflag & CSTOPB)
		ctl |= UART_CTL_STPS_2BITS;
	else
		ctl |= UART_CTL_STPS_1BITS;

	/* parity */
	if (termios->c_cflag & PARENB) {
		/* Mark or Space parity */
		if (termios->c_cflag & CMSPAR) {
			if (termios->c_cflag & PARODD)
				ctl |= UART_CTL_PRS_MARK;
			else
				ctl |= UART_CTL_PRS_SPACE;
		} else if (termios->c_cflag & PARODD)
			ctl |= UART_CTL_PRS_ODD;
		else
			ctl |= UART_CTL_PRS_EVEN;
	} else
		ctl |= UART_CTL_PRS_NONE;

	/* hardware handshake (RTS/CTS) */
	if (termios->c_cflag & CRTSCTS)
		ctl |= UART_CTL_AFE;

	owl_uart_writel(port, ctl, UART_CTL);

	/* Configure status bits to ignore based on termio flags. */

	/*
	 * Normally we need to mask the bits we do care about
	 * as there is no hardware support for
	 * (termios->c_iflag & INPACK/BRKINT/PARMRK)
	 * and it seems the interrupt happened only for tx/rx
	 * we do nothing about the port.read_status_mask
	 */
	port->read_status_mask |= UART_STAT_RXER;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UART_STAT_RXST;

	uart_update_timeout(port, termios->c_cflag, baud);

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *owl_type(struct uart_port *port)
{
	return "OWL_SERIAL";
}

static void owl_release_port(struct uart_port *port)
{
	if (port->flags & UPF_IOREMAP) {
		iounmap(port->membase);
		port->membase = NULL;
	}
}

static int owl_request_port(struct uart_port *port)
{
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *res;
	resource_size_t size;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res)) {
		dev_err(&pdev->dev, "cannot obtain I/O memory region");
		return -ENODEV;
	}
	size = resource_size(res);

	if (!devm_request_mem_region(port->dev, port->mapbase, size,
						dev_name(port->dev))) {
		dev_err(port->dev, "Memory region busy\n");
		return -EBUSY;
	}

	if (port->flags & UPF_IOREMAP) {
		port->membase = devm_ioremap_nocache(port->dev,
						port->mapbase,
						size);

		if (port->membase == NULL)
			return -ENOMEM;
	}

	return 0;
}

static void owl_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_OWL;
		owl_request_port(port);
	}
}

static int owl_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	if (unlikely(ser->type != PORT_UNKNOWN && ser->type != PORT_OWL))
		return -EINVAL;
	if (unlikely(port->irq != ser->irq))
		return -EINVAL;
	return 0;
}

#ifdef CONFIG_CONSOLE_POLL
static int owl_poll_get_char(struct uart_port *port)
{
	unsigned int old_ctl, data;
	unsigned long flags;
	unsigned int ch = NO_POLL_CHAR;

	spin_lock_irqsave(&port->lock, flags);

	/* backup old control register */
	old_ctl = owl_uart_readl(port, UART_CTL);

	/* select RX FIFO */
	data = old_ctl & (~(UART_CTL_TRFS));
	data = data | UART_CTL_TRFS_RX;

	/* disable IRQ */
	/*
	data &= ~(UART_CTL_TXIE | UART_CTL_RXIE);
	*/
	owl_uart_writel(port, data, UART_CTL);

	/* wait RX FIFO not emtpy */
	do {
		cpu_relax();
		/* Get the interrupts */
		data = owl_uart_readl(port, UART_STAT);
	} while ((data & UART_STAT_RIP) == 0);

	while (!(data & UART_STAT_RFEM)) {
		ch = owl_uart_readl(port, UART_RXDAT);
		data = owl_uart_readl(port, UART_STAT);
	}

	/* clear IRQ pending */
	data = owl_uart_readl(port, UART_STAT);
	/*
	data |= UART_STAT_TIP | UART_STAT_RIP;
	*/
	data |= UART_STAT_RIP;
	owl_uart_writel(port, data, UART_STAT);

	/* restore old ctl */
	owl_uart_writel(port, old_ctl, UART_CTL);

	spin_unlock_irqrestore(&port->lock, flags);

	return ch;
}

static void owl_poll_put_char(struct uart_port *port, unsigned char ch)
{
	unsigned int old_ctl, data;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	/* backup old control register */
	old_ctl = owl_uart_readl(port, UART_CTL);

	/* select TX FIFO */
	data = old_ctl & (~(UART_CTL_TRFS));
	data = data | UART_CTL_TRFS_TX;

	/* disable IRQ */
	data &= ~(UART_CTL_TXIE | UART_CTL_RXIE);
	owl_uart_writel(port, data, UART_CTL);

	/* wait TX FIFO not full */
	while (owl_uart_readl(port, UART_STAT) & UART_STAT_TFFU)
		cpu_relax();

	owl_uart_writel(port, ch, UART_TXDAT);

	/* wait until all content have been sent out */
	while (owl_uart_readl(port, UART_STAT) & UART_STAT_TRFL_MASK)
		cpu_relax();

	/* clear IRQ pending */
	data = owl_uart_readl(port, UART_STAT);
	/*
	data |= UART_STAT_TIP | UART_STAT_RIP;
	*/
	data |= UART_STAT_TIP;
	owl_uart_writel(port, data, UART_STAT);
	data = owl_uart_readl(port, UART_STAT);

	/* restore old ctl */
	owl_uart_writel(port, old_ctl, UART_CTL);

	spin_unlock_irqrestore(&port->lock, flags);
	return;
}
#endif

static struct uart_ops owl_uart_pops = {
	.set_mctrl = owl_set_mctrl,
	.get_mctrl = owl_get_mctrl,
	.tx_empty = owl_tx_empty,
	.start_tx = owl_start_tx,
	.stop_tx = owl_stop_tx,
	.stop_rx = owl_stop_rx,
	.enable_ms = owl_enable_ms,
	.break_ctl = owl_break_ctl,
	.startup = owl_startup,
	.shutdown = owl_shutdown,
	.set_termios = owl_set_termios,
	.type = owl_type,
	.config_port = owl_config_port,
	.request_port = owl_request_port,
	.release_port = owl_release_port,
	.verify_port = owl_verify_port,
#ifdef CONFIG_CONSOLE_POLL
	.poll_get_char  = owl_poll_get_char,
	.poll_put_char  = owl_poll_put_char,
#endif
};

#ifdef CONFIG_SERIAL_OWL_CONSOLE
static void owl_console_putchar(struct uart_port *port, int c)
{
	while (owl_uart_readl(port, UART_STAT) & UART_STAT_TFFU)
		cpu_relax();
	owl_uart_writel(port, c, UART_TXDAT);
}

static void owl_console_write(struct console *co, const char *s,
				  unsigned int count)
{
	struct uart_port *port;
	unsigned int old_ctl, data;

	BUG_ON(co->index < 0 || co->index >= OWL_MAX_UART);

	port = get_port_from_line(co->index);

	spin_lock(&port->lock);

	/* backup old control register */
	old_ctl = owl_uart_readl(port, UART_CTL);

	/* disable IRQ */
	data = old_ctl | UART_CTL_TRFS_TX;
	data &= ~(UART_CTL_TXIE | UART_CTL_RXIE);
	owl_uart_writel(port, data, UART_CTL);

	uart_console_write(port, s, count, owl_console_putchar);

	/* wait until all content have been sent out */
	while (owl_uart_readl(port, UART_STAT) & UART_STAT_TRFL_MASK)
		;

	/* clear IRQ pending */
	data = owl_uart_readl(port, UART_STAT);
	data |= UART_STAT_TIP | UART_STAT_RIP;
	owl_uart_writel(port, data, UART_STAT);

	/* restore old ctl */
	owl_uart_writel(port, old_ctl, UART_CTL);

	spin_unlock(&port->lock);
}

static int __init owl_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (unlikely(co->index >= OWL_MAX_UART || co->index < 0))
		return -ENXIO;

	port = get_port_from_line(co->index);

	if (unlikely(!port->membase))
		return -ENXIO;

	port->cons = co;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver owl_uart_driver;

static struct console owl_console = {
	.name = "ttyS",
	.write = owl_console_write,
	.device = uart_console_device,
	.setup = owl_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &owl_uart_driver,
};

#define OWL_CONSOLE	(&owl_console)

#else
#define OWL_CONSOLE	NULL
#endif

static struct uart_driver owl_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "owl_serial",
	.dev_name = "ttyS",
	.nr = OWL_MAX_UART,
	.cons = OWL_CONSOLE,
};

static int __init owl_serial_probe(struct platform_device *pdev)
{
	struct resource *res_mem, *res_irq;
	struct uart_port *u;
	int line;

	line = of_alias_get_id(pdev->dev.of_node, "serial");

	if (line < 0)
		return -EINVAL;

	u = &owl_ports[line].uart;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_mem)
		return -ENODEV;

	res_irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res_irq)
		return -ENODEV;

	u->mapbase	= res_mem->start;
	u->irq		= res_irq->start;
	u->line		= line;

	u->iotype	= UPIO_MEM;
	u->flags	= UPF_BOOT_AUTOCONF | UPF_IOREMAP | UPF_LOW_LATENCY;
	u->ops		= &owl_uart_pops;
	u->fifosize	= 16;
	u->dev		= &pdev->dev;

	platform_set_drvdata(pdev, u);

	return uart_add_one_port(&owl_uart_driver, u);
}

static int __exit owl_serial_remove(struct platform_device *pdev)
{
	struct uart_port *port = platform_get_drvdata(pdev);

	device_init_wakeup(&pdev->dev, 0);
	platform_set_drvdata(pdev, NULL);

	return uart_remove_one_port(&owl_uart_driver, port);
}

static struct of_device_id owl_uart_of_match[] = {
	{.compatible = "actions,atm7039-uart",},
	{},
};
MODULE_DEVICE_TABLE(of, owl_uart_of_match);

static struct platform_driver owl_platform_driver = {
	.remove = owl_serial_remove,
	.driver = {
		.name	= "owl-serial",
		.owner	= THIS_MODULE,
		.of_match_table	= owl_uart_of_match,
	},
};

static int __init owl_serial_init(void)
{
	int ret;

	ret = uart_register_driver(&owl_uart_driver);
	if (unlikely(ret))
		return ret;

	ret = platform_driver_probe(&owl_platform_driver, owl_serial_probe);
	if (unlikely(ret))
		uart_unregister_driver(&owl_uart_driver);

	return ret;
}

static void __exit owl_serial_exit(void)
{
	platform_driver_unregister(&owl_platform_driver);
	uart_unregister_driver(&owl_uart_driver);
}

module_init(owl_serial_init);
module_exit(owl_serial_exit);

MODULE_AUTHOR("Yixun Lan <yixun.lan@gmail.com>");
MODULE_DESCRIPTION("Actions OWL SoC serial driver");
MODULE_LICENSE("GPL v2");
