// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  ctr_pxi_hw.c
 *
 *  Copyright (C) 2021 Santiago Herrera
 */

/* Main driver code, incorporates the rest of the source via #include */

#define DRIVER_NAME "3ds-pxi"
#define pr_fmt(str) DRIVER_NAME ": " str

#define DRIVER_COMPAT_STR	"nintendo," DRIVER_NAME

#include <linux/io.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irqdomain.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

struct ctr_pxi_host {
	struct device *dev;
	void __iomem *regs;

	struct list_head list;

	struct mutex fifo_lock;
	wait_queue_head_t fifo_wq;

	struct mutex irq_lock;
	struct irq_domain *irqdom;
	struct irq_chip_generic *irqgc;
};

#define REG_PXI_SYNCRX	0x00
#define REG_PXI_SYNCTX	0x01
#define REG_PXI_SYNCIRQ	0x03

#define REG_PXI_CNT	0x04
#define REG_PXI_TX	0x08
#define REG_PXI_RX	0x0C

#define PXI_CNT_TX_FULL	BIT(1)
#define PXI_CNT_TX_IRQ	BIT(2)
#define PXI_CNT_FIFO_FLUSH	BIT(3)
#define PXI_CNT_RX_EMPTY	BIT(8)
#define PXI_CNT_RX_IRQ	BIT(10)
#define PXI_CNT_ERRACK	BIT(14)
#define PXI_CNT_ENABLE	BIT(15)

#define PXI_SYNCIRQ_TRIGGER	BIT(6)
#define PXI_SYNCIRQ_ENABLE	BIT(7)

#define PXI_FIFO_DEPTH	(16)

#define CTR_PXI_FIFO_TIMEOUT	msecs_to_jiffies(250)

/* PXI hardware layer wrappers */
static int pxi_tx_full(struct ctr_pxi_host *pxi)
{
	return ioread16(pxi->regs + REG_PXI_CNT) & PXI_CNT_TX_FULL;
}

static int pxi_rx_empty(struct ctr_pxi_host *pxi)
{
	return ioread16(pxi->regs + REG_PXI_CNT) & PXI_CNT_RX_EMPTY;
}

static int pxi_check_err(struct ctr_pxi_host *pxi)
{
	if (ioread32(pxi->regs + REG_PXI_CNT) & PXI_CNT_ERRACK) {
		iowrite32(PXI_CNT_FIFO_FLUSH | PXI_CNT_ERRACK | PXI_CNT_ENABLE,
			  pxi->regs + REG_PXI_CNT);
		return -EIO;
	}
	return 0;
}

static int pxi_tx_one(struct ctr_pxi_host *pxi, u32 data)
{
	if (wait_event_interruptible_timeout(pxi->fifo_wq,
			!pxi_tx_full(pxi), CTR_PXI_FIFO_TIMEOUT) <= 0)
		return -ETIMEDOUT;
	iowrite32(data, pxi->regs + REG_PXI_TX);
	return pxi_check_err(pxi);
}

static int pxi_rx_one(struct ctr_pxi_host *pxi, u32 *data)
{
	if (wait_event_interruptible_timeout(pxi->fifo_wq,
			!pxi_rx_empty(pxi), CTR_PXI_FIFO_TIMEOUT) <= 0)
		return -ETIMEDOUT;
	*data = ioread32(pxi->regs + REG_PXI_RX);
	return pxi_check_err(pxi);
}

#include "ctr_pxi.c"	/* Main transport layer and device management */
#include "ctr_pxi_irqc.c"	/* IRQ Controller */

static irqreturn_t pxi_fifo_irq(int irq, void *data)
{
	struct ctr_pxi_host *pxi = data;
	wake_up_interruptible(&pxi->fifo_wq);
	return IRQ_HANDLED;
}

/** PXI basic initialization and probing */
static void ctr_pxi_reset_hw(struct ctr_pxi_host *pxi)
{
	int i;

	/* reset the entire hardware block to a known-good state */
	iowrite8(0, pxi->regs + REG_PXI_SYNCTX);
	iowrite8(0, pxi->regs + REG_PXI_SYNCIRQ);
	iowrite16(PXI_CNT_FIFO_FLUSH | PXI_CNT_ERRACK | PXI_CNT_ENABLE,
		  pxi->regs + REG_PXI_CNT);

	for (i = 0; i < PXI_FIFO_DEPTH; i++)
		ioread32(pxi->regs + REG_PXI_RX);

	iowrite16(0, pxi->regs + REG_PXI_CNT);

	iowrite8(PXI_SYNCIRQ_ENABLE, pxi->regs + REG_PXI_SYNCIRQ);
	iowrite16(PXI_CNT_RX_IRQ | PXI_CNT_TX_IRQ | PXI_CNT_ERRACK |
		  PXI_CNT_FIFO_FLUSH | PXI_CNT_ENABLE,
		  pxi->regs + REG_PXI_CNT);
}

static int ctr_pxi_probe(struct platform_device *pdev)
{
	int err;
	struct device *dev;
	struct ctr_pxi_host *pxi;
	int sync_irq, tx_irq, rx_irq;

	dev = &pdev->dev;

	sync_irq = platform_get_irq(pdev, 0);
	tx_irq = platform_get_irq(pdev, 1);
	rx_irq = platform_get_irq(pdev, 2);

	if ((sync_irq < 0) || (tx_irq < 0) || (rx_irq < 0)) {
		pr_err("failed to retrieve interrupts");
		return -EINVAL;
	}

	pxi = devm_kzalloc(dev, sizeof(*pxi), GFP_KERNEL);
	if (!pxi)
		return -ENOMEM;

	pxi->dev = dev;
	mutex_init(&pxi->irq_lock);
	mutex_init(&pxi->fifo_lock);
	init_waitqueue_head(&pxi->fifo_wq);

	pxi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(pxi->regs))
		return PTR_ERR(pxi->regs);

	platform_set_drvdata(pdev, pxi);

	/* reset the hardware block */
	ctr_pxi_reset_hw(pxi);

	/* set up the host irq controller */
	err = ctr_pxi_irqc_init(pxi);
	if (err) return err;

	/* request the sync and fifo interrupts */
	err = devm_request_threaded_irq(dev, sync_irq, NULL, pxi_sync_thread,
					IRQF_ONESHOT, "pxi_sync", pxi);
	if (err) return err;

	err = devm_request_irq(dev, tx_irq, pxi_fifo_irq, 0, "pxi_tx", pxi);
	if (err) return err;

	err = devm_request_irq(dev, rx_irq, pxi_fifo_irq, 0, "pxi_rx", pxi);
	if (err) return err;

	return devm_of_platform_populate(dev);
}

static int ctr_pxi_remove(struct platform_device *pdev)
{
	return -EINVAL;
}

static const struct of_device_id ctr_pxi_of_match[] = {
	{ .compatible = DRIVER_COMPAT_STR },
	{},
};
MODULE_DEVICE_TABLE(of, ctr_pxi_of_match);

static struct platform_driver ctr_pxi_driver = {
	.probe	= ctr_pxi_probe,
	.remove = ctr_pxi_remove,

	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(ctr_pxi_of_match),
	},
};
module_platform_driver(ctr_pxi_driver);

MODULE_AUTHOR("Santiago Herrera");
MODULE_DESCRIPTION("Nintendo 3DS PXI virtio bridge");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: " DRIVER_NAME);
