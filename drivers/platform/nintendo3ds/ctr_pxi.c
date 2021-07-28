// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  ctr_pxi.c
 *
 *  Copyright (C) 2021 Santiago Herrera
 */

#include <linux/of.h>
#include <linux/device.h>

#include "ctr_pxi.h"

/** PXI Transport protocol helpers */
#define CTR_PXI_CMD_MASK	(0xF0000000)

static int pxi_cmd_needs_rx(u32 cmd) {
	cmd &= CTR_PXI_CMD_MASK;
	if (cmd == CTR_PXI_READ) return 1;
	if (cmd == CTR_PXI_IRQGET) return 1;
	return 0;
}

static int pxi_cmd_needs_tx(u32 cmd) {
	cmd &= CTR_PXI_CMD_MASK;
	if (cmd == CTR_PXI_WRITE) return 1;
	if (cmd == CTR_PXI_IOSET) return 1;
	if (cmd == CTR_PXI_IOCLR) return 1;
	return 0;
}

static int pxi_cmd_one(struct ctr_pxi_host *pxi, u32 cmd, u32 *arg)
{
	int err;

	/* TODO: defer RXing the READ commands until all the WRITE are sent */

	/* first send the base cmd */
	err = pxi_tx_one(pxi, cmd);
	if (err) return err;

	/* transmit and receive as necessary */	
	if (pxi_cmd_needs_tx(cmd)) {
		err = pxi_tx_one(pxi, *arg);
	} else if (pxi_cmd_needs_rx(cmd)) {
		err = pxi_rx_one(pxi, arg);
	}

	WARN_ON_ONCE(err);

	return err;
}

struct ctr_pxi_host *ctr_pxi_host_get(struct device *dev)
{
	struct device *pxi_dev;
	struct device_node *pxi_np;

	if (!dev)
		return ERR_PTR(-ENODEV);

	if (!dev->parent)
		return ERR_PTR(-ENODEV);

	pxi_dev = dev->parent;
	pxi_np = of_node_get(pxi_dev->of_node);

	/* minimal checking */
	if (!of_device_is_compatible(pxi_np, DRIVER_COMPAT_STR)) {
		of_node_put(pxi_np);
		return ERR_PTR(-EINVAL);
	}

	pr_info("%s acquired a reference\n",
		of_node_full_name(dev->of_node));

	/* keep the reference to pxi_np alive */
	return dev_get_drvdata(pxi_dev);
}
EXPORT_SYMBOL_GPL(ctr_pxi_host_get);

void ctr_pxi_host_put(struct device *dev, struct ctr_pxi_host *pxi)
{
	if (pxi && pxi->dev) {
		pr_debug("%s dropped a reference\n",
			 of_node_full_name(dev->of_node));
		of_node_put(pxi->dev->of_node);
		/* drop the reference acquired previously */
	}
	else
		pr_err("ctr_pxi_host_put: passed nullptr");
}
EXPORT_SYMBOL_GPL(ctr_pxi_host_put);

int ctr_pxi_cmd(struct ctr_pxi_host *pxi, const u32 *cmds, u32 *args, u32 ncmd)
{
	int err;

	if (!ncmd)
		return 0;

	might_sleep();
	mutex_lock(&pxi->fifo_lock);

	while(ncmd) {
		err = pxi_cmd_one(pxi, *cmds, args);
		if (err) break;

		cmds++;
		args++;
		ncmd--;
	}

	mutex_unlock(&pxi->fifo_lock);
	return err;
}
EXPORT_SYMBOL_GPL(ctr_pxi_cmd);
