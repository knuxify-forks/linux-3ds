// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  ctr_pxi_irqc.c
 *
 *  Copyright (C) 2021 Santiago Herrera
 */

/** PXI Interrupt controller functions */
#define CTR_PXI_NR_IRQS	32

static irqreturn_t pxi_sync_thread(int irq, void *data)
{
	int err;
	u32 pend;
	irqreturn_t ret = IRQ_NONE;
	struct ctr_pxi_host *pxi = data;

	static const u32 ctr_pxi_cmd_irqget =
		CTR_PXI_CMD(CTR_PXI_IRQGET, 0, 0);

	mutex_lock(&pxi->irq_lock);
	do { /* keep requesting the highest priority IRQ until there's none */
		err = pxi_cmd_one(pxi, ctr_pxi_cmd_irqget, &pend);

		if (pend >= CTR_PXI_NR_IRQS)
			break;

		ret = IRQ_HANDLED;
		generic_handle_irq(irq_find_mapping(pxi->irqdom, pend));
	} while(1);
	mutex_unlock(&pxi->irq_lock);

	return ret;
}

static void ctr_pxi_irqc_irq_toggle(struct irq_data *data, u32 cmd)
{
	int err;

	struct irq_chip_generic *irqgc = irq_data_get_irq_chip_data(data);
	struct ctr_pxi_host *pxi = irqgc->private;

	mutex_lock(&pxi->irq_lock);
	err = pxi_cmd_one(pxi, CTR_PXI_CMD(cmd, 0, data->hwirq), NULL);
	WARN_ON(err);
	mutex_unlock(&pxi->irq_lock);
}

static void ctr_pxi_irqc_irq_mask(struct irq_data *data)
{
	ctr_pxi_irqc_irq_toggle(data, CTR_PXI_IRQMASK);
}

static void ctr_pxi_irqc_irq_unmask(struct irq_data *data)
{
	ctr_pxi_irqc_irq_toggle(data, CTR_PXI_IRQUNMASK);
}

static int ctr_pxi_irqc_init(struct ctr_pxi_host *pxi)
{
	int err, irq_base;
	struct irq_chip_type *ct;
	struct device *dev = pxi->dev;

	irq_base = devm_irq_alloc_descs(dev, -1, 0, CTR_PXI_NR_IRQS, 0);
	if (irq_base < 0)
		return irq_base;

	pxi->irqdom = irq_domain_add_linear(dev->of_node, CTR_PXI_NR_IRQS,
					    &irq_domain_simple_ops, pxi);
	if (!pxi->irqdom)
		return -ENXIO;

	irq_domain_associate_many(pxi->irqdom, irq_base, 0, CTR_PXI_NR_IRQS);

	pxi->irqgc = devm_irq_alloc_generic_chip(dev, dev_name(dev), 1,
						 irq_base, NULL,
						 handle_simple_irq);
	if (!pxi->irqgc) {
		err = -ENOMEM;
		goto remove_irqdom;
	}

	pxi->irqgc->private = pxi;

	ct = pxi->irqgc->chip_types;
	ct->chip.irq_mask = ctr_pxi_irqc_irq_mask;
	ct->chip.irq_unmask = ctr_pxi_irqc_irq_unmask;

	err = devm_irq_setup_generic_chip(dev, pxi->irqgc,
					  IRQ_MSK(CTR_PXI_NR_IRQS), 0,
					  IRQF_SHARED | IRQF_NO_THREAD,
					  IRQF_ONESHOT);

	remove_irqdom:
	if (err < 0) {
		irq_domain_remove(pxi->irqdom);
		return err;
	}

	return 0;
}
