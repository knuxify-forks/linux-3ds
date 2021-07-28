// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Nintendo 3DS Secure Digital Host Controller driver
 *
 *  Copyright (C) 2021 Santiago Herrera
 *
 *  Based on toshsd.c, copyright (C) 2014 Ondrej Zary and 2007 Richard Betts
 */

//#define DRIVER_NAME "3ds-sdhc"
//#define pr_fmt(fmt) DRIVER_NAME ": " fmt

#include <linux/io.h>
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/scatterlist.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>

#include "ctr_sdhc.h"

static u16 ctr_sdhc_reg16_get(struct ctr_sdhc *host, unsigned off);
static void ctr_sdhc_reg16_set(struct ctr_sdhc *host, unsigned off, u16 val);
static u32 ctr_sdhc_reg32_get(struct ctr_sdhc *host, unsigned off);
static void ctr_sdhc_reg32_set(struct ctr_sdhc *host, unsigned off, u32 val);

static void ctr_sdhc_reset(struct ctr_sdhc *host, u32 default_irqmask);
static void ctr_sdhc_set_clk_opt(struct ctr_sdhc *host, u16 clk, u16 opt);

static void ctr_sdhc_send_cmdarg(struct ctr_sdhc *host, u16 cmd, u32 arg);
static void ctr_sdhc_set_blk_len_cnt(struct ctr_sdhc *host, u16 len, u16 cnt);
static void ctr_sdhc_get_resp(struct ctr_sdhc *host, u32 *resp, unsigned nword);
static void ctr_sdhc_stop_internal_set(struct ctr_sdhc *host, u16 val);

static u32 ctr_sdhc_irqstat_get(struct ctr_sdhc *host);
static void ctr_sdhc_irqstat_ack(struct ctr_sdhc *host, u32 ack);
static void ctr_sdhc_irqmask_set(struct ctr_sdhc *host, u32 mask);

static int ctr_sdhc_sdioirq_test(struct ctr_sdhc *host);
static void ctr_sdhc_sdioirq_set(struct ctr_sdhc *host, int enable);

#define SDHC_ERR_MASK                                                          \
	(SDHC_ERR_BAD_CMD | SDHC_ERR_CRC_FAIL | SDHC_ERR_STOP_BIT |            \
	 SDHC_ERR_DATATIMEOUT | SDHC_ERR_TX_OVERFLOW | SDHC_ERR_RX_UNDERRUN |  \
	 SDHC_ERR_CMD_TIMEOUT | SDHC_ERR_ILLEGAL_ACC)

#define SDHC_DEFAULT_IRQMASK                                                   \
	(SDHC_STAT_CMDRESPEND | SDHC_STAT_DATA_END | SDHC_STAT_RX_READY |      \
	 SDHC_STAT_TX_REQUEST | SDHC_STAT_CARDREMOVE | SDHC_STAT_CARDINSERT |  \
	 SDHC_ERR_MASK)

static void __ctr_sdhc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	u16 clk_ctl, card_opt;
	struct ctr_sdhc *host = mmc_priv(mmc);

	if (ios->clock) {
		int clk_div = -1;
		unsigned clk_fit = clk_get_rate(host->sdclk) / 2;

		/* get the divider that best achieves the desired clkrate */
		while ((ios->clock < clk_fit) && (clk_div < 7)) {
			clk_div++;
			clk_fit >>= 1;
		}

		clk_ctl = (BIT(clk_div + 2) >> 2) | SDHC_CARD_CLKCTL_PIN_ENABLE;
	} else {
		clk_ctl = 0;
	}

	card_opt = SDHC_CARD_OPTION_RETRIES(14) |
		   SDHC_CARD_OPTION_TIMEOUT(14) |
		   SDHC_CARD_OPTION_NOC2;

	switch (ios->bus_width) {
	default:
		dev_err(host->dev, "invalid bus width %d\n", ios->bus_width);
		return;
	case MMC_BUS_WIDTH_1:
		card_opt |= SDHC_CARD_OPTION_1BIT;
		break;
	case MMC_BUS_WIDTH_4:
		card_opt |= SDHC_CARD_OPTION_4BIT;
		break;
	}

	if (ios->power_mode == MMC_POWER_OFF)
		clk_ctl = 0; /* force-disable clock */

	/* set the desired clock divider and card option config */
	ctr_sdhc_set_clk_opt(host, clk_ctl, card_opt);

	/* wait a bit */
	mdelay(20);
}

static void ctr_sdhc_finish_request(struct ctr_sdhc *host)
{
	struct mmc_request *mrq = host->mrq;

	host->mrq = NULL;
	host->cmd = NULL;
	host->data = NULL;

	mmc_request_done(host->mmc, mrq);
}

static void ctr_sdhc_data_end_irq(struct ctr_sdhc *host)
{
	struct mmc_data *data = host->data;

	host->data = NULL;

	if (!data) {
		dev_warn(host->dev, "Spurious data end IRQ\n");
		return;
	}

	if (data->error == 0)
		data->bytes_xfered = data->blocks * data->blksz;
	else
		data->bytes_xfered = 0;

	dev_dbg(host->dev, "Completed data request xfr=%d\n",
		data->bytes_xfered);

	ctr_sdhc_finish_request(host);
}

static irqreturn_t ctr_sdhc_thread_irq(int irq, void *dev_id)
{
	struct ctr_sdhc *host = dev_id;
	struct mmc_data *data = host->data;
	struct sg_mapping_iter *sg_miter = &host->sg_miter;
	u8 *buf;
	int count;
	unsigned long flags;

	if (!data) {
		dev_warn(host->dev, "Spurious Data IRQ\n");
		if (host->cmd) {
			host->cmd->error = -EIO;
			ctr_sdhc_finish_request(host);
		}
		return IRQ_NONE;
	}
	spin_lock_irqsave(&host->lock, flags);

	if (!sg_miter_next(sg_miter))
		goto done;

	buf = sg_miter->addr;

	count = sg_miter->length;
	if (count > data->blksz)
		count = data->blksz;

	dev_dbg(host->dev, "count: %08x, flags %08x\n", count, data->flags);

	if (data->flags & MMC_DATA_READ) {
		ioread16_rep(host->regs + SDHC_DATA16_FIFO_PORT, buf,
			     count >> 1);
	} else {
		iowrite16_rep(host->regs + SDHC_DATA16_FIFO_PORT, buf,
			      count >> 1);
	}

	sg_miter->consumed = count;
	sg_miter_stop(sg_miter);

done:
	spin_unlock_irqrestore(&host->lock, flags);
	return IRQ_HANDLED;
}

static void ctr_sdhc_respend_irq(struct ctr_sdhc *host)
{
	u32 response[4], *respbuf;
	struct mmc_command *cmd = host->cmd;

	if (!host->cmd) {
		dev_err(host->dev, "Spurious CMD irq\n");
		return;
	}

	respbuf = (u32*)cmd->resp;
	host->cmd = NULL;

	ctr_sdhc_get_resp(host, response, 4);

	if (cmd->flags & MMC_RSP_PRESENT && cmd->flags & MMC_RSP_136) {
		respbuf[0] = (response[3] << 8) | (response[2] >> 24);
		respbuf[1] = (response[2] << 8) | (response[1] >> 24);
		respbuf[2] = (response[1] << 8) | (response[0] >> 24);
		respbuf[3] = response[0] << 8;
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		respbuf[0] = response[0];
	}

	dev_dbg(host->dev, "Command IRQ complete %d %d %x\n", cmd->opcode,
		cmd->error, cmd->flags);

	/* If there is data to handle we will
	 * finish the request in the data end irq handler.*/
	if (host->data)
		return;

	ctr_sdhc_finish_request(host);
}

static irqreturn_t ctr_sdhc_irq(int irq, void *dev_id)
{
	struct ctr_sdhc *host = dev_id;
	u32 int_reg;

	int error = 0, ret = IRQ_HANDLED;

	spin_lock(&host->lock);

	int_reg = ctr_sdhc_irqstat_get(host);
	dev_dbg(host->dev, "IRQ status: %x\n", int_reg);

	if (!int_reg) {
		ret = IRQ_NONE;
		goto irq_end;
	}

	ctr_sdhc_irqstat_ack(host, int_reg & SDHC_DEFAULT_IRQMASK);

	if (int_reg & (SDHC_STAT_CARDREMOVE | SDHC_STAT_CARDINSERT)) {
		if (int_reg & SDHC_STAT_CARDPRESENT)
			ctr_sdhc_reset(host, SDHC_DEFAULT_IRQMASK);
		mmc_detect_change(host->mmc, 1);
	}

	if (int_reg & SDHC_ERR_CMD_TIMEOUT) {
		error = -ETIMEDOUT;
	} else if (int_reg & SDHC_ERR_CRC_FAIL) {
		error = -EILSEQ;
	} else if (int_reg & SDHC_ERR_MASK) {
		dev_err(host->dev, "buffer error: %08X\n",
			int_reg & SDHC_ERR_MASK);
		/*dev_err(host->dev, "detail error status %08X\n",
			ioread32(host->regs + SDHC_ERROR_STATUS));*/
		error = -EIO;
	}

	if (error) {
		if (host->cmd)
			host->cmd->error = error;

		if (error != -ETIMEDOUT) {
			ctr_sdhc_reset(host, SDHC_DEFAULT_IRQMASK);
			__ctr_sdhc_set_ios(host->mmc, &host->mmc->ios);
			goto irq_end;
		}
	}

	if (int_reg & (SDHC_STAT_RX_READY | SDHC_STAT_TX_REQUEST)) {
		ret = IRQ_WAKE_THREAD;
		goto irq_end;
	}

	if (int_reg & SDHC_STAT_CMDRESPEND)
		ctr_sdhc_respend_irq(host);

	if (int_reg & SDHC_STAT_DATA_END)
		ctr_sdhc_data_end_irq(host);

irq_end:
	spin_unlock(&host->lock);
	return ret;
}

static irqreturn_t ctr_sdhc_sdio_irq(int irq, void *data)
{
	struct ctr_sdhc *host = data;
	if (ctr_sdhc_sdioirq_test(host)) {
		mmc_signal_sdio_irq(host->mmc);
		return IRQ_HANDLED;
	}
	return IRQ_NONE;
}

static void ctr_sdhc_start_cmd(struct ctr_sdhc *host, struct mmc_command *cmd)
{
	struct mmc_data *data = host->data;
	int c = cmd->opcode;

	dev_dbg(host->dev, "Command opcode: %d\n", cmd->opcode);

	if (cmd->opcode == MMC_STOP_TRANSMISSION) {
		ctr_sdhc_stop_internal_set(host, SDHC_STOP_INTERNAL_ISSUE);

		cmd->resp[0] = cmd->opcode;
		cmd->resp[1] = 0;
		cmd->resp[2] = 0;
		cmd->resp[3] = 0;

		ctr_sdhc_finish_request(host);
		return;
	}

	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		c |= SDHC_CMDRSP_NONE;
		break;
	case MMC_RSP_R1:
		c |= SDHC_CMDRSP_R1;
		break;
	case MMC_RSP_R1B:
		c |= SDHC_CMDRSP_R1B;
		break;
	case MMC_RSP_R2:
		c |= SDHC_CMDRSP_R2;
		break;
	case MMC_RSP_R3:
		c |= SDHC_CMDRSP_R3;
		break;

	default:
		dev_err(host->dev, "Unknown response type %d\n",
			mmc_resp_type(cmd));
		break;
	}

	host->cmd = cmd;

	if (cmd->opcode == SD_IO_RW_EXTENDED)
		c |= SDHC_CMD_SECURE;

	if (cmd->opcode == SD_IO_RW_DIRECT)
		c |= SDHC_CMD_SECURE;

	if (cmd->opcode == MMC_APP_CMD)
		c |= SDHC_CMDTYPE_APP;

	if (cmd->opcode == MMC_GO_IDLE_STATE)
		c |= SDHC_CMDRSP_NONE;

	if (data) {
		c |= SDHC_CMD_DATA_XFER;

		if (data->blocks > 0) {
			ctr_sdhc_stop_internal_set(host,
				SDHC_STOP_INTERNAL_ENABLE);
			c |= SDHC_CMD_DATA_MULTI;
		} else {
			ctr_sdhc_stop_internal_set(host, 0);
		}

		if (data->flags & MMC_DATA_READ)
			c |= SDHC_CMD_DATA_READ;
	}

	ctr_sdhc_send_cmdarg(host, c, cmd->arg);
}

static void ctr_sdhc_start_data(struct ctr_sdhc *host, struct mmc_data *data)
{
	unsigned int flags = SG_MITER_ATOMIC;

	dev_dbg(host->dev,
		"setup data transfer: blocksize %08x "
		"nr_blocks %d, offset: %08x\n",
		data->blksz, data->blocks, data->sg->offset);

	host->data = data;

	if (data->flags & MMC_DATA_READ)
		flags |= SG_MITER_TO_SG;
	else
		flags |= SG_MITER_FROM_SG;

	sg_miter_start(&host->sg_miter, data->sg, data->sg_len, flags);

	ctr_sdhc_set_blk_len_cnt(host, data->blksz, data->blocks);
}

/* Process requests from the MMC layer */
static void ctr_sdhc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct ctr_sdhc *host = mmc_priv(mmc);
	unsigned long flags;

	if (!(ioread16(host->regs + SDHC_IRQ_STAT) & SDHC_STAT_CARDPRESENT)) {
		mrq->cmd->error = -ENOMEDIUM;
		mmc_request_done(mmc, mrq);
		return;
	}

	spin_lock_irqsave(&host->lock, flags);

	WARN_ON(host->mrq != NULL);

	host->mrq = mrq;

	if (mrq->data)
		ctr_sdhc_start_data(host, mrq->data);

	ctr_sdhc_start_cmd(host, mrq->cmd);

	spin_unlock_irqrestore(&host->lock, flags);
}

static void ctr_sdhc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct ctr_sdhc *host = mmc_priv(mmc);
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);
	__ctr_sdhc_set_ios(mmc, ios);
	spin_unlock_irqrestore(&host->lock, flags);
}

static int ctr_sdhc_get_ro(struct mmc_host *mmc)
{
	struct ctr_sdhc *host = mmc_priv(mmc);
	return !(ctr_sdhc_irqstat_get(host) & SDHC_STAT_WRITEPROT);
}

static int ctr_sdhc_get_cd(struct mmc_host *mmc)
{
	struct ctr_sdhc *host = mmc_priv(mmc);
	return !!(ctr_sdhc_irqstat_get(host) & SDHC_STAT_CARDPRESENT);
}

static void ctr_sdhc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct ctr_sdhc *host = mmc_priv(mmc);
	pr_err("ctr_sdhc_enable_sdio_irq %d\n", enable);
	ctr_sdhc_sdioirq_set(host, enable);
}

static const struct mmc_host_ops ctr_sdhc_ops = {
	.request = ctr_sdhc_request,
	.set_ios = ctr_sdhc_set_ios,
	.get_ro = ctr_sdhc_get_ro,
	.get_cd = ctr_sdhc_get_cd,
	.enable_sdio_irq = ctr_sdhc_enable_sdio_irq,
};

#ifdef CONFIG_PM_SLEEP
static int ctr_sdhc_pm_suspend(struct device *dev)
{
	struct ctr_sdhc *host = dev_get_drvdata(dev);
	ctr_sdhc_irqmask_set(host, ~0);
	ctr_sdhc_set_clk_opt(host, 0, SDHC_CARD_OPTION_1BIT);
	return 0;
}

static int ctr_sdhc_pm_resume(struct device *dev)
{
	struct ctr_sdhc *host = dev_get_drvdata(dev);
	ctr_sdhc_reset(host, SDHC_DEFAULT_IRQMASK);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static int ctr_sdhc_probe(struct platform_device *pdev)
{
	int ret;
	struct clk *sdclk;
	struct device *dev;
	struct mmc_host *mmc;
	unsigned long clkrate;
	struct ctr_sdhc *host;

	dev = &pdev->dev;

	sdclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sdclk))
		return PTR_ERR(sdclk);

	ret = clk_prepare_enable(sdclk);
	if (ret)
		return ret;
	clkrate = clk_get_rate(sdclk);

	mmc = mmc_alloc_host(sizeof(struct ctr_sdhc), dev);
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->sdclk = sdclk;

	host->dev = dev;
	platform_set_drvdata(pdev, host);

	host->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(host->regs)) {
		ret = -ENOMEM;
		goto free_mmc;
	}

	mmc->ops = &ctr_sdhc_ops;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED |
		    MMC_CAP_SD_HIGHSPEED | MMC_CAP_SDIO_IRQ;
	mmc->ocr_avail = MMC_VDD_32_33;
	mmc->max_blk_size = 0x200;
	mmc->max_blk_count = 0xFFFF;

	mmc->f_min = clkrate >> 9;
	mmc->f_max = clkrate >> 1;

	spin_lock_init(&host->lock);

	ctr_sdhc_reset(host, SDHC_DEFAULT_IRQMASK);

	ret = devm_request_threaded_irq(dev, platform_get_irq(pdev, 0),
					ctr_sdhc_irq, ctr_sdhc_thread_irq,
					IRQF_SHARED, DRIVER_NAME, host);
	if (ret)
		goto free_mmc;

	ret = devm_request_irq(dev, platform_get_irq(pdev, 1),
			       ctr_sdhc_sdio_irq, 0, DRIVER_NAME, host);
	if (ret)
		goto free_mmc;

	mmc_add_host(mmc);
	pm_suspend_ignore_children(&pdev->dev, 1);
	return 0;

free_mmc:
	mmc_free_host(mmc);
	return ret;
}

static int ctr_sdhc_remove(struct platform_device *pdev)
{
	return -EBUSY;
}

static const struct dev_pm_ops ctr_sdhc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(ctr_sdhc_pm_suspend, ctr_sdhc_pm_resume)
};

static const struct of_device_id ctr_sdhc_of_match[] = {
	{ .compatible = "nintendo," DRIVER_NAME },
	{},
};
MODULE_DEVICE_TABLE(of, ctr_sdhc_of_match);

static struct platform_driver ctr_sdhc_driver = {
	.probe = ctr_sdhc_probe,
	.remove = ctr_sdhc_remove,

	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ctr_sdhc_of_match),
		.pm = &ctr_sdhc_pm_ops
	},
};

module_platform_driver(ctr_sdhc_driver);

MODULE_DESCRIPTION("Nintendo 3DS SDHC driver");
MODULE_AUTHOR("Santiago Herrera");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
