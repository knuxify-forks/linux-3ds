// SPDX-License-Identifier: GPL-2.0-or-later

#define DRIVER_NAME	"3ds-sdhc"
#define pr_fmt(fmt) DRIVER_NAME ": " fmt

#include <linux/io.h>

#include "ctr_sdhc.c"

/* basic primitives */
static u16 ctr_sdhc_reg16_get(struct ctr_sdhc *host, unsigned off)
{
	return ioread16(host->regs + off);
}

static void ctr_sdhc_reg16_set(struct ctr_sdhc *host, unsigned off, u16 val)
{
	iowrite16(val, host->regs + off);
}

static u32 ctr_sdhc_reg32_get(struct ctr_sdhc *host, unsigned off)
{
	return ioread32(host->regs + off);
}

static void ctr_sdhc_reg32_set(struct ctr_sdhc *host, unsigned off, u32 val)
{
	iowrite32(val, host->regs + off);
}

/* slightly more sdhc-related stuff */
static void ctr_sdhc_reset(struct ctr_sdhc *host)
{
	/* reset controller */
	ctr_sdhc_reg16_set(host, SDHC_SOFTRESET, 0);
	ctr_sdhc_reg16_set(host, SDHC_SOFTRESET, 1);

	/* clear registers */
	ctr_sdhc_reg16_set(host, SDHC_CARD_PORTSEL, 0);
	ctr_sdhc_reg16_set(host, SDHC_CARD_CLKCTL, 0);
	ctr_sdhc_reg32_set(host, SDHC_ERROR_STATUS, 0);
	ctr_sdhc_reg16_set(host, SDHC_STOP_INTERNAL, 0);

	ctr_sdhc_reg16_set(host, SDHC_DATA16_BLK_CNT, 0);
	ctr_sdhc_reg16_set(host, SDHC_DATA16_BLK_LEN, 0);

	ctr_sdhc_reg16_set(host, SDHC_DATA32_BLK_CNT, 0);
	ctr_sdhc_reg16_set(host, SDHC_DATA32_BLK_LEN, 0);

	/* use the 16bit FIFO at all times */
	ctr_sdhc_reg16_set(host, SDHC_DATA_CTL, 0);
	ctr_sdhc_reg16_set(host, SDHC_DATA32_CTL, 0);

	/* set interrupt masks */
	ctr_sdhc_reg32_set(host, SDHC_IRQ_MASK, ~SDHC_IRQMASK);
	ctr_sdhc_reg32_set(host, SDHC_IRQ_STAT, 0);

	ctr_sdhc_reg16_set(host, SDHC_CARD_OPTION, SDHC_DEFAULT_CARDOPT);
}

static void ctr_sdhc_set_clk_opt(struct ctr_sdhc *host, u16 clk, u16 opt)
{
	ctr_sdhc_reg16_set(host, SDHC_CARD_CLKCTL, clk);
	ctr_sdhc_reg16_set(host, SDHC_CARD_OPTION, opt);
}

static void ctr_sdhc_send_cmdarg(struct ctr_sdhc *host, u16 cmd, u32 arg)
{
	ctr_sdhc_reg32_set(host, SDHC_CMD_PARAM, arg);
	ctr_sdhc_reg16_set(host, SDHC_CMD, cmd);
}

static void ctr_sdhc_set_blk_len_cnt(struct ctr_sdhc *host, u16 len, u16 cnt)
{
	ctr_sdhc_reg16_set(host, SDHC_DATA16_BLK_LEN, len);
	ctr_sdhc_reg16_set(host, SDHC_DATA16_BLK_CNT, cnt);
}

static void ctr_sdhc_get_resp(struct ctr_sdhc *host, u32 *resp, unsigned n)
{
	int i;
	for (i = 0; i < n; i++)
		resp[i] = ctr_sdhc_reg32_get(host, SDHC_CMD_RESPONSE + (i * 4));
}

static void ctr_sdhc_stop_internal_set(struct ctr_sdhc *host, u16 val)
{
	ctr_sdhc_reg16_set(host, SDHC_STOP_INTERNAL, val);
}

static u32 ctr_sdhc_irqstat_get(struct ctr_sdhc *host)
{
	return ctr_sdhc_reg32_get(host, SDHC_IRQ_STAT);
}

static void ctr_sdhc_irqstat_ack(struct ctr_sdhc *host, u32 ack)
{
	ctr_sdhc_reg32_set(host, SDHC_IRQ_STAT, ~ack);
}

static void ctr_sdhc_irqmask_set(struct ctr_sdhc *host, u32 mask)
{
	ctr_sdhc_reg32_set(host, SDHC_IRQ_MASK, mask);
}

static int ctr_sdhc_sdioirq_test(struct ctr_sdhc *host)
{
	u16 state = ctr_sdhc_reg16_get(host, SDHC_CARD_IRQ_STAT);

	if (state & 1) {
		/* acknowledge the SDIO IRQ */
		ctr_sdhc_reg16_set(host, SDHC_CARD_IRQ_STAT, state & ~1);
		return 1;
	}
	return 0;
}

static void ctr_sdhc_sdioirq_set(struct ctr_sdhc *host, int enable)
{
	/* always acknowledge the card interrupts first */
	ctr_sdhc_reg16_set(host, SDHC_CARD_IRQ_STAT, 0);

	/* either disable all interrupts _except_ SDIO IRQ, or disable all */
	ctr_sdhc_reg16_set(host, SDHC_CARD_IRQ_MASK, enable ? ~1 : ~0);
}
