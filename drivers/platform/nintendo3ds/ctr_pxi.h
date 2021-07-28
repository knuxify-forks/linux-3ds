// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  ctr_pxi.h
 *
 *  Copyright (C) 2021 Santiago Herrera
 */

#ifndef CTR_PXI_H
#define CTR_PXI_H

struct ctr_pxi_host;

enum {
	CTR_PXI_WRITE = 0 << 28,
	CTR_PXI_READ = 1 << 28,

	CTR_PXI_IOSET = 2 << 28,
	CTR_PXI_IOCLR = 3 << 28,

	CTR_PXI_IRQGET = 4 << 28,
	CTR_PXI_IRQMASK = 5 << 28,
	CTR_PXI_IRQUNMASK = 6 << 28,
};

enum {
	CTR_PXI_BYTE = 0 << 24,
	CTR_PXI_HALF = 1 << 24,
	CTR_PXI_WORD = 2 << 24
};

#define CTR_PXI_1B	CTR_PXI_BYTE
#define CTR_PXI_2B	CTR_PXI_HALF
#define CTR_PXI_4B	CTR_PXI_WORD

/* make sure `reg` never exceeds 1MiB ! */
#define CTR_PXI_CMD(mode, size, reg)	(u32)((mode) | (size) | (reg))

struct ctr_pxi_host *ctr_pxi_host_get(struct device *dev);
void ctr_pxi_host_put(struct device *dev, struct ctr_pxi_host *pxi);

int ctr_pxi_cmd(struct ctr_pxi_host *pxi, const u32 *cmd, u32 *arg, u32 ncmds);

#endif /* CTR_PXI_H */
