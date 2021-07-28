// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  ctr_prng.c
 *
 *  Copyright (C) 2021 Santiago Herrera
 */

#define DRIVER_NAME "3ds-prng"
#define pr_fmt(str) DRIVER_NAME ": " str

#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/hw_random.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include "../ctr_pxi.h"

struct ctr_prng {
	struct mutex lock;
	struct hwrng hwrng;

	struct device *dev;
	struct ctr_pxi_host *pxi;
};

#define to_ctr_prng(ptr)	container_of(ptr, struct ctr_prng, hwrng)

/* reads up to 32 bytes of data in one go */
#define CTR_PXI_READ_PRNG	CTR_PXI_CMD(CTR_PXI_READ, CTR_PXI_WORD, 0x11000)

static const u32 ctr_prng_get[] = {
	CTR_PXI_READ_PRNG, CTR_PXI_READ_PRNG,
	CTR_PXI_READ_PRNG, CTR_PXI_READ_PRNG,
	CTR_PXI_READ_PRNG, CTR_PXI_READ_PRNG,
	CTR_PXI_READ_PRNG, CTR_PXI_READ_PRNG
};

#define CTR_PXI_CMD_PRNG_BLKS	ARRAY_SIZE(ctr_prng_get)

static int ctr_prng_read(struct hwrng *rng, void *buf, size_t size, bool wait)
{
	int err;
	u32 *buf32;
	size_t rem_blocks, total_blks;
	struct ctr_prng *prng = to_ctr_prng(rng);

	/*
	 * code says buf is always 4 byte aligned
	 * and size is >= 32, and a multiple of 4
	 */
	buf32 = (u32*)buf;
	rem_blocks = size / 4;
	total_blks = 0;

	/* TODO: implement non blocking ops */
	if (!wait)
		return 0;

	/* this lock is only used for cleanup, really */
	mutex_lock(&prng->lock);
	do {
		size_t blk_count = min(CTR_PXI_CMD_PRNG_BLKS, rem_blocks);

		err = ctr_pxi_cmd(prng->pxi, ctr_prng_get, buf32, blk_count);
		if (err) break;

		buf32 += blk_count;
		rem_blocks -= blk_count;
		total_blks += blk_count;
	} while(rem_blocks);
	mutex_unlock(&prng->lock);

	if (err < 0)
		return err;

	return total_blks * 4;
}

static void ctr_prng_cleanup(struct hwrng *rng)
{
	struct ctr_prng *prng = to_ctr_prng(rng);
	mutex_lock(&prng->lock);
	mutex_unlock(&prng->lock);
}

static int ctr_prng_probe(struct platform_device *pdev)
{
	struct ctr_prng *prng;
	struct device *dev = &pdev->dev;

	prng = devm_kzalloc(dev, sizeof(*prng), GFP_KERNEL);
	if (!prng)
		return -ENOMEM;

	mutex_init(&prng->lock);
	prng->dev = dev;

	prng->hwrng.name = dev_name(dev);
	prng->hwrng.read = ctr_prng_read;
	prng->hwrng.cleanup = ctr_prng_cleanup;
	prng->hwrng.quality = 0;

	prng->pxi = ctr_pxi_host_get(dev);
	if (IS_ERR(prng->pxi)) {
		pr_err("failed to get pxi host: %d\n", (int)PTR_ERR(prng->pxi));
		return PTR_ERR(prng->pxi);
	}

	pr_err("got pxi host @ %px\n", prng->pxi);

	return devm_hwrng_register(dev, &prng->hwrng);
}

static int ctr_prng_remove(struct platform_device *pdev)
{
	return -EINVAL;
}

static const struct of_device_id ctr_prng_of_match[] = {
	{ .compatible = "nintendo," DRIVER_NAME },
	{},
};
MODULE_DEVICE_TABLE(of, ctr_prng_of_match);

static struct platform_driver ctr_prng_driver = {
	.probe	= ctr_prng_probe,
	.remove = ctr_prng_remove,

	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(ctr_prng_of_match),
	},
};
module_platform_driver(ctr_prng_driver);

MODULE_AUTHOR("Santiago Herrera");
MODULE_DESCRIPTION("Nintendo 3DS PXI virtio bridge");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform: " DRIVER_NAME);

