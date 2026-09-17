// SPDX-License-Identifier: GPL-2.0
/*
 * Registers an ADXL345 on CS0 of the first dln2-spi controller and binds it
 * to spidev, so it shows up as /dev/spidevN.0.
 *
 * A USB adapter has no firmware tables describing what is wired to it, and
 * spi has no sysfs "new_device", hence this module.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>

static uint speed_hz = 1000000;
module_param(speed_hz, uint, 0444);
MODULE_PARM_DESC(speed_hz, "Max SPI clock rate (ADXL345 allows up to 5 MHz)");

static struct spi_device *adxl;

static int match_dln2_spi(struct device *dev, const void *data)
{
	return !strncmp(dev_name(dev), "dln2-spi", 8);
}

static int __init dln2_adxl345_init(void)
{
	struct spi_controller *ctlr;
	struct device *pdev;
	int ret, i;

	pdev = bus_find_device(&platform_bus_type, NULL, NULL, match_dln2_spi);
	if (!pdev)
		return -ENODEV;

	/* spi-dln2 stores its controller as the platform drvdata. */
	ctlr = platform_get_drvdata(to_platform_device(pdev));
	if (!ctlr) {
		ret = -ENODEV;
		goto out_put_pdev;
	}

	adxl = spi_alloc_device(ctlr);
	if (!adxl) {
		ret = -ENOMEM;
		goto out_put_pdev;
	}

	adxl->max_speed_hz = speed_hz;
	adxl->mode = SPI_MODE_3;
	adxl->bits_per_word = 8;
	/* Unused chip-select slots must be 0xFF, otherwise the core sees the
	 * zeroed slots as duplicates of CS0 (as spi_new_device() does). */
	for (i = 0; i < SPI_CS_CNT_MAX; i++)
		spi_set_chipselect(adxl, i, 0xFF);
	spi_set_chipselect(adxl, 0, 0);
	adxl->cs_index_mask = BIT(0);
	strscpy(adxl->modalias, "adxl345", sizeof(adxl->modalias));

	ret = driver_set_override(&adxl->dev, &adxl->driver_override,
				  "spidev", strlen("spidev"));
	if (ret)
		goto out_put_spi;

	ret = spi_add_device(adxl);
	if (ret)
		goto out_put_spi;

	/* Keep our own reference: unplugging the adapter unregisters the
	 * device behind our back. */
	get_device(&adxl->dev);
	put_device(pdev);
	return 0;

out_put_spi:
	spi_dev_put(adxl);
out_put_pdev:
	put_device(pdev);
	return ret;
}

static void __exit dln2_adxl345_exit(void)
{
	if (device_is_registered(&adxl->dev))
		spi_unregister_device(adxl);
	put_device(&adxl->dev);
}

module_init(dln2_adxl345_init);
module_exit(dln2_adxl345_exit);

MODULE_DESCRIPTION("ADXL345 spidev instance on a DLN-2 SPI adapter");
MODULE_LICENSE("GPL");
