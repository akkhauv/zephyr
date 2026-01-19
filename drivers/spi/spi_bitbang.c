/*
 * Copyright (c) 2021 Marc Reilly - Creative Product Design
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_spi_bitbang

#define LOG_LEVEL CONFIG_SPI_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(spi_bitbang);

#include <zephyr/sys/sys_io.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/spi/rtio.h>
#include "spi_context.h"
#include "spi_bitbang_common.h"

static void clock_delay(struct spi_bitbang_data *data, const struct spi_bitbang_config *info) 
{
	k_busy_wait(data->wait_us);
}

static void clock_assert_and_wait(struct spi_bitbang_data *data,
				  const struct spi_bitbang_config *info)
{
	gpio_pin_set_dt(&info->clk_gpio, !data->xfer.clock_state);
	clock_delay(data, info);
}

static void clock_deassert(struct spi_bitbang_data *data, const struct spi_bitbang_config *info)
{
	gpio_pin_set_dt(&info->clk_gpio, data->xfer.clock_state);
}

static int spi_bitbang_configure(const struct spi_bitbang_config *info,
			    struct spi_bitbang_data *data,
			    const struct spi_config *config)
{
	if (config->operation & SPI_OP_MODE_SLAVE) {
		LOG_ERR("Slave mode not supported");
		return -ENOTSUP;
	}

	if (config->operation & (SPI_LINES_DUAL | SPI_LINES_QUAD | SPI_LINES_OCTAL)) {
		LOG_ERR("Unsupported configuration");
		return -ENOTSUP;
	}

	spi_bitbang_compute_dfs(config, data);

	if (config->frequency > 0) {
		/* convert freq to period, the extra /2 is due to waiting
		 * twice in each clock cycle. The '2000' is an upscale factor.
		 */
		data->wait_us = (1000000ul * 2000ul / config->frequency) / 2000ul;
		data->wait_us /= 2;
	} else {
		data->wait_us = 8 / 2; /* 125 kHz */
	}

	data->ctx.config = config;

	return 0;
}

static int spi_bitbang_transceive(const struct device *dev,
			      const struct spi_config *spi_cfg,
			      const struct spi_buf_set *tx_bufs,
			      const struct spi_buf_set *rx_bufs)
{
	const struct spi_bitbang_config *info = dev->config;
	struct spi_bitbang_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	struct spi_bitbang_xfer *xfer = &data->xfer;
	int rc;
	gpio_flags_t mosi_flags = GPIO_OUTPUT_INACTIVE;

	rc = spi_bitbang_configure(info, data, spi_cfg);
	if (rc < 0) {
		return rc;
	}

	xfer->data_in = NULL;
	xfer->data_out = NULL;

	if (spi_cfg->operation & SPI_HALF_DUPLEX) {
		if (!info->mosi_gpio.port) {
			LOG_ERR("No MOSI pin specified in half duplex mode");
			return -EINVAL;
		}

		if (tx_bufs && rx_bufs) {
			LOG_ERR("Both RX and TX specified in half duplex mode");
			return -EINVAL;
		} else if (tx_bufs && !rx_bufs) {
			/* TX mode */
			xfer->data_out = &info->mosi_gpio;
		} else if (!tx_bufs && rx_bufs) {
			/* RX mode */
			mosi_flags = GPIO_INPUT;
			xfer->data_in = &info->mosi_gpio;
		}
	} else {
		if (info->mosi_gpio.port) {
			xfer->data_out = &info->mosi_gpio;
		}

		if (info->miso_gpio.port) {
			xfer->data_in = &info->miso_gpio;
		}
	}

	if (info->mosi_gpio.port) {
		rc = gpio_pin_configure_dt(&info->mosi_gpio, mosi_flags);
		if (rc < 0) {
			LOG_ERR("Couldn't configure MOSI pin: %d", rc);
			return rc;
		}
	}

	spi_bitbang_xfer_setup(spi_cfg, data);
	spi_context_buffers_setup(ctx, tx_bufs, rx_bufs, data->dfs);

	/* set the initial clock state before CS */
	gpio_pin_set_dt(&info->clk_gpio, xfer->clock_state);

	spi_context_cs_control(ctx, true);
	spi_bitbang_transfer(info, data);
	spi_context_cs_control(ctx, false);

	spi_context_complete(ctx, dev, 0);

	return 0;
}

#ifdef CONFIG_SPI_ASYNC
static int spi_bitbang_transceive_async(const struct device *dev,
				    const struct spi_config *spi_cfg,
				    const struct spi_buf_set *tx_bufs,
				    const struct spi_buf_set *rx_bufs,
				    spi_callback_t cb,
				    void *userdata)
{
	return -ENOTSUP;
}
#endif

int spi_bitbang_release(const struct device *dev,
			  const struct spi_config *config)
{
	struct spi_bitbang_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;

	spi_context_unlock_unconditionally(ctx);
	return 0;
}

static DEVICE_API(spi, spi_bitbang_api) = {
	.transceive = spi_bitbang_transceive,
	.release = spi_bitbang_release,
#ifdef CONFIG_SPI_ASYNC
	.transceive_async = spi_bitbang_transceive_async,
#endif /* CONFIG_SPI_ASYNC */
#ifdef CONFIG_SPI_RTIO
	.iodev_submit = spi_rtio_iodev_default_submit,
#endif
};

int spi_bitbang_init(const struct device *dev)
{
	const struct spi_bitbang_config *config = dev->config;
	struct spi_bitbang_data *data = dev->data;
	int rc;

	if (!gpio_is_ready_dt(&config->clk_gpio)) {
		LOG_ERR("GPIO port for clk pin is not ready");
		return -ENODEV;
	}
	rc = gpio_pin_configure_dt(&config->clk_gpio, GPIO_OUTPUT_INACTIVE);
	if (rc < 0) {
		LOG_ERR("Couldn't configure clk pin; (%d)", rc);
		return rc;
	}

	if (config->mosi_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->mosi_gpio)) {
			LOG_ERR("GPIO port for mosi pin is not ready");
			return -ENODEV;
		}
		rc = gpio_pin_configure_dt(&config->mosi_gpio,
				GPIO_OUTPUT_INACTIVE);
		if (rc < 0) {
			LOG_ERR("Couldn't configure mosi pin; (%d)", rc);
			return rc;
		}
	}

	if (config->miso_gpio.port != NULL) {
		if (!gpio_is_ready_dt(&config->miso_gpio)) {
			LOG_ERR("GPIO port for miso pin is not ready");
			return -ENODEV;
		}


		rc = gpio_pin_configure_dt(&config->miso_gpio, GPIO_INPUT);
		if (rc < 0) {
			LOG_ERR("Couldn't configure miso pin; (%d)", rc);
			return rc;
		}
	}

	rc = spi_context_cs_configure_all(&data->ctx);
	if (rc < 0) {
		LOG_ERR("Failed to configure CS pins: %d", rc);
		return rc;
	}

	return 0;
}

static const struct spi_bitbang_ops master_bitbang_ops = {
    .pre_clock    = clock_delay,
    .clock_active = clock_assert_and_wait,
    .post_clock   = clock_deassert
};

#define SPI_BITBANG_INIT(inst)						\
	static struct spi_bitbang_config spi_bitbang_config_##inst = {	\
		.clk_gpio = GPIO_DT_SPEC_INST_GET(inst, clk_gpios),	\
		.mosi_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, mosi_gpios, {0}),	\
		.miso_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, miso_gpios, {0}),	\
		.bitbang_ops = &master_bitbang_ops,\
	};								\
									\
	static struct spi_bitbang_data spi_bitbang_data_##inst = {	\
		SPI_CONTEXT_INIT_LOCK(spi_bitbang_data_##inst, ctx),	\
		SPI_CONTEXT_INIT_SYNC(spi_bitbang_data_##inst, ctx),	\
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(inst), ctx)	\
	};								\
									\
	SPI_DEVICE_DT_INST_DEFINE(inst,					\
			    spi_bitbang_init,				\
			    NULL,					\
			    &spi_bitbang_data_##inst,			\
			    &spi_bitbang_config_##inst,			\
			    POST_KERNEL,				\
			    CONFIG_SPI_INIT_PRIORITY,			\
			    &spi_bitbang_api);

DT_INST_FOREACH_STATUS_OKAY(SPI_BITBANG_INIT)
