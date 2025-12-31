/*
 * Copyright (c) 2020-2025 Espressif Systems (Shanghai) Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT espressif_esp32_spis
#include <zephyr/logging/log_ctrl.h>

// TODO: DELETE
#include <zephyr/sys/printk.h>

/* Include esp-idf headers first to avoid redefining BIT() macro */
#include <hal/spi_slave_hal.h>
#include <hal/spi_hal.h>
#include <esp_attr.h>
#include <esp_clk_tree.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(esp32_spi, CONFIG_SPI_LOG_LEVEL);

#include <soc.h>
#include <esp_memory_utils.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>
#ifdef SOC_GDMA_SUPPORTED
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_esp32.h>
#endif
#include <zephyr/drivers/clock_control.h>
#include "test_spi_context.h"
#include "spi_esp32_spis.h"

#define SPI_DMA_MAX_BUFFER_SIZE 4092

#define SPI_DMA_RX 0
#define SPI_DMA_TX 1

/*
 * TODO: figure out common/duplicate spim code situation
 */
static inline uint8_t spi_esp32_get_frame_size(const struct spi_config *spi_cfg)
{
	uint8_t dfs = SPI_WORD_SIZE_GET(spi_cfg->operation);

	dfs /= 8;
	if ((dfs == 0) || (dfs > 4)) {
		LOG_WRN("Unsupported dfs, 1-byte size will be used");
		dfs = 1;
	}
	return dfs;
}

static int IRAM_ATTR spis_esp32_configure(const struct device *dev,
					  const struct spi_config *spi_cfg)
{
	const struct spis_esp32_config *cfg = dev->config;
	struct spis_esp32_data *data = dev->data;
	struct spi_context *ctx = &data->ctx;
	spi_slave_hal_context_t *hal = &data->hal;
	int freq;

	if (spi_context_configured(ctx, spi_cfg)) {
		return 0;
	}

	ctx->config = spi_cfg;

	if ((spi_cfg->operation & SPI_LINES_MASK) != SPI_LINES_SINGLE) {
		LOG_ERR("Line mode not supported");
		return -ENOTSUP;
	}

	if (spi_cfg->operation & SPI_HALF_DUPLEX) {
		LOG_ERR("Half-duplex not supported");
		return -ENOTSUP;
	}

	if (spi_cfg->operation & SPI_MODE_LOOP) {
		LOG_ERR("Loopback mode is not supported");
		return -ENOTSUP;
	}

	int ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret) {
		LOG_ERR("Failed to configure SPI pins");
		return ret;
	}

	hal->tx_lsbfirst = spi_cfg->operation & SPI_TRANSFER_LSB ? 1 : 0;
	hal->rx_lsbfirst = spi_cfg->operation & SPI_TRANSFER_LSB ? 1 : 0;

	/* SPI mode */
	hal->mode = 0;
	if (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPHA) {
		hal->mode = BIT(0);
	}
	if (SPI_MODE_GET(spi_cfg->operation) & SPI_MODE_CPOL) {
		hal->mode |= BIT(1);
	}

	spi_slave_hal_setup_device(hal);

	/* Workaround to handle default state of MISO and MOSI lines */
#ifndef CONFIG_SOC_SERIES_ESP32
	spi_dev_t *hw = hal->hw;

	if (cfg->line_idle_low) {
		hw->ctrl.d_pol = 0;
		hw->ctrl.q_pol = 0;
	} else {
		hw->ctrl.d_pol = 1;
		hw->ctrl.q_pol = 1;
	}
#endif

	/*
	 * Workaround for ESP32S3 and ESP32Cx SoC's. This dummy transaction is needed
	 * to sync CLK and software controlled CS when SPI is in mode 3
	 */
#if (defined(CONFIG_SOC_SERIES_ESP32S3) || defined(CONFIG_SOC_SERIES_ESP32C2) ||                   \
     defined(CONFIG_SOC_SERIES_ESP32C3) || defined(CONFIG_SOC_SERIES_ESP32C6)) &&                  \
	!defined(DT_SPI_CTX_HAS_NO_CS_GPIOS)
	if ((ctx->num_cs_gpios != 0) && (hal->mode & (SPI_MODE_CPOL | SPI_MODE_CPHA))) {
		return -ENOTSUP;
	}
#endif

	return 0;
}

static int IRAM_ATTR spis_esp32_transfer(const struct device *dev)
{
	struct spis_esp32_data *data = dev->data;
	const struct spis_esp32_config *cfg = dev->config;
	struct spi_context *ctx = &data->ctx;
	spi_slave_hal_context_t *hal = &data->hal;

	int err = 0;

	if (cfg->dma_enabled) {
		return -ENOTSUP;
	}

	/* clean up and prepare SPI hal */
	for (size_t i = 0; i < ARRAY_SIZE(hal->hw->data_buf); ++i) {
#if defined(CONFIG_SOC_SERIES_ESP32C6) || defined(CONFIG_SOC_SERIES_ESP32H2)
		hal->hw->data_buf[i].val = 0;
#else
		hal->hw->data_buf[i] = 0;
#endif
	}

	hal->tx_buffer = (uint8_t *)ctx->tx_buf;
	hal->rx_buffer = ctx->rx_buf;
	hal->bitlen = 32;

	LOG_ERR("rx at %p | tx at %p, bitlen is %u", hal->rx_buffer, hal->tx_buffer, hal->bitlen);

	spi_slave_hal_prepare_data(hal);
	hal->hw->slave1.data_bitlen = 32;   // 4 bytes * 8 bits
	hal->hw->slave.rdbuf_bitlen_en = 1; // enable correct read length
	hal->hw->slave.wrbuf_bitlen_en = 1; // enable correct write length
	hal->hw->user.usr_miso = 1;         // enable read phase
	hal->hw->user.usr_mosi = 1;         // enable write phase
	hal->hw->slave.usr_conf = 1;        // start transfer

	LOG_ERR("STARTING");
	spi_slave_hal_user_start(hal);

	while (!spi_slave_hal_usr_is_done(hal)) {
		/* nop */
		// LOG_ERR("bitlen %u", spi_ll_slave_get_rcv_bitlen(hal->hw));
	}
	// LOG_ERR("AFTER BLOCK received %u", spi_ll_slave_get_rcv_bitlen(hal->hw));

	return err;
}

static int spis_transceive(const struct device *dev, const struct spi_config *spi_cfg,
			   const struct spi_buf_set *tx_bufs, const struct spi_buf_set *rx_bufs,
			   bool asynchronous, spi_callback_t cb, void *userdata)
{
	const struct spis_esp32_config *cfg = dev->config;
	struct spis_esp32_data *data = dev->data;
	int ret = 0;

#ifndef CONFIG_SPI_ESP32_INTERRUPT
	if (asynchronous) {
		return -ENOTSUP;
	}
#endif

	spi_context_lock(&data->ctx, asynchronous, cb, userdata, spi_cfg);

	data->dfs = spi_esp32_get_frame_size(spi_cfg);

	LOG_ERR("Frame size has been detected as %u\n", data->dfs);

	// sets up tx and rx in ctx
	spi_context_buffers_setup(&data->ctx, tx_bufs, rx_bufs, data->dfs);

	if (data->ctx.tx_buf == NULL && data->ctx.rx_buf == NULL) {
		goto done;
	}

	ret = spis_esp32_configure(dev, spi_cfg);
	if (ret) {
		goto done;
	}

#ifdef CONFIG_SPI_ESP32_INTERRUPT
	return -ENOTSUP;
#else

	spis_esp32_transfer(dev);

#endif

	spi_slave_hal_store_result(&data->hal);
	LOG_ERR("bitlen is %u", data->hal.rcv_bitlen);

	// #endif /* CONFIG_SPI_ESP32_INTERRUPT */

	printk("hello world!\n");

done:
	spi_context_release(&data->ctx, ret);

	return ret;
}

static int spis_esp32_transceive(const struct device *dev, const struct spi_config *spi_cfg,
				 const struct spi_buf_set *tx_bufs,
				 const struct spi_buf_set *rx_bufs)
{
	return spis_transceive(dev, spi_cfg, tx_bufs, rx_bufs, false, NULL, NULL);
}

static int spis_esp32_init(const struct device *dev)
{
	int err;
	const struct spis_esp32_config *cfg = dev->config;
	struct spis_esp32_data *data = dev->data;
	spi_slave_hal_context_t *hal = &data->hal;

	if (!cfg->clock_dev) {
		return -EINVAL;
	}

	if (!device_is_ready(cfg->clock_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Enables SPI peripheral */
	err = clock_control_on(cfg->clock_dev, cfg->clock_subsys);
	if (err < 0) {
		LOG_ERR("Error enabling SPI clock");
		return err;
	}

	spi_ll_slave_init(hal->hw);

	if (cfg->dma_enabled) {
		LOG_ERR("DMA on slave not supported");
		return -ENOTSUP;
	}

#ifdef CONFIG_SPI_ESP32_INTERRUPT
	LOG_ERR("Interrupts on slave not supported");
	return -ENOTSUP;
#endif

	spi_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static DEVICE_API(spi, spi_api) = {.transceive = spis_esp32_transceive,
#ifdef CONFIG_SPI_ASYNC
				   .transceive_async = NULL,
#endif
#ifdef CONFIG_SPI_RTIO
				   .iodev_submit = NULL,
#endif
				   .release = NULL};

#ifdef CONFIG_SOC_SERIES_ESP32
#define GET_AS_CS(idx) .as_cs = DT_INST_PROP(idx, clk_as_cs),
#else
#define GET_AS_CS(idx)
#endif

#if defined(SOC_GDMA_SUPPORTED)
#define SPI_DMA_CFG(idx)                                                                           \
	.dma_dev = ESP32_DT_INST_DMA_CTLR(idx, tx),                                                \
	.dma_tx_ch = ESP32_DT_INST_DMA_CELL(idx, tx, channel),                                     \
	.dma_rx_ch = ESP32_DT_INST_DMA_CELL(idx, rx, channel)
#else
#define SPI_DMA_CFG(idx) .dma_clk_src = DT_INST_PROP(idx, dma_clk)
#endif /* defined(SOC_GDMA_SUPPORTED) */

#define ESP32_SPIS_INIT(idx)                                                                                                \
                                                                                                                            \
	PINCTRL_DT_INST_DEFINE(idx);                                                                                        \
                                                                                                                            \
	static struct spis_esp32_data spis_data_##idx = {                                                                   \
		SPI_CONTEXT_INIT_LOCK(spis_data_##idx, ctx),                                                                \
		SPI_CONTEXT_INIT_SYNC(spis_data_##idx, ctx),                                                                \
		SPI_CONTEXT_CS_GPIOS_INITIALIZE(DT_DRV_INST(idx), ctx).hal =                                                \
			{                                                                                                   \
				.hw = (spi_dev_t *)DT_INST_REG_ADDR(idx),                                                   \
				.dma_in = NULL,                                                                             \
				.dma_out = NULL, /* the following should be configured at                                   \
						    initialization */                                                       \
				.dmadesc_rx = NULL,                                                                         \
				.dmadesc_tx = NULL,                                                                         \
				.dmadesc_n = 0,                                                                             \
				.tx_dma_chan = 0,                                                                           \
				.rx_dma_chan = 0,                                                                           \
				/* the following to be filled after spi_slave_hal_init */ /* updated                        \
											     to                             \
											     peripheral                     \
											     registers                      \
											     whenspi_slave_hal_setup_device \
											     called                         \
											   */                               \
				.use_dma =                                                                                  \
					0, /* the following will be updated at each transaction*/                           \
				.bitlen = 0,                                                                                \
				.tx_buffer = NULL,                                                                          \
				.rx_buffer = NULL,                                                                          \
				.rcv_bitlen = 0,                                                                            \
			},                                                                                                  \
		.hal_config = {.host_id = DT_INST_PROP(idx, dma_host) + 1,                                                  \
			       .dma_in = NULL,                                                                              \
			       .dma_out = NULL}}; /* DMA_HOST + 1 was a pattern reflected in the                            \
						     master driver*/                                                        \
                                                                                                                            \
	static const struct spis_esp32_config spis_config_##idx = {                                                         \
		.spi = (spi_dev_t *)DT_INST_REG_ADDR(idx),                                                                  \
                                                                                                                            \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(idx)),                                                       \
		.duty_cycle = 0,                                                                                            \
		.input_delay_ns = 0,                                                                                        \
		.irq_source = DT_INST_IRQ_BY_IDX(idx, 0, irq),                                                              \
		.irq_priority = DT_INST_IRQ_BY_IDX(idx, 0, priority),                                                       \
		.irq_flags = DT_INST_IRQ_BY_IDX(idx, 0, flags),                                                             \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(idx),                                                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(idx, offset),                                   \
		.use_iomux = DT_INST_PROP(idx, use_iomux),                                                                  \
		.dma_enabled = DT_INST_PROP(idx, dma_enabled),                                                              \
		.dma_host = DT_INST_PROP(idx, dma_host),                                                                    \
		SPI_DMA_CFG(idx),                                                                                           \
		.cs_setup = DT_INST_PROP_OR(idx, cs_setup_time, 0),                                                         \
		.cs_hold = DT_INST_PROP_OR(idx, cs_hold_time, 0),                                                           \
		.line_idle_low = DT_INST_PROP(idx, line_idle_low),                                                          \
		.clock_source = SPI_CLK_SRC_DEFAULT,                                                                        \
	};                                                                                                                  \
                                                                                                                            \
	SPI_DEVICE_DT_INST_DEFINE(idx, spis_esp32_init, NULL, &spis_data_##idx,                                             \
				  &spis_config_##idx, POST_KERNEL, CONFIG_SPI_INIT_PRIORITY,                                \
				  &spi_api);

DT_INST_FOREACH_STATUS_OKAY(ESP32_SPIS_INIT)
