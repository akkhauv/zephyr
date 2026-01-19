/*
 * Copyright (c) 2021 Marc Reilly - Creative Product Design
 * Copyright (c) 2026 Amber Khauv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * @file
 * @brief Private API for SPI bitbang drivers
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include "spi_context.h"

struct spi_bitbang_data;
struct spi_bitbang_config;

/* 
 * Bitbang clock edge callbacks used by the transfer engine, generalized to support both
 * master and slave driver code.
 */
struct spi_bitbang_ops {
	void (*pre_clock)(struct spi_bitbang_data *, const struct spi_bitbang_config *);
	void (*clock_active)(struct spi_bitbang_data *, const struct spi_bitbang_config *);
	void (*post_clock)(struct spi_bitbang_data *, const struct spi_bitbang_config *);
};

/* Per-transfer SPI mode and wiring state */
struct spi_bitbang_xfer {
	const struct gpio_dt_spec *data_out;
	const struct gpio_dt_spec *data_in;
	int clock_state; /* CPOL */
	int cpha;
	bool loop;
	bool lsb;
};

struct spi_bitbang_data {
	struct spi_context ctx;
	struct spi_bitbang_xfer xfer; 
	int bits;
	int wait_us;
	int dfs;
};

struct spi_bitbang_config {
	struct gpio_dt_spec clk_gpio;
	struct gpio_dt_spec mosi_gpio;
	struct gpio_dt_spec miso_gpio;
	const struct spi_bitbang_ops *bitbang_ops;
};

static inline int spi_bitbang_compute_dfs(const struct spi_config *config,
					  struct spi_bitbang_data *data)
{
    const int bits = SPI_WORD_SIZE_GET(config->operation);

	if (bits > 32) {
		return -ENOTSUP;
	}

	data->bits = bits;
	data->dfs = ((data->bits - 1) / 8) + 1;

	/* As there is no uint24_t, it is assumed uint32_t will be used as the buffer base type. */
	if (data->dfs == 3) {
		data->dfs = 4;
	}

    return 0;
}

static inline void spi_bitbang_xfer_setup(const struct spi_config *config,
					  struct spi_bitbang_data *data)
{
	data->xfer.clock_state = 0;
	data->xfer.cpha = 0;
	data->xfer.loop = false;
	data->xfer.lsb = false;

	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPOL) {
		data->xfer.clock_state = 1;
	}
	if (SPI_MODE_GET(config->operation) & SPI_MODE_CPHA) {
		data->xfer.cpha = 1;
	}
	if (SPI_MODE_GET(config->operation) & SPI_MODE_LOOP) {
		data->xfer.loop = true;
	}
	if (config->operation & SPI_TRANSFER_LSB) {
		data->xfer.lsb = true;
	}
}

/*
 * This function implements a blocking SPI transfer and is independent of the actual 
 * clock edge timing, which is provided by the spi_bitbang_ops callbacks.
 */
static inline void spi_bitbang_transfer(const struct spi_bitbang_config *info,
					struct spi_bitbang_data *data)
{
	struct spi_context *ctx = &data->ctx;
	struct spi_bitbang_xfer *xfer = &data->xfer;
	const struct spi_bitbang_ops *bitbang_ops = info->bitbang_ops;

	while (spi_context_tx_buf_on(ctx) || spi_context_rx_buf_on(ctx)) {
		uint32_t w = 0;

		if (ctx->tx_len) {
			switch (data->dfs) {
			case 4:
			case 3:
				w = *(uint32_t *)(ctx->tx_buf);
				break;
			case 2:
				w = *(uint16_t *)(ctx->tx_buf);
				break;
			case 1:
				w = *(uint8_t *)(ctx->tx_buf);
				break;
			}
		}

		uint32_t r = 0;
		uint8_t i = 0;
		int b = 0;
		bool do_read = false;

		if (xfer->data_in && spi_context_rx_buf_on(ctx)) {
			do_read = true;
		}

		while (i < data->bits) {
			const int shift = xfer->lsb ? i : (data->bits - 1 - i);
			const int d = (w >> shift) & 0x1;

			b = 0;

			/* setup data out first thing */
			if (xfer->data_out) {
				gpio_pin_set_dt(xfer->data_out, d);
			}

			bitbang_ops->pre_clock(data, info);

			if (!xfer->loop && do_read && !xfer->cpha) {
				b = gpio_pin_get_dt(xfer->data_in);
			}

			/* first (leading) clock edge */
			bitbang_ops->clock_active(data, info);

			if (!xfer->loop && do_read && xfer->cpha) {
				b = gpio_pin_get_dt(xfer->data_in);
			}

			/* second (trailing) clock edge */
			bitbang_ops->post_clock(data, info);

			if (xfer->loop) {
				b = d;
			}

			r |= (b ? 0x1 : 0x0) << shift;

			++i;
		}

		if (spi_context_rx_buf_on(ctx)) {
			switch (data->dfs) {
			case 4:
			case 3:
				*(uint32_t *)(ctx->rx_buf) = r;
				break;
			case 2:
				*(uint16_t *)(ctx->rx_buf) = r;
				break;
			case 1:
				*(uint8_t *)(ctx->rx_buf) = r;
				break;
			}
		}

		spi_context_update_tx(ctx, data->dfs, 1);
		spi_context_update_rx(ctx, data->dfs, 1);
	}
}
