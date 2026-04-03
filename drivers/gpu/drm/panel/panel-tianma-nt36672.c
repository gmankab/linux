// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 FIXME
// Generated with linux-mdss-dsi-panel-driver-generator from vendor device tree:
//   Copyright (c) 2013, The Linux Foundation. All rights reserved. (FIXME)

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/display/drm_dsc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct nt36672_tianma {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct drm_dsc_config dsc;
	struct gpio_desc *reset_gpio;
};

static inline
struct nt36672_tianma *to_nt36672_tianma(struct drm_panel *panel)
{
	return container_of_const(panel, struct nt36672_tianma, panel);
}

static void nt36672_tianma_reset(struct nt36672_tianma *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int nt36672_tianma_on(struct nt36672_tianma *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4d, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4e, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4f, 0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x30);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7a, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7b, 0x8c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x7d, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x80, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x81, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa0, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_READ_PPS_START, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa3, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa4, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa5, 0x05);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc4, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc6, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe9, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xda, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf1, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x2b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb8, 0x1a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xf0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x1c, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x23);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x00, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x01, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x05, 0x2d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x06, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x07, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x09, 0x45);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x12, 0x95);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x15, 0x68);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x16, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x29, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_PARTIAL_ROWS, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_PARTIAL_COLUMNS,
				     0xfe);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x32, 0xfd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x33, 0xfb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x34, 0xf8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0xf5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_ADDRESS_MODE, 0xf3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x37, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x38, 0xf2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x39, 0xf2);
	mipi_dsi_dcs_set_pixel_format_multi(&dsi_ctx, 0xef);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0xec);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_3D_CONTROL, 0xe9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3f, 0xe5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_VSYNC_TIMING, 0xe5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x41, 0xe5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_GET_SCANLINE, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x46, 0xf4);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x47, 0xe7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x48, 0xda);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x49, 0xcd);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4a, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4b, 0xb3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4c, 0xb2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4d, 0xb2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4e, 0xb2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x4f, 0x99);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x50, 0x80);
	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0x0068);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x52, 0x66);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x66);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x54, 0x66);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2b, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x58, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x59, 0xfb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5a, 0xf7);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5b, 0xf3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5c, 0xef);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5d, 0xe3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_CABC_MIN_BRIGHTNESS,
				     0xda);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5f, 0xd8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x60, 0xd8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x61, 0xd8);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x62, 0xcb);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x63, 0xbf);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x64, 0xb3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x65, 0xb2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x66, 0xb2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x67, 0xb2);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_VSYNC_TIMING, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x03);
	mipi_dsi_dcs_set_display_brightness_multi(&dsi_ctx, 0xb50d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY,
				     0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11, 0x00);
	mipi_dsi_msleep(&dsi_ctx, 70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x29, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x27);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfb, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3f, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x43, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_VSYNC_TIMING, 0x25);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0x10);

	return dsi_ctx.accum_err;
}

static int nt36672_tianma_prepare(struct drm_panel *panel)
{
	struct nt36672_tianma *ctx = to_nt36672_tianma(panel);
	struct device *dev = &ctx->dsi->dev;
	struct drm_dsc_picture_parameter_set pps;
	int ret;

	nt36672_tianma_reset(ctx);

	ret = nt36672_tianma_on(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		return ret;
	}

	drm_dsc_pps_payload_pack(&pps, &ctx->dsc);

	ret = mipi_dsi_picture_parameter_set(ctx->dsi, &pps);
	if (ret < 0) {
		dev_err(panel->dev, "failed to transmit PPS: %d\n", ret);
		return ret;
	}

	ret = mipi_dsi_compression_mode(ctx->dsi, true);
	if (ret < 0) {
		dev_err(dev, "failed to enable compression mode: %d\n", ret);
		return ret;
	}

	msleep(28); /* TODO: Is this panel-dependent? */

	return 0;
}

static int nt36672_tianma_unprepare(struct drm_panel *panel)
{
	struct nt36672_tianma *ctx = to_nt36672_tianma(panel);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	return 0;
}

static const struct drm_display_mode nt36672_tianma_mode = {
	.clock = (1080 + 67 + 12 + 40) * (2400 + 33 + 2 + 30) * 120 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 67,
	.hsync_end = 1080 + 67 + 12,
	.htotal = 1080 + 67 + 12 + 40,
	.vdisplay = 2400,
	.vsync_start = 2400 + 33,
	.vsync_end = 2400 + 33 + 2,
	.vtotal = 2400 + 33 + 2 + 30,
	.width_mm = 70,
	.height_mm = 154,
	.type = DRM_MODE_TYPE_DRIVER,
};

static int nt36672_tianma_get_modes(struct drm_panel *panel,
				       struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &nt36672_tianma_mode);
}

static const struct drm_panel_funcs nt36672_tianma_panel_funcs = {
	.prepare = nt36672_tianma_prepare,
	.unprepare = nt36672_tianma_unprepare,
	.get_modes = nt36672_tianma_get_modes,
};

static int nt36672_tianma_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct nt36672_tianma *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct nt36672_tianma, panel,
				   &nt36672_tianma_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset-gpios\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS | MIPI_DSI_MODE_LPM;

	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	/* This panel only supports DSC; unconditionally enable it */
	dsi->dsc = &ctx->dsc;

	ctx->dsc.dsc_version_major = 1;
	ctx->dsc.dsc_version_minor = 1;

	/* TODO: Pass slice_per_pkt = 2 */
	ctx->dsc.slice_height = 20;
	ctx->dsc.slice_width = 540;
	/*
	 * TODO: hdisplay should be read from the selected mode once
	 * it is passed back to drm_panel (in prepare?)
	 */
	WARN_ON(1080 % ctx->dsc.slice_width);
	ctx->dsc.slice_count = 1080 / ctx->dsc.slice_width;
	ctx->dsc.bits_per_component = 8;
	ctx->dsc.bits_per_pixel = 8 << 4; /* 4 fractional bits */
	ctx->dsc.block_pred_enable = true;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void nt36672_tianma_remove(struct mipi_dsi_device *dsi)
{
	struct nt36672_tianma *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id nt36672_tianma_of_match[] = {
	{ .compatible = "tianma,nt36672" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nt36672_tianma_of_match);

static struct mipi_dsi_driver nt36672_tianma_driver = {
	.probe = nt36672_tianma_probe,
	.remove = nt36672_tianma_remove,
	.driver = {
		.name = "panel-nt36672-tianma",
		.of_match_table = nt36672_tianma_of_match,
	},
};
module_mipi_dsi_driver(nt36672_tianma_driver);

MODULE_AUTHOR("linux-mdss-dsi-panel-driver-generator <fix@me>"); // FIXME
MODULE_DESCRIPTION("DRM driver for xiaomi 36 02 0a video mode dsc dsi panel");
MODULE_LICENSE("GPL");
