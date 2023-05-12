// SPDX-License-Identifier: GPL-2.0-only
/*
 * MIPI-DSI based ak3b AMOLED LCD panel driver.
 *
 * Copyright (c) 2023 Google LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/of_platform.h>
#include <video/mipi_display.h>

#include "panel/panel-samsung-drv.h"

static const struct drm_dsc_config pps_config = {
	.line_buf_depth = 9,
	.bits_per_component = 8,
	.convert_rgb = true,
	.slice_width = 540,
	.slice_height = 48,
	.simple_422 = false,
	.pic_width = 1080,
	.pic_height = 2400,
	.rc_tgt_offset_high = 3,
	.rc_tgt_offset_low = 3,
	.bits_per_pixel = 128,
	.rc_edge_factor = 6,
	.rc_quant_incr_limit1 = 11,
	.rc_quant_incr_limit0 = 11,
	.initial_xmit_delay = 512,
	.initial_dec_delay = 526,
	.block_pred_enable = true,
	.first_line_bpg_offset = 12,
	.initial_offset = 6144,
	.rc_buf_thresh = {
		14, 28, 42, 56,
		70, 84, 98, 105,
		112, 119, 121, 123,
		125, 126
	},
	.rc_range_params = {
		{.range_min_qp = 0, .range_max_qp = 4, .range_bpg_offset = 2},
		{.range_min_qp = 0, .range_max_qp = 4, .range_bpg_offset = 0},
		{.range_min_qp = 1, .range_max_qp = 5, .range_bpg_offset = 0},
		{.range_min_qp = 1, .range_max_qp = 6, .range_bpg_offset = 62},
		{.range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 60},
		{.range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 58},
		{.range_min_qp = 3, .range_max_qp = 7, .range_bpg_offset = 56},
		{.range_min_qp = 3, .range_max_qp = 8, .range_bpg_offset = 56},
		{.range_min_qp = 3, .range_max_qp = 9, .range_bpg_offset = 56},
		{.range_min_qp = 3, .range_max_qp = 10, .range_bpg_offset = 54},
		{.range_min_qp = 5, .range_max_qp = 11, .range_bpg_offset = 54},
		{.range_min_qp = 5, .range_max_qp = 12, .range_bpg_offset = 52},
		{.range_min_qp = 5, .range_max_qp = 13, .range_bpg_offset = 52},
		{.range_min_qp = 7, .range_max_qp = 13, .range_bpg_offset = 52},
		{.range_min_qp = 13, .range_max_qp = 15, .range_bpg_offset = 52}
	},
	.rc_model_size = 8192,
	.flatness_min_qp = 3,
	.flatness_max_qp = 12,
	.initial_scale_value = 32,
	.scale_decrement_interval = 7,
	.scale_increment_interval = 1190,
	.nfl_bpg_offset = 523,
	.slice_bpg_offset = 543,
	.final_offset = 4336,
	.vbr_enable = false,
	.slice_chunk_size = 540,
	.dsc_version_minor = 1,
	.dsc_version_major = 1,
	.native_422 = false,
	.native_420 = false,
	.second_line_bpg_offset = 0,
	.nsl_bpg_offset = 0,
	.second_line_offset_adj = 0,
};

#define AK3B_WRCTRLD_DIMMING_BIT    0x08
#define AK3B_WRCTRLD_BCTRL_BIT      0x20
#define AK3B_WRCTRLD_HBM_BIT        0xC0
#define AK3B_WRCTRLD_LOCAL_HBM_BIT  0x10

#define LHBM_RGB_RATIO_SIZE 3

static const u8 test_key_on_f0[] = { 0xF0, 0x5A, 0x5A };
static const u8 test_key_off_f0[] = { 0xF0, 0xA5, 0xA5 };
static const u8 freq_update[] = { 0xF7, 0x0F };

static const struct exynos_dsi_cmd ak3b_off_cmds[] = {
	EXYNOS_DSI_CMD_SEQ(MIPI_DCS_SET_DISPLAY_OFF),
	EXYNOS_DSI_CMD_SEQ_DELAY(120, MIPI_DCS_ENTER_SLEEP_MODE), /* sleep in */
};
static DEFINE_EXYNOS_CMD_SET(ak3b_off);

static const struct exynos_dsi_cmd ak3b_lp_cmds[] = {
	EXYNOS_DSI_CMD_SEQ(MIPI_DCS_SET_DISPLAY_OFF),
	EXYNOS_DSI_CMD_SEQ(0xF0, 0x5A, 0x5A), /* test_key_on_f0 */
	EXYNOS_DSI_CMD_SEQ(0xB9, 0x30), /* TE_SELECT 60Hz */
	EXYNOS_DSI_CMD_SEQ(0xF7, 0x0F), /* freq_update */
	EXYNOS_DSI_CMD_SEQ(0xF0, 0xA5, 0xA5), /* test_key_off_f0 */
};
static DEFINE_EXYNOS_CMD_SET(ak3b_lp);

static const struct exynos_dsi_cmd ak3b_lp_off_cmds[] = {
	EXYNOS_DSI_CMD_SEQ(MIPI_DCS_SET_DISPLAY_OFF),
};

static const struct exynos_dsi_cmd ak3b_lp_low_cmds[] = {
	EXYNOS_DSI_CMD_SEQ_DELAY(17, 0x53, 0x25), /* AOD 10 nit */
	EXYNOS_DSI_CMD_SEQ(MIPI_DCS_SET_DISPLAY_ON),
};

static const struct exynos_dsi_cmd ak3b_lp_high_cmds[] = {
	EXYNOS_DSI_CMD_SEQ_DELAY(17, 0x53, 0x24), /* AOD 50 nit */
	EXYNOS_DSI_CMD_SEQ(MIPI_DCS_SET_DISPLAY_ON)
};

static const struct exynos_binned_lp ak3b_binned_lp[] = {
	BINNED_LP_MODE("off", 0, ak3b_lp_off_cmds),
	/* rising time = delay = 12, falling time = delay + width = 12 + 35 */
	BINNED_LP_MODE_TIMING("low", 813, ak3b_lp_low_cmds, 12, 12 + 35), /* 40 nits */
	BINNED_LP_MODE_TIMING("high", 3175, ak3b_lp_high_cmds, 12, 12 + 35)
};

static const struct exynos_dsi_cmd ak3b_init_cmds[] = {
	EXYNOS_DSI_CMD_SEQ_DELAY(120, MIPI_DCS_EXIT_SLEEP_MODE),

	/* Frequencey settings */
	EXYNOS_DSI_CMD0_REV(test_key_on_f0, PANEL_REV_GE(PANEL_REV_PROTO1_1)),
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1), 0x60, 0x08, 0x00), /* 60 hz HS */
	EXYNOS_DSI_CMD_SEQ_REV(PANEL_REV_GE(PANEL_REV_PROTO1_1), 0xF7, 0x0F), /* freq_update */
	EXYNOS_DSI_CMD0_REV(test_key_off_f0, PANEL_REV_GE(PANEL_REV_PROTO1_1)),

	EXYNOS_DSI_CMD_SEQ(MIPI_DCS_SET_TEAR_ON, 0x00),
	EXYNOS_DSI_CMD_SEQ(MIPI_DCS_SET_COLUMN_ADDRESS, 0x00, 0x00, 0x04, 0x37),
	EXYNOS_DSI_CMD_SEQ(MIPI_DCS_SET_PAGE_ADDRESS, 0x00, 0x00, 0x09, 0x5F),
};
static DEFINE_EXYNOS_CMD_SET(ak3b_init);

static const struct exynos_dsi_cmd ak3b_lhbm_location_cmds[] = {
	/* test_key_on_f0 */
	EXYNOS_DSI_CMD_SEQ(0xF0, 0x5A, 0x5A),
	/* test_key_on_f1 */
	EXYNOS_DSI_CMD_SEQ(0xF1, 0x5A, 0x5A),

	/* global para */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x07, 0x6D),
	/* box location */
	EXYNOS_DSI_CMD_SEQ(0x6D, 0xCC),
	/* global para */
	EXYNOS_DSI_CMD_SEQ(0xB0, 0x00, 0x08, 0x6D),
	/* center position set, x: 0x21C, y: 0x6B9, size: 0x63 */
	EXYNOS_DSI_CMD_SEQ(0x6D, 0x21, 0xC6, 0xB9, 0x63),

	/* test_key_off_f1 */
	EXYNOS_DSI_CMD_SEQ(0xF1, 0xA5, 0xA5),
	/* test_key_off_f0 */
	EXYNOS_DSI_CMD_SEQ(0xF0, 0xA5, 0xA5)
};
static DEFINE_EXYNOS_CMD_SET(ak3b_lhbm_location);

#define LHBM_GAMMA_CMD_SIZE 6
/**
 * struct ak3b_panel - panel specific runtime info
 *
 * This struct maintains ak3b panel specific runtime info, any fixed details about panel
 * should most likely go into struct exynos_panel_desc
 */
struct ak3b_panel {
	/** @base: base panel struct */
	struct exynos_panel base;

	/** @local_hbm_gamma: lhbm gamma data */
	struct local_hbm_gamma {
		u8 hs120_cmd[LHBM_GAMMA_CMD_SIZE];
		u8 hs60_cmd[LHBM_GAMMA_CMD_SIZE];
		u8 ns_cmd[LHBM_GAMMA_CMD_SIZE];
		u8 aod_cmd[LHBM_GAMMA_CMD_SIZE];
	} local_hbm_gamma;
};

#define to_spanel(ctx) container_of(ctx, struct ak3b_panel, base)

enum frequency { HS120, HS60, NS60, AOD };
static const char* frequency_str[] = { "HS120", "HS60", "NS60", "AOD" };

static u8 get_lhbm_read_cmd(struct exynos_panel *ctx, enum frequency freq) {
	switch(freq) {
	case HS120:
		return 0x22;
	case HS60:
	case AOD:
		return 0x18;
	case NS60:
		return 0x1D;
	default:
		dev_err(ctx->dev, "LHBM Gamma read for unknown frequency %d\n", freq);
		return 0x22;
	}
}

static void read_lhbm_gamma(struct exynos_panel *ctx, u8 *cmd, enum frequency freq) {
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	u8 index = get_lhbm_read_cmd(ctx, freq);
	int ret;

	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0xB0, 0x00, index, 0xD8); /* global para */
	ret = mipi_dsi_dcs_read(dsi, 0xD8, cmd + 1, LHBM_GAMMA_CMD_SIZE - 1);

	if (ret != (LHBM_GAMMA_CMD_SIZE - 1)) {
		dev_err(ctx->dev, "fail to read LHBM gamma for %s\n", frequency_str[freq]);
		return;
	}

	/* fill in gamma write command 0x66 in offset 0 */
	cmd[0] = 0x66;
	dev_dbg(ctx->dev, "%s_gamma: %*ph\n", frequency_str[freq],
		LHBM_GAMMA_CMD_SIZE - 1, cmd + 1);
}

static void ak3b_lhbm_gamma_read(struct exynos_panel *ctx)
{
	struct ak3b_panel *spanel = to_spanel(ctx);

	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);

	read_lhbm_gamma(ctx, spanel->local_hbm_gamma.hs120_cmd, HS120);
	read_lhbm_gamma(ctx, spanel->local_hbm_gamma.ns_cmd, NS60);

	if (ctx->panel_rev == PANEL_REV_PROTO1) {
		read_lhbm_gamma(ctx, spanel->local_hbm_gamma.aod_cmd, AOD);
	} else {
		read_lhbm_gamma(ctx, spanel->local_hbm_gamma.hs60_cmd, HS60);
	}

	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, test_key_off_f0);
}

static void ak3b_lhbm_gamma_write(struct exynos_panel *ctx)
{
	struct ak3b_panel *spanel = to_spanel(ctx);
	const u8 hs120_cmd = spanel->local_hbm_gamma.hs120_cmd[0];
	const u8 hs60_cmd = spanel->local_hbm_gamma.hs60_cmd[0];
	const u8 ns_cmd = spanel->local_hbm_gamma.ns_cmd[0];
	const u8 aod_cmd = spanel->local_hbm_gamma.aod_cmd[0];

	if (!hs120_cmd && !hs60_cmd && !ns_cmd && !aod_cmd) {
		dev_err(ctx->dev, "%s: no lhbm gamma!\n", __func__);
		return;
	}

	dev_dbg(ctx->dev, "%s\n", __func__);
	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);

	if (hs120_cmd) {
		/* HS120 */
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x03, 0xD7, 0x66); /* global para */
		EXYNOS_DCS_BUF_ADD_SET(ctx, spanel->local_hbm_gamma.hs120_cmd); /* write gamma */

		/* HS60 */
		if (ctx->panel_rev == PANEL_REV_PROTO1) {
			EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x03, 0xDC, 0x66); /* global para */
			EXYNOS_DCS_BUF_ADD_SET(ctx,
				spanel->local_hbm_gamma.hs120_cmd); /* write gamma */
		}
	}

	if (hs60_cmd) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x03, 0xDC, 0x66); /* global para */
		EXYNOS_DCS_BUF_ADD_SET(ctx, spanel->local_hbm_gamma.hs60_cmd); /* write gamma */
	}

	if (ns_cmd) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x03, 0xE6, 0x66); /* global para */
		EXYNOS_DCS_BUF_ADD_SET(ctx, spanel->local_hbm_gamma.ns_cmd); /* write gamma */
	}

	if (aod_cmd) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x03, 0xEB, 0x66); /* global para */
		EXYNOS_DCS_BUF_ADD_SET(ctx, spanel->local_hbm_gamma.aod_cmd); /* write gamma */
	}

	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, test_key_off_f0);
}

static void ak3b_get_te2_setting(struct exynos_panel_te2_timing *timing,
				    u8 *setting)
{
	u8 delay_low_byte, delay_high_byte;
	u8 width_low_byte, width_high_byte;
	u32 rising, falling;

	if (!timing || !setting)
		return;

	rising = timing->rising_edge;
	falling = timing->falling_edge;

	delay_low_byte = rising & 0xFF;
	delay_high_byte = (rising >> 8) & 0xF;
	width_low_byte = (falling - rising) & 0xFF;
	width_high_byte = ((falling - rising) >> 8) & 0xF;

	setting[0] = (delay_high_byte << 4) | width_high_byte;
	setting[1] = delay_low_byte;
	setting[2] = width_low_byte;
}

static void ak3b_update_te2(struct exynos_panel *ctx)
{
	struct exynos_panel_te2_timing timing;
	u8 hs_120hz_setting[4] = {0xCB, 0x00, 0x0C, 0x32};
	u8 hs_60hz_setting[4] = {0xCB, 0x00, 0x0C, 0x32};
	u8 lp_setting[4] = {0xCB, 0x00, 0x0C, 0x23}; // lp low/high

	if (!ctx)
		return;

	/* HS mode */
	timing = ctx->te2.mode_data[0].timing;
	ak3b_get_te2_setting(&timing, &hs_120hz_setting[1]);

	dev_dbg(ctx->dev, "TE2 updated HS 120Hz: [HEX] %*ph\n", 4, hs_120hz_setting);

	timing = ctx->te2.mode_data[1].timing;
	ak3b_get_te2_setting(&timing, &hs_60hz_setting[1]);

	dev_dbg(ctx->dev, "TE2 updated HS 60Hz: [HEX] %*ph\n", 4, hs_60hz_setting);

	/* LP mode */
	if (ctx->current_mode->exynos_mode.is_lp_mode) {
		int ret = exynos_panel_get_current_mode_te2(ctx, &timing);

		if (!ret)
			ak3b_get_te2_setting(&timing, &lp_setting[1]);
		else if (ret == -EAGAIN)
			dev_dbg(ctx->dev,
				"Panel is not ready, use default setting\n");
		else
			return;

		dev_dbg(ctx->dev, "TE2 updated LP: [HEX] %*ph\n", 4, lp_setting);
	}

	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x2B, 0xF2); /* global para */
	EXYNOS_DCS_BUF_ADD(ctx, 0xF2, 0x03, 0x14); /* TE2 on */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x73, 0xCB); /* global para */
	EXYNOS_DCS_BUF_ADD_SET(ctx, hs_120hz_setting); /* HS 120Hz control */
	EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0xC9, 0xCB); /* global para */
	EXYNOS_DCS_BUF_ADD_SET(ctx, hs_60hz_setting); /* HS 60Hz control */
	if (ctx->current_mode->exynos_mode.is_lp_mode) {
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x01, 0xC8, 0xCB); /* global para */
		EXYNOS_DCS_BUF_ADD_SET(ctx, lp_setting); /* HLPM mode */
	}
	EXYNOS_DCS_BUF_ADD_SET(ctx, freq_update); /* LTPS update */
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, test_key_off_f0);
}

static void ak3b_change_frequency(struct exynos_panel *ctx,
				    const unsigned int vrefresh)
{
	const u8 hs60_setting[3] = {0x60, 0x08, 0x00};
	const u8 hs120_setting[3] = {0x60, 0x00, 0x00};

	if (!ctx || (vrefresh != 60 && vrefresh != 120))
		return;

	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);
	if (vrefresh == 120) {
		EXYNOS_DCS_BUF_ADD_SET(ctx, hs120_setting);
		EXYNOS_DCS_BUF_ADD(ctx, 0xB9, 0x31);
	} else {
		EXYNOS_DCS_BUF_ADD_SET(ctx, hs60_setting);
		EXYNOS_DCS_BUF_ADD(ctx, 0xB9, 0x30);
	}
	EXYNOS_DCS_BUF_ADD_SET(ctx, freq_update);
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, test_key_off_f0);

	dev_dbg(ctx->dev, "%s: change to %uhz\n", __func__, vrefresh);
}

static void buf_add_frequency_select_cmd(struct exynos_panel *ctx) {
	u32 vrefresh = drm_mode_vrefresh(&ctx->current_mode->mode);
	/* NS60: 0x18, HS120: 0x00, HS60: 0x08 */
	u8 index = 0x18;
	if (ctx->op_hz != 60) {
	    index = (vrefresh == 120) ? 0x00 : 0x08;
	}

	EXYNOS_DCS_BUF_ADD(ctx, 0x60, index, 0x00);
}

static int ak3b_set_op_hz(struct exynos_panel *ctx, unsigned int hz)
{
	const unsigned int vrefresh = drm_mode_vrefresh(&ctx->current_mode->mode);

	if ((vrefresh > hz) || ((hz != 60) && (hz != 120))) {
		dev_err(ctx->dev, "invalid op_hz=%u for vrefresh=%u\n",
			hz, vrefresh);
		return -EINVAL;
	}

	ctx->op_hz = hz;

	EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);
	buf_add_frequency_select_cmd(ctx);
	EXYNOS_DCS_BUF_ADD_SET(ctx, freq_update);
	EXYNOS_DCS_BUF_ADD_SET_AND_FLUSH(ctx, test_key_off_f0);

	dev_info(ctx->dev, "set op_hz at %u\n", hz);
	return 0;
}

static void ak3b_update_wrctrld(struct exynos_panel *ctx)
{
	u8 val = AK3B_WRCTRLD_BCTRL_BIT;

	if (IS_HBM_ON(ctx->hbm_mode))
		val |= AK3B_WRCTRLD_HBM_BIT;

	if (ctx->hbm.local_hbm.enabled)
		val |= AK3B_WRCTRLD_LOCAL_HBM_BIT;

	if (ctx->dimming_on)
		val |= AK3B_WRCTRLD_DIMMING_BIT;

	dev_dbg(ctx->dev,
		"%s(wrctrld:0x%x, hbm: %s, dimming: %s, local_hbm: %s)\n",
		__func__, val, IS_HBM_ON(ctx->hbm_mode) ? "on" : "off",
		ctx->dimming_on ? "on" : "off",
		ctx->hbm.local_hbm.enabled ? "on" : "off");

	if (ctx->panel_rev >= PANEL_REV_PROTO1_1) {
		EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_on_f0);
		EXYNOS_DCS_BUF_ADD(ctx, 0xB0, 0x00, 0x01, 0xBD); /* global_para */

		if (ctx->hbm.local_hbm.enabled) {
			EXYNOS_DCS_BUF_ADD(ctx, 0xBD, 0x00, 0x01, 0x01, 0x01); /* pulse settings */
		} else {
			EXYNOS_DCS_BUF_ADD(ctx, 0xBD, 0x01, 0x03, 0x03, 0x03); /* pulse settings */
			buf_add_frequency_select_cmd(ctx);
		}

		EXYNOS_DCS_BUF_ADD_SET(ctx, freq_update);
		EXYNOS_DCS_BUF_ADD_SET(ctx, test_key_off_f0);
	}

	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, MIPI_DCS_WRITE_CONTROL_DISPLAY, val);
}

#define MAX_BR_HBM 4095
static int ak3b_set_brightness(struct exynos_panel *ctx, u16 br)
{
	u16 brightness;

	if (ctx->current_mode->exynos_mode.is_lp_mode) {
		const struct exynos_panel_funcs *funcs;

		funcs = ctx->desc->exynos_panel_func;
		if (funcs && funcs->set_binned_lp)
			funcs->set_binned_lp(ctx, br);
		return 0;
	}

	if (br > MAX_BR_HBM) {
		br = MAX_BR_HBM;
		dev_dbg(ctx->dev, "%s: capped to dbv(%d)\n",
			__func__, MAX_BR_HBM);
	}

	brightness = (br & 0xff) << 8 | br >> 8;

	return exynos_dcs_set_brightness(ctx, brightness);
}

static void ak3b_set_nolp_mode(struct exynos_panel *ctx,
				  const struct exynos_panel_mode *pmode)
{
	unsigned int vrefresh = drm_mode_vrefresh(&pmode->mode);
	u32 delay_us = mult_frac(1000, 1020, vrefresh);

	if (!is_panel_active(ctx))
		return;

	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, MIPI_DCS_SET_DISPLAY_OFF);
	/* backlight control and dimming */
	ak3b_update_wrctrld(ctx);
	ak3b_change_frequency(ctx, vrefresh);
	usleep_range(delay_us, delay_us + 10);
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, MIPI_DCS_SET_DISPLAY_ON);

	dev_info(ctx->dev, "exit LP mode\n");
}

static int ak3b_enable(struct drm_panel *panel)
{
	struct exynos_panel *ctx = container_of(panel, struct exynos_panel, panel);
	const struct exynos_panel_mode *pmode = ctx->current_mode;
	const struct drm_display_mode *mode;
	struct drm_dsc_picture_parameter_set pps_payload;

	if (!pmode) {
		dev_err(ctx->dev, "no current mode set\n");
		return -EINVAL;
	}
	mode = &pmode->mode;

	dev_dbg(ctx->dev, "%s\n", __func__);

	exynos_panel_reset(ctx);

	exynos_panel_send_cmd_set(ctx, &ak3b_init_cmd_set);

	ak3b_change_frequency(ctx, drm_mode_vrefresh(mode));

	ak3b_lhbm_gamma_write(ctx);
	exynos_panel_send_cmd_set(ctx, &ak3b_lhbm_location_cmd_set);

	/* DSC related configuration */
	drm_dsc_pps_payload_pack(&pps_payload, &pps_config);
	exynos_dcs_compression_mode(ctx, 0x1); /* DSC_DEC_ON */
	EXYNOS_PPS_WRITE_BUF(ctx, &pps_payload); /* PPS_SETTING */
	EXYNOS_DCS_BUF_ADD(ctx, 0xC2, 0x14); /* PPS_MIC_OFF */
	EXYNOS_DCS_BUF_ADD_AND_FLUSH(ctx, 0x9D, 0x01); /* PPS_DSC_EN */

	ak3b_update_wrctrld(ctx); /* dimming and HBM */

	if (pmode->exynos_mode.is_lp_mode)
		exynos_panel_set_lp_mode(ctx, pmode);
	else
		EXYNOS_DCS_WRITE_SEQ(ctx, MIPI_DCS_SET_DISPLAY_ON); /* display on */

	return 0;
}

static void ak3b_set_hbm_mode(struct exynos_panel *exynos_panel,
				enum exynos_hbm_mode mode)
{
	const bool hbm_update =
		(IS_HBM_ON(exynos_panel->hbm_mode) != IS_HBM_ON(mode));
	const bool irc_update =
		(IS_HBM_ON_IRC_OFF(exynos_panel->hbm_mode) != IS_HBM_ON_IRC_OFF(mode));

	exynos_panel->hbm_mode = mode;

	if (hbm_update)
		ak3b_update_wrctrld(exynos_panel);

	if (irc_update) {
		EXYNOS_DCS_BUF_ADD(exynos_panel, 0xF0, 0x5A, 0x5A); /* test_key_on */
		EXYNOS_DCS_BUF_ADD(exynos_panel, 0xB0, 0x00, 0x01, 0x6A); /* global para */
		EXYNOS_DCS_BUF_ADD(exynos_panel, 0x6A, IS_HBM_ON_IRC_OFF(mode) ? 0x01 : 0x21);
		EXYNOS_DCS_BUF_ADD_AND_FLUSH(exynos_panel, 0xF0, 0xA5, 0xA5); /* test_key_off */
	}
	dev_info(exynos_panel->dev, "hbm_on=%d hbm_ircoff=%d\n", IS_HBM_ON(exynos_panel->hbm_mode),
		 IS_HBM_ON_IRC_OFF(exynos_panel->hbm_mode));
}

static void ak3b_set_dimming_on(struct exynos_panel *exynos_panel,
				 bool dimming_on)
{
	const struct exynos_panel_mode *pmode = exynos_panel->current_mode;

	exynos_panel->dimming_on = dimming_on;
	if (pmode->exynos_mode.is_lp_mode) {
		dev_info(exynos_panel->dev, "in lp mode, skip to update");
		return;
	}

	ak3b_update_wrctrld(exynos_panel);
}

static void ak3b_set_local_hbm_mode(struct exynos_panel *exynos_panel,
				 bool local_hbm_en)
{
	ak3b_update_wrctrld(exynos_panel);
}

static void ak3b_mode_set(struct exynos_panel *ctx,
			     const struct exynos_panel_mode *pmode)
{
	ak3b_change_frequency(ctx, drm_mode_vrefresh(&pmode->mode));
}

static bool ak3b_is_mode_seamless(const struct exynos_panel *ctx,
				     const struct exynos_panel_mode *pmode)
{
	/* seamless mode switch is possible if only changing refresh rate */
	return drm_mode_equal_no_clocks(&ctx->current_mode->mode, &pmode->mode);
}

static void ak3b_panel_init(struct exynos_panel *ctx)
{
	struct dentry *csroot = ctx->debugfs_cmdset_entry;

	exynos_panel_debugfs_create_cmdset(ctx, csroot,
					   &ak3b_init_cmd_set, "init");

	exynos_panel_send_cmd_set(ctx, &ak3b_lhbm_location_cmd_set);
	ak3b_lhbm_gamma_read(ctx);
	ak3b_lhbm_gamma_write(ctx);
}

static void ak3b_get_panel_rev(struct exynos_panel *ctx, u32 id)
{
	/* extract command 0xDB */
	u8 build_code = (id & 0xFF00) >> 8;
	u8 rev = ((build_code & 0xE0) >> 3) | ((build_code & 0x0C) >> 2);

	exynos_panel_get_panel_rev(ctx, rev);
}

static int ak3b_panel_probe(struct mipi_dsi_device *dsi)
{
	struct ak3b_panel *spanel;

	spanel = devm_kzalloc(&dsi->dev, sizeof(*spanel), GFP_KERNEL);
	if (!spanel)
		return -ENOMEM;

	spanel->base.op_hz = 120;

	return exynos_panel_common_init(dsi, &spanel->base);
}

static const struct exynos_display_underrun_param underrun_param = {
	.te_idle_us = 1000,
	.te_var = 1,
};

static const u32 ak3b_bl_range[] = {
	95, 205, 315, 400, 3175
};

#define AK3_DSC {\
	.enabled = true,\
	.dsc_count = 2,\
	.slice_count = 2,\
	.slice_height = 48,\
	.cfg = &pps_config\
}

static const struct exynos_panel_mode ak3b_modes[] = {
	{
		.mode = {
			.name = "1080x2400x60",
			.clock = 168498,
			.hdisplay = 1080,
			.hsync_start = 1080 + 32, // add hfp
			.hsync_end = 1080 + 32 + 12, // add hsa
			.htotal = 1080 + 32 + 12 + 26, // add hbp
			.vdisplay = 2400,
			.vsync_start = 2400 + 12, // add vfp
			.vsync_end = 2400 + 12 + 4, // add vsa
			.vtotal = 2400 + 12 + 4 + 26, // add vbp
			.flags = 0,
			.width_mm = 70,
			.height_mm = 149,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.te_usec = 8500,
			.bpc = 8,
			.dsc = AK3_DSC,
			.underrun_param = &underrun_param,
		},
		.te2_timing = {
			.rising_edge = 12,
			.falling_edge = 12 + 50,
		},
	},
	{
		.mode = {
			.name = "1080x2400x120",
			.clock = 336996,
			.hdisplay = 1080,
			.hsync_start = 1080 + 32, // add hfp
			.hsync_end = 1080 + 32 + 12, // add hsa
			.htotal = 1080 + 32 + 12 + 26, // add hbp
			.vdisplay = 2400,
			.vsync_start = 2400 + 12, // add vfp
			.vsync_end = 2400 + 12 + 4, // add vsa
			.vtotal = 2400 + 12 + 4 + 26, // add vbp
			.flags = 0,
			.width_mm = 70,
			.height_mm = 149,
		},
		.exynos_mode = {
			.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
			.vblank_usec = 120,
			.te_usec = 221,
			.bpc = 8,
			.dsc = AK3_DSC,
			.underrun_param = &underrun_param,
		},
		.te2_timing = {
			.rising_edge = 12,
			.falling_edge = 12 + 50,
		},
	},
};

static const struct exynos_panel_mode ak3b_lp_mode = {
	.mode = {
		.name = "1080x2400x30",
		.clock = 84249,
		.hdisplay = 1080,
		.hsync_start = 1080 + 32, // add hfp
		.hsync_end = 1080 + 32 + 12, // add hsa
		.htotal = 1080 + 32 + 12 + 26, // add hbp
		.vdisplay = 2400,
		.vsync_start = 2400 + 12, // add vfp
		.vsync_end = 2400 + 12 + 4, // add vsa
		.vtotal = 2400 + 12 + 4 + 26, // add vbp
		.flags = 0,
		.type = DRM_MODE_TYPE_DRIVER,
		.width_mm = 70,
		.height_mm = 149,
	},
	.exynos_mode = {
		.mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS,
		.vblank_usec = 120,
		.bpc = 8,
		.dsc = AK3_DSC,
		.underrun_param = &underrun_param,
		.is_lp_mode = true,
	}
};

static const struct drm_panel_funcs ak3b_drm_funcs = {
	.disable = exynos_panel_disable,
	.unprepare = exynos_panel_unprepare,
	.prepare = exynos_panel_prepare,
	.enable = ak3b_enable,
	.get_modes = exynos_panel_get_modes,
};

static const struct exynos_panel_funcs ak3b_exynos_funcs = {
	.set_brightness = ak3b_set_brightness,
	.set_lp_mode = exynos_panel_set_lp_mode,
	.set_nolp_mode = ak3b_set_nolp_mode,
	.set_binned_lp = exynos_panel_set_binned_lp,
	.set_hbm_mode = ak3b_set_hbm_mode,
	.set_dimming_on = ak3b_set_dimming_on,
	.set_local_hbm_mode = ak3b_set_local_hbm_mode,
	.is_mode_seamless = ak3b_is_mode_seamless,
	.mode_set = ak3b_mode_set,
	.panel_init = ak3b_panel_init,
	.get_panel_rev = ak3b_get_panel_rev,
	.get_te2_edges = exynos_panel_get_te2_edges,
	.configure_te2_edges = exynos_panel_configure_te2_edges,
	.update_te2 = ak3b_update_te2,
	.set_op_hz = ak3b_set_op_hz,
	.read_id = exynos_panel_read_ddic_id,
};

const struct brightness_capability ak3b_brightness_capability = {
	.normal = {
		.nits = {
			.min = 2,
			.max = 800,
		},
		.level = {
			.min = 209,
			.max = 3175,
		},
		.percentage = {
			.min = 0,
			.max = 60,
		},
	},
	.hbm = {
		.nits = {
			.min = 800,
			.max = 1400,
		},
		.level = {
			.min = 3176,
			.max = 4095,
		},
		.percentage = {
			.min = 60,
			.max = 100,
		},
	},
};

const struct exynos_panel_desc google_ak3b = {
	.data_lane_cnt = 4,
	.max_brightness = 4095,
	.min_brightness = 209,
	.dft_brightness = 1023,
	.brt_capability = &ak3b_brightness_capability,
	/* supported HDR format bitmask : 1(DOLBY_VISION), 2(HDR10), 3(HLG) */
	.hdr_formats = BIT(2) | BIT(3),
	.max_luminance = 14000000,
	.max_avg_luminance = 1200000,
	.min_luminance = 5,
	.bl_range = ak3b_bl_range,
	.bl_num_ranges = ARRAY_SIZE(ak3b_bl_range),
	.modes = ak3b_modes,
	.num_modes = ARRAY_SIZE(ak3b_modes),
	.off_cmd_set = &ak3b_off_cmd_set,
	.lp_mode = &ak3b_lp_mode,
	.lp_cmd_set = &ak3b_lp_cmd_set,
	.binned_lp = ak3b_binned_lp,
	.num_binned_lp = ARRAY_SIZE(ak3b_binned_lp),
	.no_lhbm_rr_constraints = true,
	.panel_func = &ak3b_drm_funcs,
	.exynos_panel_func = &ak3b_exynos_funcs,
	.reset_timing_ms = {1, 1, 20},
	.reg_ctrl_enable = {
		{PANEL_REG_ID_VDDI, 0},
		{PANEL_REG_ID_VCI, 0},
		{PANEL_REG_ID_VDDD, 10},
	},
	.reg_ctrl_disable = {
		{PANEL_REG_ID_VDDD, 0},
		{PANEL_REG_ID_VCI, 0},
		{PANEL_REG_ID_VDDI, 0},
	},
};

static const struct of_device_id exynos_panel_of_match[] = {
	{ .compatible = "google,ak3b", .data = &google_ak3b },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos_panel_of_match);

static struct mipi_dsi_driver exynos_panel_driver = {
	.probe = ak3b_panel_probe,
	.remove = exynos_panel_remove,
	.driver = {
		.name = "panel-google-ak3b",
		.of_match_table = exynos_panel_of_match,
	},
};
module_mipi_dsi_driver(exynos_panel_driver);

MODULE_AUTHOR("Safayat Ullah <safayat@google.com>");
MODULE_DESCRIPTION("MIPI-DSI based Google ak3b panel driver");
MODULE_LICENSE("GPL");
