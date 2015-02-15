/*
 * Copyright © 2008 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Keith Packard <keithp@keithp.com>
 *
 */

#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/export.h>
#include "drmP.h"
#include "drm.h"
#include "drm_crtc.h"
#include "drm_crtc_helper.h"
#include "intel_drv.h"
#include "i915_drm.h"
#include "i915_drv.h"
#include "drm_dp_helper.h"

#define DP_RECEIVER_CAP_SIZE	0xf
#define DP_LINK_STATUS_SIZE	6
#define DP_LINK_CHECK_TIMEOUT	(10 * 1000)

#define DP_LINK_CONFIGURATION_SIZE	9

struct intel_dp {
	struct intel_encoder base;
	uint32_t output_reg;
	uint32_t DP;
	uint8_t  link_configuration[DP_LINK_CONFIGURATION_SIZE];
	bool has_audio;
	enum hdmi_force_audio force_audio;
	uint32_t color_range;
	int dpms_mode;
	uint8_t link_bw;
	uint8_t lane_count;
	uint8_t dpcd[DP_RECEIVER_CAP_SIZE];
	struct i2c_adapter adapter;
	struct i2c_algo_dp_aux_data algo;
	bool is_pch_edp;
	uint8_t	train_set[4];
	int panel_power_up_delay;
	int panel_power_down_delay;
	int panel_power_cycle_delay;
	int backlight_on_delay;
	int backlight_off_delay;
	struct drm_display_mode *panel_fixed_mode;  /* for eDP */
	struct delayed_work panel_vdd_work;
	bool want_panel_vdd;
};

/**
 * is_edp - is the given port attached to an eDP panel (either CPU or PCH)
 * @intel_dp: DP struct
 *
 * If a CPU or PCH DP output is attached to an eDP panel, this function
 * will return true, and false otherwise.
 */
static bool is_edp(struct intel_dp *intel_dp)
{
	return intel_dp->base.type == INTEL_OUTPUT_EDP;
}

/**
 * is_pch_edp - is the port on the PCH and attached to an eDP panel?
 * @intel_dp: DP struct
 *
 * Returns true if the given DP struct corresponds to a PCH DP port attached
 * to an eDP panel, false otherwise.  Helpful for determining whether we
 * may need FDI resources for a given DP output or not.
 */
static bool is_pch_edp(struct intel_dp *intel_dp)
{
	return intel_dp->is_pch_edp;
}

/**
 * is_cpu_edp - is the port on the CPU and attached to an eDP panel?
 * @intel_dp: DP struct
 *
 * Returns true if the given DP struct corresponds to a CPU eDP port.
 */
static bool is_cpu_edp(struct intel_dp *intel_dp)
{
	return is_edp(intel_dp) && !is_pch_edp(intel_dp);
}

static struct intel_dp *enc_to_intel_dp(struct drm_encoder *encoder)
{
	return container_of(encoder, struct intel_dp, base.base);
}

static struct intel_dp *intel_attached_dp(struct drm_connector *connector)
{
	return container_of(intel_attached_encoder(connector),
			    struct intel_dp, base);
}

/**
 * intel_encoder_is_pch_edp - is the given encoder a PCH attached eDP?
 * @encoder: DRM encoder
 *
 * Return true if @encoder corresponds to a PCH attached eDP panel.  Needed
 * by intel_display.c.
 */
bool intel_encoder_is_pch_edp(struct drm_encoder *encoder)
{
	struct intel_dp *intel_dp;

	if (!encoder)
		return false;

	intel_dp = enc_to_intel_dp(encoder);

	return is_pch_edp(intel_dp);
}

static void intel_dp_start_link_train(struct intel_dp *intel_dp);
static void intel_dp_complete_link_train(struct intel_dp *intel_dp);
static void intel_dp_link_down(struct intel_dp *intel_dp);

void
intel_edp_link_config(struct intel_encoder *intel_encoder,
		       int *lane_num, int *link_bw)
{
	struct intel_dp *intel_dp = container_of(intel_encoder, struct intel_dp, base);

	*lane_num = intel_dp->lane_count;
	if (intel_dp->link_bw == DP_LINK_BW_1_62)
		*link_bw = 162000;
	else if (intel_dp->link_bw == DP_LINK_BW_2_7)
		*link_bw = 270000;
}

static int
intel_dp_max_lane_count(struct intel_dp *intel_dp)
{
	int max_lane_count = intel_dp->dpcd[DP_MAX_LANE_COUNT] & 0x1f;
	switch (max_lane_count) {
	case 1: case 2: case 4:
		break;
	default:
		max_lane_count = 4;
	}
	return max_lane_count;
}

static int
intel_dp_max_link_bw(struct intel_dp *intel_dp)
{
	int max_link_bw = intel_dp->dpcd[DP_MAX_LINK_RATE];

	switch (max_link_bw) {
	case DP_LINK_BW_1_62:
	case DP_LINK_BW_2_7:
		break;
	default:
		max_link_bw = DP_LINK_BW_1_62;
		break;
	}
	return max_link_bw;
}

static int
intel_dp_link_clock(uint8_t link_bw)
{
	if (link_bw == DP_LINK_BW_2_7)
		return 270000;
	else
		return 162000;
}

/*
 * The units on the numbers in the next two are... bizarre.  Examples will
 * make it clearer; this one parallels an example in the eDP spec.
 *
 * intel_dp_max_data_rate for one lane of 2.7GHz evaluates as:
 *
 *     270000 * 1 * 8 / 10 == 216000
 *
 * The actual data capacity of that configuration is 2.16Gbit/s, so the
 * units are decakilobits.  ->clock in a drm_display_mode is in kilohertz -
 * or equivalently, kilopixels per second - so for 1680x1050R it'd be
 * 119000.  At 18bpp that's 2142000 kilobits per second.
 *
 * Thus the strange-looking division by 10 in intel_dp_link_required, to
 * get the result in decakilobits instead of kilobits.
 */

static int
intel_dp_link_required(int pixel_clock, int bpp)
{
	return (pixel_clock * bpp + 9) / 10;
}

static int
intel_dp_max_data_rate(int max_link_clock, int max_lanes)
{
	return (max_link_clock * max_lanes * 8) / 10;
}

static bool
intel_dp_adjust_dithering(struct intel_dp *intel_dp,
			  struct drm_display_mode *mode,
			  struct drm_display_mode *adjusted_mode)
{
	int max_link_clock = intel_dp_link_clock(intel_dp_max_link_bw(intel_dp));
	int max_lanes = intel_dp_max_lane_count(intel_dp);
	int max_rate, mode_rate;

	mode_rate = intel_dp_link_required(mode->clock, 24);
	max_rate = intel_dp_max_data_rate(max_link_clock, max_lanes);

	if (mode_rate > max_rate) {
		mode_rate = intel_dp_link_required(mode->clock, 18);
		if (mode_rate > max_rate)
			return false;

		if (adjusted_mode)
			adjusted_mode->private_flags
				|= INTEL_MODE_DP_FORCE_6BPC;

		return true;
	}

	return true;
}

static int
intel_dp_mode_valid(struct drm_connector *connector,
		    struct drm_display_mode *mode)
{
	struct intel_dp *intel_dp = intel_attached_dp(connector);

	if (is_edp(intel_dp) && intel_dp->panel_fixed_mode) {
		if (mode->hdisplay > intel_dp->panel_fixed_mode->hdisplay)
			return MODE_PANEL;

		if (mode->vdisplay > intel_dp->panel_fixed_mode->vdisplay)
			return MODE_PANEL;
	}

	if (!intel_dp_adjust_dithering(intel_dp, mode, NULL))
		return MODE_CLOCK_HIGH;

	if (mode->clock < 10000)
		return MODE_CLOCK_LOW;

	return MODE_OK;
}

static uint32_t
pack_aux(uint8_t *src, int src_bytes)
{
	int	i;
	uint32_t v = 0;

	if (src_bytes > 4)
		src_bytes = 4;
	for (i = 0; i < src_bytes; i++)
		v |= ((uint32_t) src[i]) << ((3-i) * 8);
	return v;
}

static void
unpack_aux(uint32_t src, uint8_t *dst, int dst_bytes)
{
	int i;
	if (dst_bytes > 4)
		dst_bytes = 4;
	for (i = 0; i < dst_bytes; i++)
		dst[i] = src >> ((3-i) * 8);
}

/* hrawclock is 1/4 the FSB frequency */
static int
intel_hrawclk(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t clkcfg;

	clkcfg = I915_READ(CLKCFG);
	switch (clkcfg & CLKCFG_FSB_MASK) {
	case CLKCFG_FSB_400:
		return 100;
	case CLKCFG_FSB_533:
		return 133;
	case CLKCFG_FSB_667:
		return 166;
	case CLKCFG_FSB_800:
		return 200;
	case CLKCFG_FSB_1067:
		return 266;
	case CLKCFG_FSB_1333:
		return 333;
	/* these two are just a guess; one of them might be right */
	case CLKCFG_FSB_1600:
	case CLKCFG_FSB_1600_ALT:
		return 400;
	default:
		return 133;
	}
}

static bool ironlake_edp_have_panel_power(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	return (I915_READ(PCH_PP_STATUS) & PP_ON) != 0;
}

static bool ironlake_edp_have_panel_vdd(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	return (I915_READ(PCH_PP_CONTROL) & EDP_FORCE_VDD) != 0;
}

static void
intel_dp_check_edp(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (!is_edp(intel_dp))
		return;
	if (!ironlake_edp_have_panel_power(intel_dp) && !ironlake_edp_have_panel_vdd(intel_dp)) {
		WARN(1, "eDP powered off while attempting aux channel communication.\n");
		DRM_DEBUG_KMS("Status 0x%08x Control 0x%08x\n",
			      I915_READ(PCH_PP_STATUS),
			      I915_READ(PCH_PP_CONTROL));
	}
}

static int
intel_dp_aux_ch(struct intel_dp *intel_dp,
		uint8_t *send, int send_bytes,
		uint8_t *recv, int recv_size)
{
	uint32_t output_reg = intel_dp->output_reg;
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t ch_ctl = output_reg + 0x10;
	uint32_t ch_data = ch_ctl + 4;
	int i;
	int recv_bytes;
	uint32_t status;
	uint32_t aux_clock_divider;
	int try, precharge = 5;

	intel_dp_check_edp(intel_dp);
	/* The clock divider is based off the hrawclk,
	 * and would like to run at 2MHz. So, take the
	 * hrawclk value and divide by 2 and use that
	 *
	 * Note that PCH attached eDP panels should use a 125MHz input
	 * clock divider.
	 */
	if (is_cpu_edp(intel_dp)) {
		if (IS_GEN6(dev) || IS_GEN7(dev))
			aux_clock_divider = 200; /* SNB & IVB eDP input clock at 400Mhz */
		else
			aux_clock_divider = 225; /* eDP input clock at 450Mhz */
	} else if (HAS_PCH_SPLIT(dev))
		aux_clock_divider = 63; /* IRL input clock fixed at 125Mhz */
	else
		aux_clock_divider = intel_hrawclk(dev) / 2;

	/* Try to wait for any previous AUX channel activity */
	for (try = 0; try < 3; try++) {
		status = I915_READ(ch_ctl);
		if ((status & DP_AUX_CH_CTL_SEND_BUSY) == 0)
			break;
		msleep(1);
	}

	if (try == 3) {
		WARN(1, "dp_aux_ch not started status 0x%08x\n",
		     I915_READ(ch_ctl));
		return -EBUSY;
	}

	/* Must try at least 3 times according to DP spec */
	for (try = 0; try < 5; try++) {
		/* Load the send data into the aux channel data registers */
		for (i = 0; i < send_bytes; i += 4)
			I915_WRITE(ch_data + i,
				   pack_aux(send + i, send_bytes - i));

		/* Send the command and wait for it to complete */
		I915_WRITE(ch_ctl,
			   DP_AUX_CH_CTL_SEND_BUSY |
			   DP_AUX_CH_CTL_TIME_OUT_400us |
			   (send_bytes << DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT) |
			   (precharge << DP_AUX_CH_CTL_PRECHARGE_2US_SHIFT) |
			   (aux_clock_divider << DP_AUX_CH_CTL_BIT_CLOCK_2X_SHIFT) |
			   DP_AUX_CH_CTL_DONE |
			   DP_AUX_CH_CTL_TIME_OUT_ERROR |
			   DP_AUX_CH_CTL_RECEIVE_ERROR);
		for (;;) {
			status = I915_READ(ch_ctl);
			if ((status & DP_AUX_CH_CTL_SEND_BUSY) == 0)
				break;
			udelay(100);
		}

		/* Clear done status and any errors */
		I915_WRITE(ch_ctl,
			   status |
			   DP_AUX_CH_CTL_DONE |
			   DP_AUX_CH_CTL_TIME_OUT_ERROR |
			   DP_AUX_CH_CTL_RECEIVE_ERROR);

		if (status & (DP_AUX_CH_CTL_TIME_OUT_ERROR |
			      DP_AUX_CH_CTL_RECEIVE_ERROR))
			continue;
		if (status & DP_AUX_CH_CTL_DONE)
			break;
	}

	if ((status & DP_AUX_CH_CTL_DONE) == 0) {
		DRM_ERROR("dp_aux_ch not done status 0x%08x\n", status);
		return -EBUSY;
	}

	/* Check for timeout or receive error.
	 * Timeouts occur when the sink is not connected
	 */
	if (status & DP_AUX_CH_CTL_RECEIVE_ERROR) {
		DRM_ERROR("dp_aux_ch receive error status 0x%08x\n", status);
		return -EIO;
	}

	/* Timeouts occur when the device isn't connected, so they're
	 * "normal" -- don't fill the kernel log with these */
	if (status & DP_AUX_CH_CTL_TIME_OUT_ERROR) {
		DRM_DEBUG_KMS("dp_aux_ch timeout status 0x%08x\n", status);
		return -ETIMEDOUT;
	}

	/* Unload any bytes sent back from the other side */
	recv_bytes = ((status & DP_AUX_CH_CTL_MESSAGE_SIZE_MASK) >>
		      DP_AUX_CH_CTL_MESSAGE_SIZE_SHIFT);
	if (recv_bytes > recv_size)
		recv_bytes = recv_size;

	for (i = 0; i < recv_bytes; i += 4)
		unpack_aux(I915_READ(ch_data + i),
			   recv + i, recv_bytes - i);

	return recv_bytes;
}

/* Write data to the aux channel in native mode */
static int
intel_dp_aux_native_write(struct intel_dp *intel_dp,
			  uint16_t address, uint8_t *send, int send_bytes)
{
	int ret;
	uint8_t	msg[20];
	int msg_bytes;
	uint8_t	ack;

	intel_dp_check_edp(intel_dp);
	if (send_bytes > 16)
		return -1;
	msg[0] = AUX_NATIVE_WRITE << 4;
	msg[1] = address >> 8;
	msg[2] = address & 0xff;
	msg[3] = send_bytes - 1;
	memcpy(&msg[4], send, send_bytes);
	msg_bytes = send_bytes + 4;
	for (;;) {
		ret = intel_dp_aux_ch(intel_dp, msg, msg_bytes, &ack, 1);
		if (ret < 0)
			return ret;
		if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_ACK)
			break;
		else if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_DEFER)
			udelay(100);
		else
			return -EIO;
	}
	return send_bytes;
}

/* Write a single byte to the aux channel in native mode */
static int
intel_dp_aux_native_write_1(struct intel_dp *intel_dp,
			    uint16_t address, uint8_t byte)
{
	return intel_dp_aux_native_write(intel_dp, address, &byte, 1);
}

/* read bytes from a native aux channel */
static int
intel_dp_aux_native_read(struct intel_dp *intel_dp,
			 uint16_t address, uint8_t *recv, int recv_bytes)
{
	uint8_t msg[4];
	int msg_bytes;
	uint8_t reply[20];
	int reply_bytes;
	uint8_t ack;
	int ret;

	intel_dp_check_edp(intel_dp);
	msg[0] = AUX_NATIVE_READ << 4;
	msg[1] = address >> 8;
	msg[2] = address & 0xff;
	msg[3] = recv_bytes - 1;

	msg_bytes = 4;
	reply_bytes = recv_bytes + 1;

	for (;;) {
		ret = intel_dp_aux_ch(intel_dp, msg, msg_bytes,
				      reply, reply_bytes);
		if (ret == 0)
			return -EPROTO;
		if (ret < 0)
			return ret;
		ack = reply[0];
		if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_ACK) {
			memcpy(recv, reply + 1, ret - 1);
			return ret - 1;
		}
		else if ((ack & AUX_NATIVE_REPLY_MASK) == AUX_NATIVE_REPLY_DEFER)
			udelay(100);
		else
			return -EIO;
	}
}

static int
intel_dp_i2c_aux_ch(struct i2c_adapter *adapter, int mode,
		    uint8_t write_byte, uint8_t *read_byte)
{
	struct i2c_algo_dp_aux_data *algo_data = adapter->algo_data;
	struct intel_dp *intel_dp = container_of(adapter,
						struct intel_dp,
						adapter);
	uint16_t address = algo_data->address;
	uint8_t msg[5];
	uint8_t reply[2];
	unsigned retry;
	int msg_bytes;
	int reply_bytes;
	int ret;

	intel_dp_check_edp(intel_dp);
	/* Set up the command byte */
	if (mode & MODE_I2C_READ)
		msg[0] = AUX_I2C_READ << 4;
	else
		msg[0] = AUX_I2C_WRITE << 4;

	if (!(mode & MODE_I2C_STOP))
		msg[0] |= AUX_I2C_MOT << 4;

	msg[1] = address >> 8;
	msg[2] = address;

	switch (mode) {
	case MODE_I2C_WRITE:
		msg[3] = 0;
		msg[4] = write_byte;
		msg_bytes = 5;
		reply_bytes = 1;
		break;
	case MODE_I2C_READ:
		msg[3] = 0;
		msg_bytes = 4;
		reply_bytes = 2;
		break;
	default:
		msg_bytes = 3;
		reply_bytes = 1;
		break;
	}

	for (retry = 0; retry < 5; retry++) {
		ret = intel_dp_aux_ch(intel_dp,
				      msg, msg_bytes,
				      reply, reply_bytes);
		if (ret < 0) {
			DRM_DEBUG_KMS("aux_ch failed %d\n", ret);
			return ret;
		}

		switch (reply[0] & AUX_NATIVE_REPLY_MASK) {
		case AUX_NATIVE_REPLY_ACK:
			/* I2C-over-AUX Reply field is only valid
			 * when paired with AUX ACK.
			 */
			break;
		case AUX_NATIVE_REPLY_NACK:
			DRM_DEBUG_KMS("aux_ch native nack\n");
			return -EREMOTEIO;
		case AUX_NATIVE_REPLY_DEFER:
			udelay(100);
			continue;
		default:
			DRM_ERROR("aux_ch invalid native reply 0x%02x\n",
				  reply[0]);
			return -EREMOTEIO;
		}

		switch (reply[0] & AUX_I2C_REPLY_MASK) {
		case AUX_I2C_REPLY_ACK:
			if (mode == MODE_I2C_READ) {
				*read_byte = reply[1];
			}
			return reply_bytes - 1;
		case AUX_I2C_REPLY_NACK:
			DRM_DEBUG_KMS("aux_i2c nack\n");
			return -EREMOTEIO;
		case AUX_I2C_REPLY_DEFER:
			DRM_DEBUG_KMS("aux_i2c defer\n");
			udelay(100);
			break;
		default:
			DRM_ERROR("aux_i2c invalid reply 0x%02x\n", reply[0]);
			return -EREMOTEIO;
		}
	}

	DRM_ERROR("too many retries, giving up\n");
	return -EREMOTEIO;
}

static void ironlake_edp_panel_vdd_on(struct intel_dp *intel_dp);
static void ironlake_edp_panel_vdd_off(struct intel_dp *intel_dp, bool sync);

static int
intel_dp_i2c_init(struct intel_dp *intel_dp,
		  struct intel_connector *intel_connector, const char *name)
{
	int	ret;

	DRM_DEBUG_KMS("i2c_init %s\n", name);
	intel_dp->algo.running = false;
	intel_dp->algo.address = 0;
	intel_dp->algo.aux_ch = intel_dp_i2c_aux_ch;

	memset(&intel_dp->adapter, '\0', sizeof(intel_dp->adapter));
	intel_dp->adapter.owner = THIS_MODULE;
	intel_dp->adapter.class = I2C_CLASS_DDC;
	strncpy(intel_dp->adapter.name, name, sizeof(intel_dp->adapter.name) - 1);
	intel_dp->adapter.name[sizeof(intel_dp->adapter.name) - 1] = '\0';
	intel_dp->adapter.algo_data = &intel_dp->algo;
	intel_dp->adapter.dev.parent = &intel_connector->base.kdev;

	ironlake_edp_panel_vdd_on(intel_dp);
	ret = i2c_dp_aux_add_bus(&intel_dp->adapter);
	ironlake_edp_panel_vdd_off(intel_dp, false);
	return ret;
}

static bool
intel_dp_mode_fixup(struct drm_encoder *encoder, struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct intel_dp *intel_dp = enc_to_intel_dp(encoder);
	int lane_count, clock;
	int max_lane_count = intel_dp_max_lane_count(intel_dp);
	int max_clock = intel_dp_max_link_bw(intel_dp) == DP_LINK_BW_2_7 ? 1 : 0;
	int bpp;
	static int bws[2] = { DP_LINK_BW_1_62, DP_LINK_BW_2_7 };

	if (is_edp(intel_dp) && intel_dp->panel_fixed_mode) {
		intel_fixed_panel_mode(intel_dp->panel_fixed_mode, adjusted_mode);
		intel_pch_panel_fitting(dev, DRM_MODE_SCALE_FULLSCREEN,
					mode, adjusted_mode);
		/*
		 * the mode->clock is used to calculate the Data&Link M/N
		 * of the pipe. For the eDP the fixed clock should be used.
		 */
		mode->clock = intel_dp->panel_fixed_mode->clock;
	}

	if (!intel_dp_adjust_dithering(intel_dp, mode, adjusted_mode))
		return false;

	bpp = adjusted_mode->private_flags & INTEL_MODE_DP_FORCE_6BPC ? 18 : 24;

	for (lane_count = 1; lane_count <= max_lane_count; lane_count <<= 1) {
		for (clock = 0; clock <= max_clock; clock++) {
			int link_avail = intel_dp_max_data_rate(intel_dp_link_clock(bws[clock]), lane_count);

			if (intel_dp_link_required(mode->clock, bpp)
					<= link_avail) {
				intel_dp->link_bw = bws[clock];
				intel_dp->lane_count = lane_count;
				adjusted_mode->clock = intel_dp_link_clock(intel_dp->link_bw);
				DRM_DEBUG_KMS("Display port link bw %02x lane "
						"count %d clock %d\n",
				       intel_dp->link_bw, intel_dp->lane_count,
				       adjusted_mode->clock);
				return true;
			}
		}
	}

	return false;
}

struct intel_dp_m_n {
	uint32_t	tu;
	uint32_t	gmch_m;
	uint32_t	gmch_n;
	uint32_t	link_m;
	uint32_t	link_n;
};

static void
intel_reduce_ratio(uint32_t *num, uint32_t *den)
{
	while (*num > 0xffffff || *den > 0xffffff) {
		*num >>= 1;
		*den >>= 1;
	}
}

static void
intel_dp_compute_m_n(int bpp,
		     int nlanes,
		     int pixel_clock,
		     int link_clock,
		     struct intel_dp_m_n *m_n)
{
	m_n->tu = 64;
	m_n->gmch_m = (pixel_clock * bpp) >> 3;
	m_n->gmch_n = link_clock * nlanes;
	intel_reduce_ratio(&m_n->gmch_m, &m_n->gmch_n);
	m_n->link_m = pixel_clock;
	m_n->link_n = link_clock;
	intel_reduce_ratio(&m_n->link_m, &m_n->link_n);
}

void
intel_dp_set_m_n(struct drm_crtc *crtc, struct drm_display_mode *mode,
		 struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = crtc->dev;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct drm_encoder *encoder;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	int lane_count = 4;
	struct intel_dp_m_n m_n;
	int pipe = intel_crtc->pipe;

	/*
	 * Find the lane count in the intel_encoder private
	 */
	list_for_each_entry(encoder, &mode_config->encoder_list, head) {
		struct intel_dp *intel_dp;

		if (encoder->crtc != crtc)
			continue;

		intel_dp = enc_to_intel_dp(encoder);
		if (intel_dp->base.type == INTEL_OUTPUT_DISPLAYPORT ||
		    intel_dp->base.type == INTEL_OUTPUT_EDP)
		{
			lane_count = intel_dp->lane_count;
			break;
		}
	}

	/*
	 * Compute the GMCH and Link ratios. The '3' here is
	 * the number of bytes_per_pixel post-LUT, which we always
	 * set up for 8-bits of R/G/B, or 3 bytes total.
	 */
	intel_dp_compute_m_n(intel_crtc->bpp, lane_count,
			     mode->clock, adjusted_mode->clock, &m_n);

	if (HAS_PCH_SPLIT(dev)) {
		I915_WRITE(TRANSDATA_M1(pipe),
			   ((m_n.tu - 1) << PIPE_GMCH_DATA_M_TU_SIZE_SHIFT) |
			   m_n.gmch_m);
		I915_WRITE(TRANSDATA_N1(pipe), m_n.gmch_n);
		I915_WRITE(TRANSDPLINK_M1(pipe), m_n.link_m);
		I915_WRITE(TRANSDPLINK_N1(pipe), m_n.link_n);
	} else {
		I915_WRITE(PIPE_GMCH_DATA_M(pipe),
			   ((m_n.tu - 1) << PIPE_GMCH_DATA_M_TU_SIZE_SHIFT) |
			   m_n.gmch_m);
		I915_WRITE(PIPE_GMCH_DATA_N(pipe), m_n.gmch_n);
		I915_WRITE(PIPE_DP_LINK_M(pipe), m_n.link_m);
		I915_WRITE(PIPE_DP_LINK_N(pipe), m_n.link_n);
	}
}

static void ironlake_edp_pll_on(struct drm_encoder *encoder);
static void ironlake_edp_pll_off(struct drm_encoder *encoder);

static void
intel_dp_mode_set(struct drm_encoder *encoder, struct drm_display_mode *mode,
		  struct drm_display_mode *adjusted_mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_dp *intel_dp = enc_to_intel_dp(encoder);
	struct drm_crtc *crtc = intel_dp->base.base.crtc;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);

	/* Turn on the eDP PLL if needed */
	if (is_edp(intel_dp)) {
		if (!is_pch_edp(intel_dp))
			ironlake_edp_pll_on(encoder);
		else
			ironlake_edp_pll_off(encoder);
	}

	/*
	 * There are four kinds of DP registers:
	 *
	 * 	IBX PCH
	 * 	SNB CPU
	 *	IVB CPU
	 * 	CPT PCH
	 *
	 * IBX PCH and CPU are the same for almost everything,
	 * except that the CPU DP PLL is configured in this
	 * register
	 *
	 * CPT PCH is quite different, having many bits moved
	 * to the TRANS_DP_CTL register instead. That
	 * configuration happens (oddly) in ironlake_pch_enable
	 */

	/* Preserve the BIOS-computed detected bit. This is
	 * supposed to be read-only.
	 */
	intel_dp->DP = I915_READ(intel_dp->output_reg) & DP_DETECTED;
	intel_dp->DP |=  DP_VOLTAGE_0_4 | DP_PRE_EMPHASIS_0;

	/* Handle DP bits in common between all three register formats */

	intel_dp->DP |= DP_VOLTAGE_0_4 | DP_PRE_EMPHASIS_0;

	switch (intel_dp->lane_count) {
	case 1:
		intel_dp->DP |= DP_PORT_WIDTH_1;
		break;
	case 2:
		intel_dp->DP |= DP_PORT_WIDTH_2;
		break;
	case 4:
		intel_dp->DP |= DP_PORT_WIDTH_4;
		break;
	}
	if (intel_dp->has_audio) {
		DRM_DEBUG_DRIVER("Enabling DP audio on pipe %c\n",
				 pipe_name(intel_crtc->pipe));
		intel_dp->DP |= DP_AUDIO_OUTPUT_ENABLE;
		intel_write_eld(encoder, adjusted_mode);
	}
	memset(intel_dp->link_configuration, 0, DP_LINK_CONFIGURATION_SIZE);
	intel_dp->link_configuration[0] = intel_dp->link_bw;
	intel_dp->link_configuration[1] = intel_dp->lane_count;
	intel_dp->link_configuration[8] = DP_SET_ANSI_8B10B;
	/*
	 * Check for DPCD version > 1.1 and enhanced framing support
	 */
	if (intel_dp->dpcd[DP_DPCD_REV] >= 0x11 &&
	    (intel_dp->dpcd[DP_MAX_LANE_COUNT] & DP_ENHANCED_FRAME_CAP)) {
		intel_dp->link_configuration[1] |= DP_LANE_COUNT_ENHANCED_FRAME_EN;
	}

	/* Split out the IBX/CPU vs CPT settings */

	if (is_cpu_edp(intel_dp) && IS_GEN7(dev)) {
		if (adjusted_mode->flags & DRM_MODE_FLAG_PHSYNC)
			intel_dp->DP |= DP_SYNC_HS_HIGH;
		if (adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC)
			intel_dp->DP |= DP_SYNC_VS_HIGH;
		intel_dp->DP |= DP_LINK_TRAIN_OFF_CPT;

		if (intel_dp->link_configuration[1] & DP_LANE_COUNT_ENHANCED_FRAME_EN)
			intel_dp->DP |= DP_ENHANCED_FRAMING;

		intel_dp->DP |= intel_crtc->pipe << 29;

		/* don't miss out required setting for eDP */
		intel_dp->DP |= DP_PLL_ENABLE;
		if (adjusted_mode->clock < 200000)
			intel_dp->DP |= DP_PLL_FREQ_160MHZ;
		else
			intel_dp->DP |= DP_PLL_FREQ_270MHZ;
	} else if (!HAS_PCH_CPT(dev) || is_cpu_edp(intel_dp)) {
		intel_dp->DP |= intel_dp->color_range;

		if (adjusted_mode->flags & DRM_MODE_FLAG_PHSYNC)
			intel_dp->DP |= DP_SYNC_HS_HIGH;
		if (adjusted_mode->flags & DRM_MODE_FLAG_PVSYNC)
			intel_dp->DP |= DP_SYNC_VS_HIGH;
		intel_dp->DP |= DP_LINK_TRAIN_OFF;

		if (intel_dp->link_configuration[1] & DP_LANE_COUNT_ENHANCED_FRAME_EN)
			intel_dp->DP |= DP_ENHANCED_FRAMING;

		if (intel_crtc->pipe == 1)
			intel_dp->DP |= DP_PIPEB_SELECT;

		if (is_cpu_edp(intel_dp)) {
			/* don't miss out required setting for eDP */
			intel_dp->DP |= DP_PLL_ENABLE;
			if (adjusted_mode->clock < 200000)
				intel_dp->DP |= DP_PLL_FREQ_160MHZ;
			else
				intel_dp->DP |= DP_PLL_FREQ_270MHZ;
		}
	} else {
		intel_dp->DP |= DP_LINK_TRAIN_OFF_CPT;
	}
}

#define IDLE_ON_MASK		(PP_ON | 0 	  | PP_SEQUENCE_MASK | 0                     | PP_SEQUENCE_STATE_MASK)
#define IDLE_ON_VALUE   	(PP_ON | 0 	  | PP_SEQUENCE_NONE | 0                     | PP_SEQUENCE_STATE_ON_IDLE)

#define IDLE_OFF_MASK		(PP_ON | 0        | PP_SEQUENCE_MASK | 0                     | PP_SEQUENCE_STATE_MASK)
#define IDLE_OFF_VALUE		(0     | 0        | PP_SEQUENCE_NONE | 0                     | PP_SEQUENCE_STATE_OFF_IDLE)

#define IDLE_CYCLE_MASK		(PP_ON | 0        | PP_SEQUENCE_MASK | PP_CYCLE_DELAY_ACTIVE | PP_SEQUENCE_STATE_MASK)
#define IDLE_CYCLE_VALUE	(0     | 0        | PP_SEQUENCE_NONE | 0                     | PP_SEQUENCE_STATE_OFF_IDLE)

static void ironlake_wait_panel_status(struct intel_dp *intel_dp,
				       u32 mask,
				       u32 value)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	DRM_DEBUG_KMS("mask %08x value %08x status %08x control %08x\n",
		      mask, value,
		      I915_READ(PCH_PP_STATUS),
		      I915_READ(PCH_PP_CONTROL));

	if (_wait_for((I915_READ(PCH_PP_STATUS) & mask) == value, 5000, 10)) {
		DRM_ERROR("Panel status timeout: status %08x control %08x\n",
			  I915_READ(PCH_PP_STATUS),
			  I915_READ(PCH_PP_CONTROL));
	}
}

static void ironlake_wait_panel_on(struct intel_dp *intel_dp)
{
	DRM_DEBUG_KMS("Wait for panel power on\n");
	ironlake_wait_panel_status(intel_dp, IDLE_ON_MASK, IDLE_ON_VALUE);
}

static void ironlake_wait_panel_off(struct intel_dp *intel_dp)
{
	DRM_DEBUG_KMS("Wait for panel power off time\n");
	ironlake_wait_panel_status(intel_dp, IDLE_OFF_MASK, IDLE_OFF_VALUE);
}

static void ironlake_wait_panel_power_cycle(struct intel_dp *intel_dp)
{
	DRM_DEBUG_KMS("Wait for panel power cycle\n");
	ironlake_wait_panel_status(intel_dp, IDLE_CYCLE_MASK, IDLE_CYCLE_VALUE);
}


/* Read the current pp_control value, unlocking the register if it
 * is locked
 */

static  u32 ironlake_get_pp_control(struct drm_i915_private *dev_priv)
{
	u32	control = I915_READ(PCH_PP_CONTROL);

	control &= ~PANEL_UNLOCK_MASK;
	control |= PANEL_UNLOCK_REGS;
	return control;
}

static void ironlake_edp_panel_vdd_on(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 pp;

	if (!is_edp(intel_dp))
		return;
	DRM_DEBUG_KMS("Turn eDP VDD on\n");

	WARN(intel_dp->want_panel_vdd,
	     "eDP VDD already requested on\n");

	intel_dp->want_panel_vdd = true;

	if (ironlake_edp_have_panel_vdd(intel_dp)) {
		DRM_DEBUG_KMS("eDP VDD already on\n");
		return;
	}

	if (!ironlake_edp_have_panel_power(intel_dp))
		ironlake_wait_panel_power_cycle(intel_dp);

	pp = ironlake_get_pp_control(dev_priv);
	pp |= EDP_FORCE_VDD;
	I915_WRITE(PCH_PP_CONTROL, pp);
	POSTING_READ(PCH_PP_CONTROL);
	DRM_DEBUG_KMS("PCH_PP_STATUS: 0x%08x PCH_PP_CONTROL: 0x%08x\n",
		      I915_READ(PCH_PP_STATUS), I915_READ(PCH_PP_CONTROL));

	/*
	 * If the panel wasn't on, delay before accessing aux channel
	 */
	if (!ironlake_edp_have_panel_power(intel_dp)) {
		DRM_DEBUG_KMS("eDP was not running\n");
		msleep(intel_dp->panel_power_up_delay);
	}
}

static void ironlake_panel_vdd_off_sync(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 pp;

	if (!intel_dp->want_panel_vdd && ironlake_edp_have_panel_vdd(intel_dp)) {
		pp = ironlake_get_pp_control(dev_priv);
		pp &= ~EDP_FORCE_VDD;
		I915_WRITE(PCH_PP_CONTROL, pp);
		POSTING_READ(PCH_PP_CONTROL);

		/* Make sure sequencer is idle before allowing subsequent activity */
		DRM_DEBUG_KMS("PCH_PP_STATUS: 0x%08x PCH_PP_CONTROL: 0x%08x\n",
			      I915_READ(PCH_PP_STATUS), I915_READ(PCH_PP_CONTROL));

		msleep(intel_dp->panel_power_down_delay);
	}
}

static void ironlake_panel_vdd_work(struct work_struct *__work)
{
	struct intel_dp *intel_dp = container_of(to_delayed_work(__work),
						 struct intel_dp, panel_vdd_work);
	struct drm_device *dev = intel_dp->base.base.dev;

	mutex_lock(&dev->mode_config.mutex);
	ironlake_panel_vdd_off_sync(intel_dp);
	mutex_unlock(&dev->mode_config.mutex);
}

static void ironlake_edp_panel_vdd_off(struct intel_dp *intel_dp, bool sync)
{
	if (!is_edp(intel_dp))
		return;

	DRM_DEBUG_KMS("Turn eDP VDD off %d\n", intel_dp->want_panel_vdd);
	WARN(!intel_dp->want_panel_vdd, "eDP VDD not forced on");

	intel_dp->want_panel_vdd = false;

	if (sync) {
		ironlake_panel_vdd_off_sync(intel_dp);
	} else {
		/*
		 * Queue the timer to fire a long
		 * time from now (relative to the power down delay)
		 * to keep the panel power up across a sequence of operations
		 */
		schedule_delayed_work(&intel_dp->panel_vdd_work,
				      msecs_to_jiffies(intel_dp->panel_power_cycle_delay * 5));
	}
}

static void ironlake_edp_panel_on(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 pp;

	if (!is_edp(intel_dp))
		return;

	DRM_DEBUG_KMS("Turn eDP power on\n");

	if (ironlake_edp_have_panel_power(intel_dp)) {
		DRM_DEBUG_KMS("eDP power already on\n");
		return;
	}

	ironlake_wait_panel_power_cycle(intel_dp);

	pp = ironlake_get_pp_control(dev_priv);
	if (IS_GEN5(dev)) {
		/* ILK workaround: disable reset around power sequence */
		pp &= ~PANEL_POWER_RESET;
		I915_WRITE(PCH_PP_CONTROL, pp);
		POSTING_READ(PCH_PP_CONTROL);
	}

	pp |= POWER_TARGET_ON;
	if (!IS_GEN5(dev))
		pp |= PANEL_POWER_RESET;

	I915_WRITE(PCH_PP_CONTROL, pp);
	POSTING_READ(PCH_PP_CONTROL);

	ironlake_wait_panel_on(intel_dp);

	if (IS_GEN5(dev)) {
		pp |= PANEL_POWER_RESET; /* restore panel reset bit */
		I915_WRITE(PCH_PP_CONTROL, pp);
		POSTING_READ(PCH_PP_CONTROL);
	}
}

static void ironlake_edp_panel_off(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 pp;

	if (!is_edp(intel_dp))
		return;

	DRM_DEBUG_KMS("Turn eDP power off\n");

	WARN(intel_dp->want_panel_vdd, "Cannot turn power off while VDD is on\n");

	pp = ironlake_get_pp_control(dev_priv);
	pp &= ~(POWER_TARGET_ON | EDP_FORCE_VDD | PANEL_POWER_RESET | EDP_BLC_ENABLE);
	I915_WRITE(PCH_PP_CONTROL, pp);
	POSTING_READ(PCH_PP_CONTROL);

	ironlake_wait_panel_off(intel_dp);
}

static void ironlake_edp_backlight_on(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 pp;

	if (!is_edp(intel_dp))
		return;

	DRM_DEBUG_KMS("\n");
	/*
	 * If we enable the backlight right away following a panel power
	 * on, we may see slight flicker as the panel syncs with the eDP
	 * link.  So delay a bit to make sure the image is solid before
	 * allowing it to appear.
	 */
	msleep(intel_dp->backlight_on_delay);
	pp = ironlake_get_pp_control(dev_priv);
	pp |= EDP_BLC_ENABLE;
	I915_WRITE(PCH_PP_CONTROL, pp);
	POSTING_READ(PCH_PP_CONTROL);
}

static void ironlake_edp_backlight_off(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 pp;

	if (!is_edp(intel_dp))
		return;

	DRM_DEBUG_KMS("\n");
	pp = ironlake_get_pp_control(dev_priv);
	pp &= ~EDP_BLC_ENABLE;
	I915_WRITE(PCH_PP_CONTROL, pp);
	POSTING_READ(PCH_PP_CONTROL);
	msleep(intel_dp->backlight_off_delay);
}

static void ironlake_edp_pll_on(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 dpa_ctl;

	DRM_DEBUG_KMS("\n");
	dpa_ctl = I915_READ(DP_A);
	dpa_ctl |= DP_PLL_ENABLE;
	I915_WRITE(DP_A, dpa_ctl);
	POSTING_READ(DP_A);
	udelay(200);
}

static void ironlake_edp_pll_off(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 dpa_ctl;

	dpa_ctl = I915_READ(DP_A);
	dpa_ctl &= ~DP_PLL_ENABLE;
	I915_WRITE(DP_A, dpa_ctl);
	POSTING_READ(DP_A);
	udelay(200);
}

/* If the sink supports it, try to set the power state appropriately */
static void intel_dp_sink_dpms(struct intel_dp *intel_dp, int mode)
{
	int ret, i;

	/* Should have a valid DPCD by this point */
	if (intel_dp->dpcd[DP_DPCD_REV] < 0x11)
		return;

	if (mode != DRM_MODE_DPMS_ON) {
		ret = intel_dp_aux_native_write_1(intel_dp, DP_SET_POWER,
						  DP_SET_POWER_D3);
		if (ret != 1)
			DRM_DEBUG_DRIVER("failed to write sink power state\n");
	} else {
		/*
		 * When turning on, we need to retry for 1ms to give the sink
		 * time to wake up.
		 */
		for (i = 0; i < 3; i++) {
			ret = intel_dp_aux_native_write_1(intel_dp,
							  DP_SET_POWER,
							  DP_SET_POWER_D0);
			if (ret == 1)
				break;
			msleep(1);
		}
	}
}

static void intel_dp_prepare(struct drm_encoder *encoder)
{
	struct intel_dp *intel_dp = enc_to_intel_dp(encoder);

	ironlake_edp_backlight_off(intel_dp);
	ironlake_edp_panel_off(intel_dp);

	/* Wake up the sink first */
	ironlake_edp_panel_vdd_on(intel_dp);
	intel_dp_sink_dpms(intel_dp, DRM_MODE_DPMS_ON);
	intel_dp_link_down(intel_dp);
	ironlake_edp_panel_vdd_off(intel_dp, false);

	/* Make sure the panel is off before trying to
	 * change the mode
	 */
}

static void intel_dp_commit(struct drm_encoder *encoder)
{
	struct intel_dp *intel_dp = enc_to_intel_dp(encoder);
	struct drm_device *dev = encoder->dev;
	struct intel_crtc *intel_crtc = to_intel_crtc(intel_dp->base.base.crtc);

	ironlake_edp_panel_vdd_on(intel_dp);
	intel_dp_sink_dpms(intel_dp, DRM_MODE_DPMS_ON);
	intel_dp_start_link_train(intel_dp);
	ironlake_edp_panel_on(intel_dp);
	ironlake_edp_panel_vdd_off(intel_dp, true);
	intel_dp_complete_link_train(intel_dp);
	ironlake_edp_backlight_on(intel_dp);

	intel_dp->dpms_mode = DRM_MODE_DPMS_ON;

	if (HAS_PCH_CPT(dev))
		intel_cpt_verify_modeset(dev, intel_crtc->pipe);
}

static void
intel_dp_dpms(struct drm_encoder *encoder, int mode)
{
	struct intel_dp *intel_dp = enc_to_intel_dp(encoder);
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t dp_reg = I915_READ(intel_dp->output_reg);

	if (mode != DRM_MODE_DPMS_ON) {
		ironlake_edp_backlight_off(intel_dp);
		ironlake_edp_panel_off(intel_dp);

		ironlake_edp_panel_vdd_on(intel_dp);
		intel_dp_sink_dpms(intel_dp, mode);
		intel_dp_link_down(intel_dp);
		ironlake_edp_panel_vdd_off(intel_dp, false);

		if (is_cpu_edp(intel_dp))
			ironlake_edp_pll_off(encoder);
	} else {
		if (is_cpu_edp(intel_dp))
			ironlake_edp_pll_on(encoder);

		ironlake_edp_panel_vdd_on(intel_dp);
		intel_dp_sink_dpms(intel_dp, mode);
		if (!(dp_reg & DP_PORT_EN)) {
			intel_dp_start_link_train(intel_dp);
			ironlake_edp_panel_on(intel_dp);
			ironlake_edp_panel_vdd_off(intel_dp, true);
			intel_dp_complete_link_train(intel_dp);
		} else
			ironlake_edp_panel_vdd_off(intel_dp, false);
		ironlake_edp_backlight_on(intel_dp);
	}
	intel_dp->dpms_mode = mode;
}

/*
 * Native read with retry for link status and receiver capability reads for
 * cases where the sink may still be asleep.
 */
static bool
intel_dp_aux_native_read_retry(struct intel_dp *intel_dp, uint16_t address,
			       uint8_t *recv, int recv_bytes)
{
	int ret, i;

	/*
	 * Sinks are *supposed* to come up within 1ms from an off state,
	 * but we're also supposed to retry 3 times per the spec.
	 */
	for (i = 0; i < 3; i++) {
		ret = intel_dp_aux_native_read(intel_dp, address, recv,
					       recv_bytes);
		if (ret == recv_bytes)
			return true;
		msleep(1);
	}

	return false;
}

/*
 * Fetch AUX CH registers 0x202 - 0x207 which contain
 * link status information
 */
static bool
intel_dp_get_link_status(struct intel_dp *intel_dp, uint8_t link_status[DP_LINK_STATUS_SIZE])
{
	return intel_dp_aux_native_read_retry(intel_dp,
					      DP_LANE0_1_STATUS,
					      link_status,
					      DP_LINK_STATUS_SIZE);
}

static uint8_t
intel_dp_link_status(uint8_t link_status[DP_LINK_STATUS_SIZE],
		     int r)
{
	return link_status[r - DP_LANE0_1_STATUS];
}

static uint8_t
intel_get_adjust_request_voltage(uint8_t adjust_request[2],
				 int lane)
{
	int	    s = ((lane & 1) ?
			 DP_ADJUST_VOLTAGE_SWING_LANE1_SHIFT :
			 DP_ADJUST_VOLTAGE_SWING_LANE0_SHIFT);
	uint8_t l = adjust_request[lane>>1];

	return ((l >> s) & 3) << DP_TRAIN_VOLTAGE_SWING_SHIFT;
}

static uint8_t
intel_get_adjust_request_pre_emphasis(uint8_t adjust_request[2],
				      int lane)
{
	int	    s = ((lane & 1) ?
			 DP_ADJUST_PRE_EMPHASIS_LANE1_SHIFT :
			 DP_ADJUST_PRE_EMPHASIS_LANE0_SHIFT);
	uint8_t l = adjust_request[lane>>1];

	return ((l >> s) & 3) << DP_TRAIN_PRE_EMPHASIS_SHIFT;
}


#if 0
static char	*voltage_names[] = {
	"0.4V", "0.6V", "0.8V", "1.2V"
};
static char	*pre_emph_names[] = {
	"0dB", "3.5dB", "6dB", "9.5dB"
};
static char	*link_train_names[] = {
	"pattern 1", "pattern 2", "idle", "off"
};
#endif

/*
 * These are source-specific values; current Intel hardware supports
 * a maximum voltage of 800mV and a maximum pre-emphasis of 6dB
 */

static uint8_t
intel_dp_voltage_max(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;

	if (IS_GEN7(dev) && is_cpu_edp(intel_dp))
		return DP_TRAIN_VOLTAGE_SWING_800;
	else if (HAS_PCH_CPT(dev) && !is_cpu_edp(intel_dp))
		return DP_TRAIN_VOLTAGE_SWING_1200;
	else
		return DP_TRAIN_VOLTAGE_SWING_800;
}

static uint8_t
intel_dp_pre_emphasis_max(struct intel_dp *intel_dp, uint8_t voltage_swing)
{
	struct drm_device *dev = intel_dp->base.base.dev;

	if (IS_GEN7(dev) && is_cpu_edp(intel_dp)) {
		switch (voltage_swing & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_400:
			return DP_TRAIN_PRE_EMPHASIS_6;
		case DP_TRAIN_VOLTAGE_SWING_600:
		case DP_TRAIN_VOLTAGE_SWING_800:
			return DP_TRAIN_PRE_EMPHASIS_3_5;
		default:
			return DP_TRAIN_PRE_EMPHASIS_0;
		}
	} else {
		switch (voltage_swing & DP_TRAIN_VOLTAGE_SWING_MASK) {
		case DP_TRAIN_VOLTAGE_SWING_400:
			return DP_TRAIN_PRE_EMPHASIS_6;
		case DP_TRAIN_VOLTAGE_SWING_600:
			return DP_TRAIN_PRE_EMPHASIS_6;
		case DP_TRAIN_VOLTAGE_SWING_800:
			return DP_TRAIN_PRE_EMPHASIS_3_5;
		case DP_TRAIN_VOLTAGE_SWING_1200:
		default:
			return DP_TRAIN_PRE_EMPHASIS_0;
		}
	}
}

static void
intel_get_adjust_train(struct intel_dp *intel_dp, uint8_t link_status[DP_LINK_STATUS_SIZE])
{
	uint8_t v = 0;
	uint8_t p = 0;
	int lane;
	uint8_t	*adjust_request = link_status + (DP_ADJUST_REQUEST_LANE0_1 - DP_LANE0_1_STATUS);
	uint8_t voltage_max;
	uint8_t preemph_max;

	for (lane = 0; lane < intel_dp->lane_count; lane++) {
		uint8_t this_v = intel_get_adjust_request_voltage(adjust_request, lane);
		uint8_t this_p = intel_get_adjust_request_pre_emphasis(adjust_request, lane);

		if (this_v > v)
			v = this_v;
		if (this_p > p)
			p = this_p;
	}

	voltage_max = intel_dp_voltage_max(intel_dp);
	if (v >= voltage_max)
		v = voltage_max | DP_TRAIN_MAX_SWING_REACHED;

	preemph_max = intel_dp_pre_emphasis_max(intel_dp, v);
	if (p >= preemph_max)
		p = preemph_max | DP_TRAIN_MAX_PRE_EMPHASIS_REACHED;

	for (lane = 0; lane < 4; lane++)
		intel_dp->train_set[lane] = v | p;
}

static uint32_t
intel_dp_signal_levels(uint8_t train_set)
{
	uint32_t	signal_levels = 0;

	switch (train_set & DP_TRAIN_VOLTAGE_SWING_MASK) {
	case DP_TRAIN_VOLTAGE_SWING_400:
	default:
		signal_levels |= DP_VOLTAGE_0_4;
		break;
	case DP_TRAIN_VOLTAGE_SWING_600:
		signal_levels |= DP_VOLTAGE_0_6;
		break;
	case DP_TRAIN_VOLTAGE_SWING_800:
		signal_levels |= DP_VOLTAGE_0_8;
		break;
	case DP_TRAIN_VOLTAGE_SWING_1200:
		signal_levels |= DP_VOLTAGE_1_2;
		break;
	}
	switch (train_set & DP_TRAIN_PRE_EMPHASIS_MASK) {
	case DP_TRAIN_PRE_EMPHASIS_0:
	default:
		signal_levels |= DP_PRE_EMPHASIS_0;
		break;
	case DP_TRAIN_PRE_EMPHASIS_3_5:
		signal_levels |= DP_PRE_EMPHASIS_3_5;
		break;
	case DP_TRAIN_PRE_EMPHASIS_6:
		signal_levels |= DP_PRE_EMPHASIS_6;
		break;
	case DP_TRAIN_PRE_EMPHASIS_9_5:
		signal_levels |= DP_PRE_EMPHASIS_9_5;
		break;
	}
	return signal_levels;
}

/* Gen6's DP voltage swing and pre-emphasis control */
static uint32_t
intel_gen6_edp_signal_levels(uint8_t train_set)
{
	int signal_levels = train_set & (DP_TRAIN_VOLTAGE_SWING_MASK |
					 DP_TRAIN_PRE_EMPHASIS_MASK);
	switch (signal_levels) {
	case DP_TRAIN_VOLTAGE_SWING_400 | DP_TRAIN_PRE_EMPHASIS_0:
	case DP_TRAIN_VOLTAGE_SWING_600 | DP_TRAIN_PRE_EMPHASIS_0:
		return EDP_LINK_TRAIN_400_600MV_0DB_SNB_B;
	case DP_TRAIN_VOLTAGE_SWING_400 | DP_TRAIN_PRE_EMPHASIS_3_5:
		return EDP_LINK_TRAIN_400MV_3_5DB_SNB_B;
	case DP_TRAIN_VOLTAGE_SWING_400 | DP_TRAIN_PRE_EMPHASIS_6:
	case DP_TRAIN_VOLTAGE_SWING_600 | DP_TRAIN_PRE_EMPHASIS_6:
		return EDP_LINK_TRAIN_400_600MV_6DB_SNB_B;
	case DP_TRAIN_VOLTAGE_SWING_600 | DP_TRAIN_PRE_EMPHASIS_3_5:
	case DP_TRAIN_VOLTAGE_SWING_800 | DP_TRAIN_PRE_EMPHASIS_3_5:
		return EDP_LINK_TRAIN_600_800MV_3_5DB_SNB_B;
	case DP_TRAIN_VOLTAGE_SWING_800 | DP_TRAIN_PRE_EMPHASIS_0:
	case DP_TRAIN_VOLTAGE_SWING_1200 | DP_TRAIN_PRE_EMPHASIS_0:
		return EDP_LINK_TRAIN_800_1200MV_0DB_SNB_B;
	default:
		DRM_DEBUG_KMS("Unsupported voltage swing/pre-emphasis level:"
			      "0x%x\n", signal_levels);
		return EDP_LINK_TRAIN_400_600MV_0DB_SNB_B;
	}
}

/* Gen7's DP voltage swing and pre-emphasis control */
static uint32_t
intel_gen7_edp_signal_levels(uint8_t train_set)
{
	int signal_levels = train_set & (DP_TRAIN_VOLTAGE_SWING_MASK |
					 DP_TRAIN_PRE_EMPHASIS_MASK);
	switch (signal_levels) {
	case DP_TRAIN_VOLTAGE_SWING_400 | DP_TRAIN_PRE_EMPHASIS_0:
		return EDP_LINK_TRAIN_400MV_0DB_IVB;
	case DP_TRAIN_VOLTAGE_SWING_400 | DP_TRAIN_PRE_EMPHASIS_3_5:
		return EDP_LINK_TRAIN_400MV_3_5DB_IVB;
	case DP_TRAIN_VOLTAGE_SWING_400 | DP_TRAIN_PRE_EMPHASIS_6:
		return EDP_LINK_TRAIN_400MV_6DB_IVB;

	case DP_TRAIN_VOLTAGE_SWING_600 | DP_TRAIN_PRE_EMPHASIS_0:
		return EDP_LINK_TRAIN_600MV_0DB_IVB;
	case DP_TRAIN_VOLTAGE_SWING_600 | DP_TRAIN_PRE_EMPHASIS_3_5:
		return EDP_LINK_TRAIN_600MV_3_5DB_IVB;

	case DP_TRAIN_VOLTAGE_SWING_800 | DP_TRAIN_PRE_EMPHASIS_0:
		return EDP_LINK_TRAIN_800MV_0DB_IVB;
	case DP_TRAIN_VOLTAGE_SWING_800 | DP_TRAIN_PRE_EMPHASIS_3_5:
		return EDP_LINK_TRAIN_800MV_3_5DB_IVB;

	default:
		DRM_DEBUG_KMS("Unsupported voltage swing/pre-emphasis level:"
			      "0x%x\n", signal_levels);
		return EDP_LINK_TRAIN_500MV_0DB_IVB;
	}
}

static uint8_t
intel_get_lane_status(uint8_t link_status[DP_LINK_STATUS_SIZE],
		      int lane)
{
	int s = (lane & 1) * 4;
	uint8_t l = link_status[lane>>1];

	return (l >> s) & 0xf;
}

/* Check for clock recovery is done on all channels */
static bool
intel_clock_recovery_ok(uint8_t link_status[DP_LINK_STATUS_SIZE], int lane_count)
{
	int lane;
	uint8_t lane_status;

	for (lane = 0; lane < lane_count; lane++) {
		lane_status = intel_get_lane_status(link_status, lane);
		if ((lane_status & DP_LANE_CR_DONE) == 0)
			return false;
	}
	return true;
}

/* Check to see if channel eq is done on all channels */
#define CHANNEL_EQ_BITS (DP_LANE_CR_DONE|\
			 DP_LANE_CHANNEL_EQ_DONE|\
			 DP_LANE_SYMBOL_LOCKED)
static bool
intel_channel_eq_ok(struct intel_dp *intel_dp, uint8_t link_status[DP_LINK_STATUS_SIZE])
{
	uint8_t lane_align;
	uint8_t lane_status;
	int lane;

	lane_align = intel_dp_link_status(link_status,
					  DP_LANE_ALIGN_STATUS_UPDATED);
	if ((lane_align & DP_INTERLANE_ALIGN_DONE) == 0)
		return false;
	for (lane = 0; lane < intel_dp->lane_count; lane++) {
		lane_status = intel_get_lane_status(link_status, lane);
		if ((lane_status & CHANNEL_EQ_BITS) != CHANNEL_EQ_BITS)
			return false;
	}
	return true;
}

static bool
intel_dp_set_link_train(struct intel_dp *intel_dp,
			uint32_t dp_reg_value,
			uint8_t dp_train_pat)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	I915_WRITE(intel_dp->output_reg, dp_reg_value);
	POSTING_READ(intel_dp->output_reg);

	intel_dp_aux_native_write_1(intel_dp,
				    DP_TRAINING_PATTERN_SET,
				    dp_train_pat);

	ret = intel_dp_aux_native_write(intel_dp,
					DP_TRAINING_LANE0_SET,
					intel_dp->train_set,
					intel_dp->lane_count);
	if (ret != intel_dp->lane_count)
		return false;

	return true;
}

/* Enable corresponding port and start training pattern 1 */
static void
intel_dp_start_link_train(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(intel_dp->base.base.crtc);
	int i;
	uint8_t voltage;
	bool clock_recovery = false;
	int voltage_tries, loop_tries;
	u32 reg;
	uint32_t DP = intel_dp->DP;

	/*
	 * On CPT we have to enable the port in training pattern 1, which
	 * will happen below in intel_dp_set_link_train.  Otherwise, enable
	 * the port and wait for it to become active.
	 */
	if (!HAS_PCH_CPT(dev)) {
		I915_WRITE(intel_dp->output_reg, intel_dp->DP);
		POSTING_READ(intel_dp->output_reg);
		intel_wait_for_vblank(dev, intel_crtc->pipe);
	}

	/* Write the link configuration data */
	intel_dp_aux_native_write(intel_dp, DP_LINK_BW_SET,
				  intel_dp->link_configuration,
				  DP_LINK_CONFIGURATION_SIZE);

	DP |= DP_PORT_EN;

	if (HAS_PCH_CPT(dev) && (IS_GEN7(dev) || !is_cpu_edp(intel_dp)))
		DP &= ~DP_LINK_TRAIN_MASK_CPT;
	else
		DP &= ~DP_LINK_TRAIN_MASK;
	memset(intel_dp->train_set, 0, 4);
	voltage = 0xff;
	voltage_tries = 0;
	loop_tries = 0;
	clock_recovery = false;
	for (;;) {
		/* Use intel_dp->train_set[0] to set the voltage and pre emphasis values */
		uint8_t	    link_status[DP_LINK_STATUS_SIZE];
		uint32_t    signal_levels;


		if (IS_GEN7(dev) && is_cpu_edp(intel_dp)) {
			signal_levels = intel_gen7_edp_signal_levels(intel_dp->train_set[0]);
			DP = (DP & ~EDP_LINK_TRAIN_VOL_EMP_MASK_IVB) | signal_levels;
		} else if (IS_GEN6(dev) && is_cpu_edp(intel_dp)) {
			signal_levels = intel_gen6_edp_signal_levels(intel_dp->train_set[0]);
			DP = (DP & ~EDP_LINK_TRAIN_VOL_EMP_MASK_SNB) | signal_levels;
		} else {
			signal_levels = intel_dp_signal_levels(intel_dp->train_set[0]);
			DRM_DEBUG_KMS("training pattern 1 signal levels %08x\n", signal_levels);
			DP = (DP & ~(DP_VOLTAGE_MASK|DP_PRE_EMPHASIS_MASK)) | signal_levels;
		}

		if (HAS_PCH_CPT(dev) && (IS_GEN7(dev) || !is_cpu_edp(intel_dp)))
			reg = DP | DP_LINK_TRAIN_PAT_1_CPT;
		else
			reg = DP | DP_LINK_TRAIN_PAT_1;

		if (!intel_dp_set_link_train(intel_dp, reg,
					     DP_TRAINING_PATTERN_1 |
					     DP_LINK_SCRAMBLING_DISABLE))
			break;
		/* Set training pattern 1 */

		udelay(100);
		if (!intel_dp_get_link_status(intel_dp, link_status)) {
			DRM_ERROR("failed to get link status\n");
			break;
		}

		if (intel_clock_recovery_ok(link_status, intel_dp->lane_count)) {
			DRM_DEBUG_KMS("clock recovery OK\n");
			clock_recovery = true;
			break;
		}

		/* Check to see if we've tried the max voltage */
		for (i = 0; i < intel_dp->lane_count; i++)
			if ((intel_dp->train_set[i] & DP_TRAIN_MAX_SWING_REACHED) == 0)
				break;
		if (i == intel_dp->lane_count) {
			++loop_tries;
			if (loop_tries == 5) {
				DRM_DEBUG_KMS("too many full retries, give up\n");
				break;
			}
			memset(intel_dp->train_set, 0, 4);
			voltage_tries = 0;
			continue;
		}

		/* Check to see if we've tried the same voltage 5 times */
		if ((intel_dp->train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK) == voltage) {
			++voltage_tries;
			if (voltage_tries == 5) {
				DRM_DEBUG_KMS("too many voltage retries, give up\n");
				break;
			}
		} else
			voltage_tries = 0;
		voltage = intel_dp->train_set[0] & DP_TRAIN_VOLTAGE_SWING_MASK;

		/* Compute new intel_dp->train_set as requested by target */
		intel_get_adjust_train(intel_dp, link_status);
	}

	intel_dp->DP = DP;
}

static void
intel_dp_complete_link_train(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	bool channel_eq = false;
	int tries, cr_tries;
	u32 reg;
	uint32_t DP = intel_dp->DP;

	/* channel equalization */
	tries = 0;
	cr_tries = 0;
	channel_eq = false;
	for (;;) {
		/* Use intel_dp->train_set[0] to set the voltage and pre emphasis values */
		uint32_t    signal_levels;
		uint8_t	    link_status[DP_LINK_STATUS_SIZE];

		if (cr_tries > 5) {
			DRM_ERROR("failed to train DP, aborting\n");
			intel_dp_link_down(intel_dp);
			break;
		}

		if (IS_GEN7(dev) && is_cpu_edp(intel_dp)) {
			signal_levels = intel_gen7_edp_signal_levels(intel_dp->train_set[0]);
			DP = (DP & ~EDP_LINK_TRAIN_VOL_EMP_MASK_IVB) | signal_levels;
		} else if (IS_GEN6(dev) && is_cpu_edp(intel_dp)) {
			signal_levels = intel_gen6_edp_signal_levels(intel_dp->train_set[0]);
			DP = (DP & ~EDP_LINK_TRAIN_VOL_EMP_MASK_SNB) | signal_levels;
		} else {
			signal_levels = intel_dp_signal_levels(intel_dp->train_set[0]);
			DP = (DP & ~(DP_VOLTAGE_MASK|DP_PRE_EMPHASIS_MASK)) | signal_levels;
		}

		if (HAS_PCH_CPT(dev) && (IS_GEN7(dev) || !is_cpu_edp(intel_dp)))
			reg = DP | DP_LINK_TRAIN_PAT_2_CPT;
		else
			reg = DP | DP_LINK_TRAIN_PAT_2;

		/* channel eq pattern */
		if (!intel_dp_set_link_train(intel_dp, reg,
					     DP_TRAINING_PATTERN_2 |
					     DP_LINK_SCRAMBLING_DISABLE))
			break;

		udelay(400);
		if (!intel_dp_get_link_status(intel_dp, link_status))
			break;

		/* Make sure clock is still ok */
		if (!intel_clock_recovery_ok(link_status, intel_dp->lane_count)) {
			intel_dp_start_link_train(intel_dp);
			cr_tries++;
			continue;
		}

		if (intel_channel_eq_ok(intel_dp, link_status)) {
			channel_eq = true;
			break;
		}

		/* Try 5 times, then try clock recovery if that fails */
		if (tries > 5) {
			intel_dp_link_down(intel_dp);
			intel_dp_start_link_train(intel_dp);
			tries = 0;
			cr_tries++;
			continue;
		}

		/* Compute new intel_dp->train_set as requested by target */
		intel_get_adjust_train(intel_dp, link_status);
		++tries;
	}

	if (HAS_PCH_CPT(dev) && (IS_GEN7(dev) || !is_cpu_edp(intel_dp)))
		reg = DP | DP_LINK_TRAIN_OFF_CPT;
	else
		reg = DP | DP_LINK_TRAIN_OFF;

	I915_WRITE(intel_dp->output_reg, reg);
	POSTING_READ(intel_dp->output_reg);
	intel_dp_aux_native_write_1(intel_dp,
				    DP_TRAINING_PATTERN_SET, DP_TRAINING_PATTERN_DISABLE);
}

static void
intel_dp_link_down(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t DP = intel_dp->DP;

	if ((I915_READ(intel_dp->output_reg) & DP_PORT_EN) == 0)
		return;

	DRM_DEBUG_KMS("\n");

	if (is_edp(intel_dp)) {
		DP &= ~DP_PLL_ENABLE;
		I915_WRITE(intel_dp->output_reg, DP);
		POSTING_READ(intel_dp->output_reg);
		udelay(100);
	}

	if (HAS_PCH_CPT(dev) && (IS_GEN7(dev) || !is_cpu_edp(intel_dp))) {
		DP &= ~DP_LINK_TRAIN_MASK_CPT;
		I915_WRITE(intel_dp->output_reg, DP | DP_LINK_TRAIN_PAT_IDLE_CPT);
	} else {
		DP &= ~DP_LINK_TRAIN_MASK;
		I915_WRITE(intel_dp->output_reg, DP | DP_LINK_TRAIN_PAT_IDLE);
	}
	POSTING_READ(intel_dp->output_reg);

	msleep(17);

	if (is_edp(intel_dp)) {
		if (HAS_PCH_CPT(dev) && (IS_GEN7(dev) || !is_cpu_edp(intel_dp)))
			DP |= DP_LINK_TRAIN_OFF_CPT;
		else
			DP |= DP_LINK_TRAIN_OFF;
	}

	if (!HAS_PCH_CPT(dev) &&
	    I915_READ(intel_dp->output_reg) & DP_PIPEB_SELECT) {
		struct drm_crtc *crtc = intel_dp->base.base.crtc;

		/* Hardware workaround: leaving our transcoder select
		 * set to transcoder B while it's off will prevent the
		 * corresponding HDMI output on transcoder A.
		 *
		 * Combine this with another hardware workaround:
		 * transcoder select bit can only be cleared while the
		 * port is enabled.
		 */
		DP &= ~DP_PIPEB_SELECT;
		I915_WRITE(intel_dp->output_reg, DP);

		/* Changes to enable or select take place the vblank
		 * after being written.
		 */
		if (crtc == NULL) {
			/* We can arrive here never having been attached
			 * to a CRTC, for instance, due to inheriting
			 * random state from the BIOS.
			 *
			 * If the pipe is not running, play safe and
			 * wait for the clocks to stabilise before
			 * continuing.
			 */
			POSTING_READ(intel_dp->output_reg);
			msleep(50);
		} else
			intel_wait_for_vblank(dev, to_intel_crtc(crtc)->pipe);
	}

	DP &= ~DP_AUDIO_OUTPUT_ENABLE;
	I915_WRITE(intel_dp->output_reg, DP & ~DP_PORT_EN);
	POSTING_READ(intel_dp->output_reg);
	msleep(intel_dp->panel_power_down_delay);
}

static bool
intel_dp_get_dpcd(struct intel_dp *intel_dp)
{
	if (intel_dp_aux_native_read_retry(intel_dp, 0x000, intel_dp->dpcd,
					   sizeof(intel_dp->dpcd)) &&
	    (intel_dp->dpcd[DP_DPCD_REV] != 0)) {
		return true;
	}

	return false;
}

static bool
intel_dp_get_sink_irq(struct intel_dp *intel_dp, u8 *sink_irq_vector)
{
	int ret;

	ret = intel_dp_aux_native_read_retry(intel_dp,
					     DP_DEVICE_SERVICE_IRQ_VECTOR,
					     sink_irq_vector, 1);
	if (!ret)
		return false;

	return true;
}

static void
intel_dp_handle_test_request(struct intel_dp *intel_dp)
{
	/* NAK by default */
	intel_dp_aux_native_write_1(intel_dp, DP_TEST_RESPONSE, DP_TEST_ACK);
}

/*
 * According to DP spec
 * 5.1.2:
 *  1. Read DPCD
 *  2. Configure link according to Receiver Capabilities
 *  3. Use Link Training from 2.5.3.3 and 3.5.1.3
 *  4. Check link status on receipt of hot-plug interrupt
 */

static void
intel_dp_check_link_status(struct intel_dp *intel_dp)
{
	u8 sink_irq_vector;
	u8 link_status[DP_LINK_STATUS_SIZE];

	if (intel_dp->dpms_mode != DRM_MODE_DPMS_ON)
		return;

	if (!intel_dp->base.base.crtc)
		return;

	/* Try to read receiver status if the link appears to be up */
	if (!intel_dp_get_link_status(intel_dp, link_status)) {
		intel_dp_link_down(intel_dp);
		return;
	}

	/* Now read the DPCD to see if it's actually running */
	if (!intel_dp_get_dpcd(intel_dp)) {
		intel_dp_link_down(intel_dp);
		return;
	}

	/* Try to read the source of the interrupt */
	if (intel_dp->dpcd[DP_DPCD_REV] >= 0x11 &&
	    intel_dp_get_sink_irq(intel_dp, &sink_irq_vector)) {
		/* Clear interrupt source */
		intel_dp_aux_native_write_1(intel_dp,
					    DP_DEVICE_SERVICE_IRQ_VECTOR,
					    sink_irq_vector);

		if (sink_irq_vector & DP_AUTOMATED_TEST_REQUEST)
			intel_dp_handle_test_request(intel_dp);
		if (sink_irq_vector & (DP_CP_IRQ | DP_SINK_SPECIFIC_IRQ))
			DRM_DEBUG_DRIVER("CP or sink specific irq unhandled\n");
	}

	if (!intel_channel_eq_ok(intel_dp, link_status)) {
		DRM_DEBUG_KMS("%s: channel EQ not ok, retraining\n",
			      drm_get_encoder_name(&intel_dp->base.base));
		intel_dp_start_link_train(intel_dp);
		intel_dp_complete_link_train(intel_dp);
	}
}

static enum drm_connector_status
intel_dp_detect_dpcd(struct intel_dp *intel_dp)
{
	if (intel_dp_get_dpcd(intel_dp))
		return connector_status_connected;
	return connector_status_disconnected;
}

static enum drm_connector_status
ironlake_dp_detect(struct intel_dp *intel_dp)
{
	enum drm_connector_status status;

	/* Can't disconnect eDP, but you can close the lid... */
	if (is_edp(intel_dp)) {
		status = intel_panel_detect(intel_dp->base.base.dev);
		if (status == connector_status_unknown)
			status = connector_status_connected;
		return status;
	}

	return intel_dp_detect_dpcd(intel_dp);
}

static enum drm_connector_status
g4x_dp_detect(struct intel_dp *intel_dp)
{
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	uint32_t temp, bit;

	switch (intel_dp->output_reg) {
	case DP_B:
		bit = DPB_HOTPLUG_INT_STATUS;
		break;
	case DP_C:
		bit = DPC_HOTPLUG_INT_STATUS;
		break;
	case DP_D:
		bit = DPD_HOTPLUG_INT_STATUS;
		break;
	default:
		return connector_status_unknown;
	}

	temp = I915_READ(PORT_HOTPLUG_STAT);

	if ((temp & bit) == 0)
		return connector_status_disconnected;

	return intel_dp_detect_dpcd(intel_dp);
}

static struct edid *
intel_dp_get_edid(struct drm_connector *connector, struct i2c_adapter *adapter)
{
	struct intel_dp *intel_dp = intel_attached_dp(connector);
	struct edid	*edid;

	ironlake_edp_panel_vdd_on(intel_dp);
	edid = drm_get_edid(connector, adapter);
	ironlake_edp_panel_vdd_off(intel_dp, false);
	return edid;
}

static int
intel_dp_get_edid_modes(struct drm_connector *connector, struct i2c_adapter *adapter)
{
	struct intel_dp *intel_dp = intel_attached_dp(connector);
	int	ret;

	ironlake_edp_panel_vdd_on(intel_dp);
	ret = intel_ddc_get_modes(connector, adapter);
	ironlake_edp_panel_vdd_off(intel_dp, false);
	return ret;
}


/**
 * Uses CRT_HOTPLUG_EN and CRT_HOTPLUG_STAT to detect DP connection.
 *
 * \return true if DP port is connected.
 * \return false if DP port is disconnected.
 */
static enum drm_connector_status
intel_dp_detect(struct drm_connector *connector, bool force)
{
	struct intel_dp *intel_dp = intel_attached_dp(connector);
	struct drm_device *dev = intel_dp->base.base.dev;
	enum drm_connector_status status;
	struct edid *edid = NULL;

	intel_dp->has_audio = false;

	if (HAS_PCH_SPLIT(dev))
		status = ironlake_dp_detect(intel_dp);
	else
		status = g4x_dp_detect(intel_dp);

	DRM_DEBUG_KMS("DPCD: %02hx%02hx%02hx%02hx%02hx%02hx%02hx%02hx\n",
		      intel_dp->dpcd[0], intel_dp->dpcd[1], intel_dp->dpcd[2],
		      intel_dp->dpcd[3], intel_dp->dpcd[4], intel_dp->dpcd[5],
		      intel_dp->dpcd[6], intel_dp->dpcd[7]);

	if (status != connector_status_connected)
		return status;

	if (intel_dp->force_audio != HDMI_AUDIO_AUTO) {
		intel_dp->has_audio = (intel_dp->force_audio == HDMI_AUDIO_ON);
	} else {
		edid = intel_dp_get_edid(connector, &intel_dp->adapter);
		if (edid) {
			intel_dp->has_audio = drm_detect_monitor_audio(edid);
			connector->display_info.raw_edid = NULL;
			kfree(edid);
		}
	}

	return connector_status_connected;
}

static int intel_dp_get_modes(struct drm_connector *connector)
{
	struct intel_dp *intel_dp = intel_attached_dp(connector);
	struct drm_device *dev = intel_dp->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	int ret;

	/* We should parse the EDID data and find out if it has an audio sink
	 */

	ret = intel_dp_get_edid_modes(connector, &intel_dp->adapter);
	if (ret) {
		if (is_edp(intel_dp) && !intel_dp->panel_fixed_mode) {
			struct drm_display_mode *newmode;
			list_for_each_entry(newmode, &connector->probed_modes,
					    head) {
				if ((newmode->type & DRM_MODE_TYPE_PREFERRED)) {
					intel_dp->panel_fixed_mode =
						drm_mode_duplicate(dev, newmode);
					break;
				}
			}
		}
		return ret;
	}

	/* if eDP has no EDID, try to use fixed panel mode from VBT */
	if (is_edp(intel_dp)) {
		/* initialize panel mode from VBT if available for eDP */
		if (intel_dp->panel_fixed_mode == NULL && dev_priv->lfp_lvds_vbt_mode != NULL) {
			intel_dp->panel_fixed_mode =
				drm_mode_duplicate(dev, dev_priv->lfp_lvds_vbt_mode);
			if (intel_dp->panel_fixed_mode) {
				intel_dp->panel_fixed_mode->type |=
					DRM_MODE_TYPE_PREFERRED;
			}
		}
		if (intel_dp->panel_fixed_mode) {
			struct drm_display_mode *mode;
			mode = drm_mode_duplicate(dev, intel_dp->panel_fixed_mode);
			drm_mode_probed_add(connector, mode);
			return 1;
		}
	}
	return 0;
}

static bool
intel_dp_detect_audio(struct drm_connector *connector)
{
	struct intel_dp *intel_dp = intel_attached_dp(connector);
	struct edid *edid;
	bool has_audio = false;

	edid = intel_dp_get_edid(connector, &intel_dp->adapter);
	if (edid) {
		has_audio = drm_detect_monitor_audio(edid);

		connector->display_info.raw_edid = NULL;
		kfree(edid);
	}

	return has_audio;
}

static int
intel_dp_set_property(struct drm_connector *connector,
		      struct drm_property *property,
		      uint64_t val)
{
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	struct intel_dp *intel_dp = intel_attached_dp(connector);
	int ret;

	ret = drm_connector_property_set_value(connector, property, val);
	if (ret)
		return ret;

	if (property == dev_priv->force_audio_property) {
		int i = val;
		bool has_audio;

		if (i == intel_dp->force_audio)
			return 0;

		intel_dp->force_audio = i;

		if (i == HDMI_AUDIO_AUTO)
			has_audio = intel_dp_detect_audio(connector);
		else
			has_audio = (i == HDMI_AUDIO_ON);

		if (has_audio == intel_dp->has_audio)
			return 0;

		intel_dp->has_audio = has_audio;
		goto done;
	}

	if (property == dev_priv->broadcast_rgb_property) {
		if (val == !!intel_dp->color_range)
			return 0;

		intel_dp->color_range = val ? DP_COLOR_RANGE_16_235 : 0;
		goto done;
	}

	return -EINVAL;

done:
	if (intel_dp->base.base.crtc) {
		struct drm_crtc *crtc = intel_dp->base.base.crtc;
		drm_crtc_helper_set_mode(crtc, &crtc->mode,
					 crtc->x, crtc->y,
					 crtc->fb);
	}

	return 0;
}

static void
intel_dp_destroy(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;

	if (intel_dpd_is_edp(dev))
		intel_panel_destroy_backlight(dev);

	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

static void intel_dp_encoder_destroy(struct drm_encoder *encoder)
{
	struct intel_dp *intel_dp = enc_to_intel_dp(encoder);

	i2c_del_adapter(&intel_dp->adapter);
	drm_encoder_cleanup(encoder);
	if (is_edp(intel_dp)) {
		cancel_delayed_work_sync(&intel_dp->panel_vdd_work);
		ironlake_panel_vdd_off_sync(intel_dp);
	}
	kfree(intel_dp);
}

static const struct drm_encoder_helper_funcs intel_dp_helper_funcs = {
	.dpms = intel_dp_dpms,
	.mode_fixup = intel_dp_mode_fixup,
	.prepare = intel_dp_prepare,
	.mode_set = intel_dp_mode_set,
	.commit = intel_dp_commit,
};

static const struct drm_connector_funcs intel_dp_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.detect = intel_dp_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = intel_dp_set_property,
	.destroy = intel_dp_destroy,
};

static const struct drm_connector_helper_funcs intel_dp_connector_helper_funcs = {
	.get_modes = intel_dp_get_modes,
	.mode_valid = intel_dp_mode_valid,
	.best_encoder = intel_best_encoder,
};

static const struct drm_encoder_funcs intel_dp_enc_funcs = {
	.destroy = intel_dp_encoder_destroy,
};

static void
intel_dp_hot_plug(struct intel_encoder *intel_encoder)
{
	struct intel_dp *intel_dp = container_of(intel_encoder, struct intel_dp, base);

	intel_dp_check_link_status(intel_dp);
}

/* Return which DP Port should be selected for Transcoder DP control */
int
intel_trans_dp_port_sel(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct drm_encoder *encoder;

	list_for_each_entry(encoder, &mode_config->encoder_list, head) {
		struct intel_dp *intel_dp;

		if (encoder->crtc != crtc)
			continue;

		intel_dp = enc_to_intel_dp(encoder);
		if (intel_dp->base.type == INTEL_OUTPUT_DISPLAYPORT ||
		    intel_dp->base.type == INTEL_OUTPUT_EDP)
			return intel_dp->output_reg;
	}

	return -1;
}

/* check the VBT to see whether the eDP is on DP-D port */
bool intel_dpd_is_edp(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct child_device_config *p_child;
	int i;

	if (!dev_priv->child_dev_num)
		return false;

	for (i = 0; i < dev_priv->child_dev_num; i++) {
		p_child = dev_priv->child_dev + i;

		if (p_child->dvo_port == PORT_IDPD &&
		    p_child->device_type == DEVICE_TYPE_eDP)
			return true;
	}
	return false;
}

static void
intel_dp_add_properties(struct intel_dp *intel_dp, struct drm_connector *connector)
{
	intel_attach_force_audio_property(connector);
	intel_attach_broadcast_rgb_property(connector);
}

void
intel_dp_init(struct drm_device *dev, int output_reg)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_connector *connector;
	struct intel_dp *intel_dp;
	struct intel_encoder *intel_encoder;
	struct intel_connector *intel_connector;
	const char *name = NULL;
	int type;

	intel_dp = kzalloc(sizeof(struct intel_dp), GFP_KERNEL);
	if (!intel_dp)
		return;

	intel_dp->output_reg = output_reg;
	intel_dp->dpms_mode = -1;

	intel_connector = kzalloc(sizeof(struct intel_connector), GFP_KERNEL);
	if (!intel_connector) {
		kfree(intel_dp);
		return;
	}
	intel_encoder = &intel_dp->base;

	if (HAS_PCH_SPLIT(dev) && output_reg == PCH_DP_D)
		if (intel_dpd_is_edp(dev))
			intel_dp->is_pch_edp = true;

	if (output_reg == DP_A || is_pch_edp(intel_dp)) {
		type = DRM_MODE_CONNECTOR_eDP;
		intel_encoder->type = INTEL_OUTPUT_EDP;
	} else {
		type = DRM_MODE_CONNECTOR_DisplayPort;
		intel_encoder->type = INTEL_OUTPUT_DISPLAYPORT;
	}

	connector = &intel_connector->base;
	drm_connector_init(dev, connector, &intel_dp_connector_funcs, type);
	drm_connector_helper_add(connector, &intel_dp_connector_helper_funcs);

	connector->polled = DRM_CONNECTOR_POLL_HPD;

	if (output_reg == DP_B || output_reg == PCH_DP_B)
		intel_encoder->clone_mask = (1 << INTEL_DP_B_CLONE_BIT);
	else if (output_reg == DP_C || output_reg == PCH_DP_C)
		intel_encoder->clone_mask = (1 << INTEL_DP_C_CLONE_BIT);
	else if (output_reg == DP_D || output_reg == PCH_DP_D)
		intel_encoder->clone_mask = (1 << INTEL_DP_D_CLONE_BIT);

	if (is_edp(intel_dp)) {
		intel_encoder->clone_mask = (1 << INTEL_EDP_CLONE_BIT);
		INIT_DELAYED_WORK(&intel_dp->panel_vdd_work,
				  ironlake_panel_vdd_work);
	}

	intel_encoder->crtc_mask = (1 << 0) | (1 << 1) | (1 << 2);
	connector->interlace_allowed = true;
	connector->doublescan_allowed = 0;

	drm_encoder_init(dev, &intel_encoder->base, &intel_dp_enc_funcs,
			 DRM_MODE_ENCODER_TMDS);
	drm_encoder_helper_add(&intel_encoder->base, &intel_dp_helper_funcs);

	intel_connector_attach_encoder(intel_connector, intel_encoder);
	drm_sysfs_connector_add(connector);

	/* Set up the DDC bus. */
	switch (output_reg) {
		case DP_A:
			name = "DPDDC-A";
			break;
		case DP_B:
		case PCH_DP_B:
			dev_priv->hotplug_supported_mask |=
				HDMIB_HOTPLUG_INT_STATUS;
			name = "DPDDC-B";
			break;
		case DP_C:
		case PCH_DP_C:
			dev_priv->hotplug_supported_mask |=
				HDMIC_HOTPLUG_INT_STATUS;
			name = "DPDDC-C";
			break;
		case DP_D:
		case PCH_DP_D:
			dev_priv->hotplug_supported_mask |=
				HDMID_HOTPLUG_INT_STATUS;
			name = "DPDDC-D";
			break;
	}

	/* Cache some DPCD data in the eDP case */
	if (is_edp(intel_dp)) {
		bool ret;
		struct edp_power_seq	cur, vbt;
		u32 pp_on, pp_off, pp_div;

		pp_on = I915_READ(PCH_PP_ON_DELAYS);
		pp_off = I915_READ(PCH_PP_OFF_DELAYS);
		pp_div = I915_READ(PCH_PP_DIVISOR);

		/* Pull timing values out of registers */
		cur.t1_t3 = (pp_on & PANEL_POWER_UP_DELAY_MASK) >>
			PANEL_POWER_UP_DELAY_SHIFT;

		cur.t8 = (pp_on & PANEL_LIGHT_ON_DELAY_MASK) >>
			PANEL_LIGHT_ON_DELAY_SHIFT;

		cur.t9 = (pp_off & PANEL_LIGHT_OFF_DELAY_MASK) >>
			PANEL_LIGHT_OFF_DELAY_SHIFT;

		cur.t10 = (pp_off & PANEL_POWER_DOWN_DELAY_MASK) >>
			PANEL_POWER_DOWN_DELAY_SHIFT;

		cur.t11_t12 = ((pp_div & PANEL_POWER_CYCLE_DELAY_MASK) >>
			       PANEL_POWER_CYCLE_DELAY_SHIFT) * 1000;

		DRM_DEBUG_KMS("cur t1_t3 %d t8 %d t9 %d t10 %d t11_t12 %d\n",
			      cur.t1_t3, cur.t8, cur.t9, cur.t10, cur.t11_t12);

		vbt = dev_priv->edp.pps;

		DRM_DEBUG_KMS("vbt t1_t3 %d t8 %d t9 %d t10 %d t11_t12 %d\n",
			      vbt.t1_t3, vbt.t8, vbt.t9, vbt.t10, vbt.t11_t12);

#define get_delay(field)	((max(cur.field, vbt.field) + 9) / 10)

		intel_dp->panel_power_up_delay = get_delay(t1_t3);
		intel_dp->backlight_on_delay = get_delay(t8);
		intel_dp->backlight_off_delay = get_delay(t9);
		intel_dp->panel_power_down_delay = get_delay(t10);
		intel_dp->panel_power_cycle_delay = get_delay(t11_t12);

		DRM_DEBUG_KMS("panel power up delay %d, power down delay %d, power cycle delay %d\n",
			      intel_dp->panel_power_up_delay, intel_dp->panel_power_down_delay,
			      intel_dp->panel_power_cycle_delay);

		DRM_DEBUG_KMS("backlight on delay %d, off delay %d\n",
			      intel_dp->backlight_on_delay, intel_dp->backlight_off_delay);

		ironlake_edp_panel_vdd_on(intel_dp);
		ret = intel_dp_get_dpcd(intel_dp);
		ironlake_edp_panel_vdd_off(intel_dp, false);

		if (ret) {
			if (intel_dp->dpcd[DP_DPCD_REV] >= 0x11)
				dev_priv->no_aux_handshake =
					intel_dp->dpcd[DP_MAX_DOWNSPREAD] &
					DP_NO_AUX_HANDSHAKE_LINK_TRAINING;
		} else {
			/* if this fails, presume the device is a ghost */
			DRM_INFO("failed to retrieve link info, disabling eDP\n");
			intel_dp_encoder_destroy(&intel_dp->base.base);
			intel_dp_destroy(&intel_connector->base);
			return;
		}
	}

	intel_dp_i2c_init(intel_dp, intel_connector, name);

	intel_encoder->hot_plug = intel_dp_hot_plug;

	if (is_edp(intel_dp)) {
		dev_priv->int_edp_connector = connector;
		intel_panel_setup_backlight(dev);
	}

	intel_dp_add_properties(intel_dp, connector);

	/* For G4X desktop chip, PEG_BAND_GAP_DATA 3:0 must first be written
	 * 0xd.  Failure to do so will result in spurious interrupts being
	 * generated on the port when a cable is not attached.
	 */
	if (IS_G4X(dev) && !IS_GM45(dev)) {
		u32 temp = I915_READ(PEG_BAND_GAP_DATA);
		I915_WRITE(PEG_BAND_GAP_DATA, (temp & ~0xf) | 0xd);
	}
}
