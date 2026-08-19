// SPDX-License-Identifier: GPL-2.0
/*
 * Jujing JN3929595A 3.92-inch AMOLED MIPI-DSI panel
 *
 * The module uses a factory-programmed CH13726A controller.  Keep the
 * Jujing command sequence and runtime controls in this dedicated driver so
 * the existing FPT panel and Rockchip's generic panel code stay untouched.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/rockchip-panel-notifier.h>

#include <video/mipi_display.h>

#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#define JN3929595A_WIDTH_MM		65
#define JN3929595A_HEIGHT_MM		76
#define JN3929595A_MAX_BRIGHTNESS	255

struct jn3929595a {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator *supply;
	struct gpio_desc *reset;
	struct backlight_device *backlight;
	struct rockchip_panel_notifier panel_notifier;
	struct mutex lock;
	enum drm_panel_orientation orientation;
	bool prepared;
	bool enabled;
	bool display_on;
};

static const struct drm_display_mode jn3929595a_mode = {
	/* Keep this byte-for-byte aligned with the U-Boot handoff mode. */
	.clock = 84296,
	.hdisplay = 1080,
	.hsync_start = 1080 + 12,
	.hsync_end = 1080 + 12 + 4,
	.htotal = 1080 + 12 + 4 + 12,
	.vdisplay = 1240,
	.vsync_start = 1240 + 12,
	.vsync_end = 1240 + 12 + 4,
	.vtotal = 1240 + 12 + 4 + 12,
	.width_mm = JN3929595A_WIDTH_MM,
	.height_mm = JN3929595A_HEIGHT_MM,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static const u32 jn3929595a_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
};

/*
 * The panel controller is programmed in OTP.  Do not send the full vendor
 * register table here: the Jujing supplier only requires this short DCS code
 * after reset.  Sleep-out/display-on and their delays follow in
 * jn3929595a_init().
 */
static const u8 jn3929595a_init_sequence[] = {
	5, 0x2a, 0x00, 0x00, 0x04, 0x37,
	5, 0x2b, 0x00, 0x00, 0x04, 0xd7,
	2, 0x51, 0xff,
	2, 0x35, 0x00,
};

static inline struct jn3929595a *to_jn3929595a(struct drm_panel *panel)
{
	return container_of(panel, struct jn3929595a, panel);
}

static int jn3929595a_dcs_write(struct jn3929595a *ctx, const void *data,
				 size_t len)
{
	ssize_t ret;

	ret = mipi_dsi_dcs_write_buffer(ctx->dsi, data, len);
	return ret < 0 ? ret : 0;
}

static int jn3929595a_set_display(struct jn3929595a *ctx, bool on)
{
	unsigned long mode_flags = ctx->dsi->mode_flags;
	int ret;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = on ? mipi_dsi_dcs_set_display_on(ctx->dsi) :
		mipi_dsi_dcs_set_display_off(ctx->dsi);
	ctx->dsi->mode_flags = mode_flags;

	return ret < 0 ? ret : 0;
}

static int jn3929595a_set_brightness(struct jn3929595a *ctx, u8 value)
{
	unsigned long mode_flags = ctx->dsi->mode_flags;
	u8 payload[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, value };
	int ret;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = jn3929595a_dcs_write(ctx, payload, sizeof(payload));
	ctx->dsi->mode_flags = mode_flags;

	return ret;
}

static int jn3929595a_init(struct jn3929595a *ctx)
{
	const u8 *command = jn3929595a_init_sequence;
	const u8 *end = command + ARRAY_SIZE(jn3929595a_init_sequence);
	int ret;

	while (command < end) {
		u8 length = *command++;

		if (!length || command + length > end)
			return -EINVAL;
		ret = jn3929595a_dcs_write(ctx, command, length);
		if (ret) {
			dev_err(&ctx->dsi->dev,
				"init command 0x%02x failed: %d\n",
				command[0], ret);
			return ret;
		}
		command += length;
	}

	ret = mipi_dsi_dcs_exit_sleep_mode(ctx->dsi);
	if (ret < 0)
		return ret;
	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
	if (ret < 0)
		return ret;
	msleep(40);

	return 0;
}

static int jn3929595a_prepare(struct drm_panel *panel)
{
	struct jn3929595a *ctx = to_jn3929595a(panel);
	unsigned long mode_flags;
	int ret;

	mutex_lock(&ctx->lock);
	if (ctx->prepared) {
		ret = 0;
		goto out_unlock;
	}

	ret = regulator_enable(ctx->supply);
	if (ret) {
		dev_err(panel->dev, "failed to enable panel supply: %d\n", ret);
		goto out_unlock;
	}

	/* RSTB low >= 10 ms, then high >= 10 ms. */
	msleep(15);
	gpiod_direction_output(ctx->reset, 1);
	msleep(10);
	gpiod_direction_output(ctx->reset, 0);
	msleep(10);

	mode_flags = ctx->dsi->mode_flags;
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = jn3929595a_init(ctx);
	ctx->dsi->mode_flags = mode_flags;
	if (ret) {
		dev_err(panel->dev, "panel initialization failed: %d\n", ret);
		gpiod_direction_output(ctx->reset, 1);
		regulator_disable(ctx->supply);
		goto out_unlock;
	}

	ctx->prepared = true;
	ctx->display_on = true;

out_unlock:
	mutex_unlock(&ctx->lock);
	return ret;
}

/*
 * Synchronize the dedicated panel driver's software state with an already
 * running display inherited from U-Boot.  Do not reset the panel or transmit
 * DCS commands here: the DSI host is protected immediately after this call
 * and the bootloader's live frame must remain undisturbed.
 */
int panel_jujing_jn3929595a_loader_protect(struct drm_panel *panel)
{
	struct jn3929595a *ctx = to_jn3929595a(panel);
	int ret = 0;

	mutex_lock(&ctx->lock);
	if (!ctx->prepared) {
		ret = regulator_enable(ctx->supply);
		if (ret) {
			dev_err(panel->dev,
				"failed to acquire bootloader panel supply: %d\n",
				ret);
			goto out_unlock;
		}
	}

	ctx->prepared = true;
	ctx->enabled = true;
	ctx->display_on = true;
	dev_info(panel->dev, "preserving bootloader-enabled panel\n");

out_unlock:
	mutex_unlock(&ctx->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(panel_jujing_jn3929595a_loader_protect);

static int jn3929595a_unprepare(struct drm_panel *panel)
{
	struct jn3929595a *ctx = to_jn3929595a(panel);
	unsigned long mode_flags;
	int first_error = 0;
	int ret;

	mutex_lock(&ctx->lock);
	if (!ctx->prepared)
		goto out_unlock;

	mode_flags = ctx->dsi->mode_flags;
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0)
		first_error = ret;
	ret = mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	ctx->dsi->mode_flags = mode_flags;
	if (ret < 0 && !first_error)
		first_error = ret;
	msleep(120);

	gpiod_direction_output(ctx->reset, 1);
	ret = regulator_disable(ctx->supply);
	if (ret && !first_error)
		first_error = ret;
	msleep(5);

	ctx->prepared = false;
	ctx->display_on = false;

out_unlock:
	mutex_unlock(&ctx->lock);
	return first_error;
}

static int jn3929595a_enable(struct drm_panel *panel)
{
	struct jn3929595a *ctx = to_jn3929595a(panel);
	bool notify = false;

	mutex_lock(&ctx->lock);
	if (!ctx->enabled) {
		ctx->enabled = true;
		notify = true;
	}
	mutex_unlock(&ctx->lock);
	if (notify)
		rockchip_panel_notifier_call_chain(&ctx->panel_notifier,
						   PANEL_ENABLED, NULL);
	return 0;
}

static int jn3929595a_disable(struct drm_panel *panel)
{
	struct jn3929595a *ctx = to_jn3929595a(panel);
	bool notify;

	mutex_lock(&ctx->lock);
	notify = ctx->enabled;
	mutex_unlock(&ctx->lock);
	if (!notify)
		return 0;

	rockchip_panel_notifier_call_chain(&ctx->panel_notifier,
					   PANEL_PRE_DISABLE, NULL);
	mutex_lock(&ctx->lock);
	ctx->enabled = false;
	mutex_unlock(&ctx->lock);
	return 0;
}

static int jn3929595a_get_modes(struct drm_panel *panel,
				 struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &jn3929595a_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);
	connector->display_info.bpc = 8;
	connector->display_info.width_mm = JN3929595A_WIDTH_MM;
	connector->display_info.height_mm = JN3929595A_HEIGHT_MM;
	drm_display_info_set_bus_formats(&connector->display_info,
					 jn3929595a_bus_formats,
					 ARRAY_SIZE(jn3929595a_bus_formats));
	return 1;
}

static enum drm_panel_orientation
jn3929595a_get_orientation(struct drm_panel *panel)
{
	return to_jn3929595a(panel)->orientation;
}

static const struct drm_panel_funcs jn3929595a_panel_funcs = {
	.prepare = jn3929595a_prepare,
	.unprepare = jn3929595a_unprepare,
	.enable = jn3929595a_enable,
	.disable = jn3929595a_disable,
	.get_modes = jn3929595a_get_modes,
	.get_orientation = jn3929595a_get_orientation,
};

static int jn3929595a_bl_update_status(struct backlight_device *bl)
{
	struct jn3929595a *ctx = bl_get_data(bl);
	int ret = 0;

	mutex_lock(&ctx->lock);
	if (ctx->prepared && !backlight_is_blank(bl))
		ret = jn3929595a_set_brightness(ctx,
					       backlight_get_brightness(bl));
	mutex_unlock(&ctx->lock);
	return ret;
}

static int jn3929595a_bl_get_brightness(struct backlight_device *bl)
{
	struct jn3929595a *ctx = bl_get_data(bl);
	int brightness;

	mutex_lock(&ctx->lock);
	brightness = ctx->prepared ? bl->props.brightness : 0;
	mutex_unlock(&ctx->lock);
	return brightness;
}

static const struct backlight_ops jn3929595a_bl_ops = {
	.update_status = jn3929595a_bl_update_status,
	.get_brightness = jn3929595a_bl_get_brightness,
};

static ssize_t display_power_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct jn3929595a *ctx = dev_get_drvdata(dev);
	bool display_on;

	mutex_lock(&ctx->lock);
	display_on = ctx->prepared && ctx->display_on;
	mutex_unlock(&ctx->lock);

	return sysfs_emit(buf, "%u\n", display_on);
}

static ssize_t display_power_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct jn3929595a *ctx = dev_get_drvdata(dev);
	bool display_on;
	int ret;

	ret = kstrtobool(buf, &display_on);
	if (ret)
		return ret;

	mutex_lock(&ctx->lock);
	if (!ctx->prepared) {
		ret = -EHOSTDOWN;
	} else if (display_on == ctx->display_on) {
		ret = 0;
	} else {
		ret = jn3929595a_set_display(ctx, display_on);
		if (!ret) {
			msleep(20);
			ctx->display_on = display_on;
		}
	}
	mutex_unlock(&ctx->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(display_power);

static struct attribute *jn3929595a_attrs[] = {
	&dev_attr_display_power.attr,
	NULL,
};

static const struct attribute_group jn3929595a_attr_group = {
	.attrs = jn3929595a_attrs,
};

static int jn3929595a_probe(struct mipi_dsi_device *dsi)
{
	struct backlight_properties props = { 0 };
	struct device *dev = &dsi->dev;
	struct jn3929595a *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dsi = dsi;
	mutex_init(&ctx->lock);
	mipi_dsi_set_drvdata(dsi, ctx);
	ctx->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(ctx->supply))
		return dev_err_probe(dev, PTR_ERR(ctx->supply),
				     "failed to get panel supply\n");
	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset),
				     "failed to get panel reset GPIO\n");

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET;
	drm_panel_init(&ctx->panel, dev, &jn3929595a_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	ret = of_drm_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret)
		ctx->orientation = DRM_MODE_PANEL_ORIENTATION_UNKNOWN;

	props.type = BACKLIGHT_RAW;
	props.brightness = JN3929595A_MAX_BRIGHTNESS;
	props.max_brightness = JN3929595A_MAX_BRIGHTNESS;
	ctx->backlight = devm_backlight_device_register(dev, dev_name(dev), dev,
						ctx, &jn3929595a_bl_ops,
						&props);
	if (IS_ERR(ctx->backlight))
		return dev_err_probe(dev, PTR_ERR(ctx->backlight),
				     "failed to register DCS backlight\n");
	ctx->panel.backlight = ctx->backlight;

	ret = devm_rockchip_panel_notifier_register(dev, &ctx->panel,
						   &ctx->panel_notifier);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register panel notifier\n");
	ret = devm_device_add_group(dev, &jn3929595a_attr_group);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add display controls\n");

	drm_panel_add(&ctx->panel);
	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach DSI panel\n");
	}

	dev_info(dev, "Jujing JN3929595A AMOLED panel initialized\n");
	return 0;
}

static void jn3929595a_remove(struct mipi_dsi_device *dsi)
{
	struct jn3929595a *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static void jn3929595a_shutdown(struct mipi_dsi_device *dsi)
{
	struct jn3929595a *ctx = mipi_dsi_get_drvdata(dsi);

	jn3929595a_disable(&ctx->panel);
	jn3929595a_unprepare(&ctx->panel);
}

static const struct of_device_id jn3929595a_of_match[] = {
	{ .compatible = "jujing,jn3929595a" },
	{ }
};
MODULE_DEVICE_TABLE(of, jn3929595a_of_match);

static struct mipi_dsi_driver jn3929595a_driver = {
	.probe = jn3929595a_probe,
	.remove = jn3929595a_remove,
	.shutdown = jn3929595a_shutdown,
	.driver = {
		.name = "panel-jujing-jn3929595a",
		.of_match_table = jn3929595a_of_match,
	},
};
module_mipi_dsi_driver(jn3929595a_driver);

MODULE_AUTHOR("Violoop");
MODULE_DESCRIPTION("Jujing JN3929595A AMOLED panel driver");
MODULE_LICENSE("GPL");
