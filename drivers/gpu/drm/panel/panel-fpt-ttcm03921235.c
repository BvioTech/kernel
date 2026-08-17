// SPDX-License-Identifier: GPL-2.0
/*
 * FPT TTCM03921235 3.92-inch AMOLED MIPI-DSI panel
 *
 * The module uses a factory-programmed Chip Wealth CH13726A controller.
 * Keep all panel-specific power, reset, DCS brightness and display-control
 * handling here so generic Rockchip display drivers remain unchanged.
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

#define TTCM03921235_WIDTH_MM		65
#define TTCM03921235_HEIGHT_MM		76
#define TTCM03921235_MAX_BRIGHTNESS	255

struct ttcm03921235 {
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
	bool skip_next_brightness;
};

static const struct drm_display_mode ttcm03921235_mode = {
	.clock = 84297,
	.hdisplay = 1080,
	.hsync_start = 1080 + 12,
	.hsync_end = 1080 + 12 + 4,
	.htotal = 1080 + 12 + 4 + 12,
	.vdisplay = 1240,
	.vsync_start = 1240 + 12,
	.vsync_end = 1240 + 12 + 4,
	.vtotal = 1240 + 12 + 4 + 12,
	.width_mm = TTCM03921235_WIDTH_MM,
	.height_mm = TTCM03921235_HEIGHT_MM,
	.flags = DRM_MODE_FLAG_NHSYNC | DRM_MODE_FLAG_NVSYNC,
};

static const u32 ttcm03921235_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
};

static inline struct ttcm03921235 *to_ttcm03921235(struct drm_panel *panel)
{
	return container_of(panel, struct ttcm03921235, panel);
}

static int ttcm03921235_set_display(struct ttcm03921235 *ctx, bool on)
{
	unsigned long mode_flags = ctx->dsi->mode_flags;
	int ret;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	if (on)
		ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
	else
		ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	ctx->dsi->mode_flags = mode_flags;

	return ret < 0 ? ret : 0;
}

static int ttcm03921235_set_brightness(struct ttcm03921235 *ctx, u8 value)
{
	unsigned long mode_flags = ctx->dsi->mode_flags;
	int ret;

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_write(ctx->dsi,
				      MIPI_DCS_SET_DISPLAY_BRIGHTNESS,
				      &value, sizeof(value));
	ctx->dsi->mode_flags = mode_flags;

	return ret < 0 ? ret : 0;
}

static int ttcm03921235_prepare(struct drm_panel *panel)
{
	struct ttcm03921235 *ctx = to_ttcm03921235(panel);
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

	/* RSTB high >= 10 ms, low >= 10 ms, then high >= 10 ms. */
	msleep(15);
	gpiod_direction_output(ctx->reset, 1);
	msleep(10);
	gpiod_direction_output(ctx->reset, 0);
	msleep(10);

	mode_flags = ctx->dsi->mode_flags;
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_exit_sleep_mode(ctx->dsi);
	if (ret < 0) {
		dev_err(panel->dev, "failed to exit sleep mode: %d\n", ret);
		goto err_power_off;
	}
	msleep(120);

	ret = mipi_dsi_dcs_set_display_on(ctx->dsi);
	ctx->dsi->mode_flags = mode_flags;
	if (ret < 0) {
		dev_err(panel->dev, "failed to enable display: %d\n", ret);
		goto err_power_off;
	}
	msleep(40);

	ctx->prepared = true;
	ctx->display_on = true;
	ret = 0;
	goto out_unlock;

err_power_off:
	ctx->dsi->mode_flags = mode_flags;
	gpiod_direction_output(ctx->reset, 1);
	regulator_disable(ctx->supply);
out_unlock:
	mutex_unlock(&ctx->lock);
	return ret;
}

static int ttcm03921235_unprepare(struct drm_panel *panel)
{
	struct ttcm03921235 *ctx = to_ttcm03921235(panel);
	unsigned long mode_flags;
	int first_error = 0;
	int ret;

	mutex_lock(&ctx->lock);
	if (!ctx->prepared)
		goto out_unlock;

	mode_flags = ctx->dsi->mode_flags;
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;
	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0) {
		dev_err(panel->dev, "failed to disable display: %d\n", ret);
		first_error = ret;
	}

	ret = mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	ctx->dsi->mode_flags = mode_flags;
	if (ret < 0) {
		dev_err(panel->dev, "failed to enter sleep mode: %d\n", ret);
		if (!first_error)
			first_error = ret;
	}
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

static int ttcm03921235_enable(struct drm_panel *panel)
{
	struct ttcm03921235 *ctx = to_ttcm03921235(panel);

	mutex_lock(&ctx->lock);
	if (ctx->enabled) {
		mutex_unlock(&ctx->lock);
		return 0;
	}
	ctx->enabled = true;
	mutex_unlock(&ctx->lock);

	rockchip_panel_notifier_call_chain(&ctx->panel_notifier,
					   PANEL_ENABLED, NULL);
	return 0;
}

static int ttcm03921235_disable(struct drm_panel *panel)
{
	struct ttcm03921235 *ctx = to_ttcm03921235(panel);

	rockchip_panel_notifier_call_chain(&ctx->panel_notifier,
					   PANEL_PRE_DISABLE, NULL);

	mutex_lock(&ctx->lock);
	ctx->enabled = false;
	mutex_unlock(&ctx->lock);
	return 0;
}

static int ttcm03921235_get_modes(struct drm_panel *panel,
				  struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &ttcm03921235_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_set_name(mode);
	mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.bpc = 8;
	connector->display_info.width_mm = TTCM03921235_WIDTH_MM;
	connector->display_info.height_mm = TTCM03921235_HEIGHT_MM;
	drm_display_info_set_bus_formats(&connector->display_info,
					 ttcm03921235_bus_formats,
					 ARRAY_SIZE(ttcm03921235_bus_formats));

	return 1;
}

static enum drm_panel_orientation
ttcm03921235_get_orientation(struct drm_panel *panel)
{
	return to_ttcm03921235(panel)->orientation;
}

static const struct drm_panel_funcs ttcm03921235_panel_funcs = {
	.prepare = ttcm03921235_prepare,
	.unprepare = ttcm03921235_unprepare,
	.enable = ttcm03921235_enable,
	.disable = ttcm03921235_disable,
	.get_modes = ttcm03921235_get_modes,
	.get_orientation = ttcm03921235_get_orientation,
};

static int ttcm03921235_bl_update_status(struct backlight_device *bl)
{
	struct ttcm03921235 *ctx = bl_get_data(bl);
	u8 brightness;
	int ret = 0;

	mutex_lock(&ctx->lock);
	if (!ctx->prepared || backlight_is_blank(bl))
		goto out_unlock;

	if (ctx->skip_next_brightness) {
		ctx->skip_next_brightness = false;
		goto out_unlock;
	}

	brightness = backlight_get_brightness(bl);
	ret = ttcm03921235_set_brightness(ctx, brightness);
	if (ret)
		dev_err(&ctx->dsi->dev, "failed to set brightness: %d\n", ret);

out_unlock:
	mutex_unlock(&ctx->lock);
	return ret;
}

static int ttcm03921235_bl_get_brightness(struct backlight_device *bl)
{
	struct ttcm03921235 *ctx = bl_get_data(bl);
	int brightness;

	mutex_lock(&ctx->lock);
	brightness = ctx->prepared ? bl->props.brightness : 0;
	mutex_unlock(&ctx->lock);

	return brightness;
}

static const struct backlight_ops ttcm03921235_bl_ops = {
	.update_status = ttcm03921235_bl_update_status,
	.get_brightness = ttcm03921235_bl_get_brightness,
};

static ssize_t display_power_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct ttcm03921235 *ctx = dev_get_drvdata(dev);
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
	struct ttcm03921235 *ctx = dev_get_drvdata(dev);
	bool display_on;
	int ret;

	ret = kstrtobool(buf, &display_on);
	if (ret)
		return ret;

	mutex_lock(&ctx->lock);
	if (!ctx->prepared) {
		ret = -EHOSTDOWN;
		goto out_unlock;
	}

	if (display_on == ctx->display_on) {
		ret = 0;
		goto out_unlock;
	}

	ret = ttcm03921235_set_display(ctx, display_on);
	if (!ret) {
		msleep(20);
		ctx->display_on = display_on;
	}

out_unlock:
	mutex_unlock(&ctx->lock);
	if (ret) {
		dev_err(dev, "failed to set display power to %u: %d\n",
			display_on, ret);
		return ret;
	}

	return count;
}
static DEVICE_ATTR_RW(display_power);

static struct attribute *ttcm03921235_attrs[] = {
	&dev_attr_display_power.attr,
	NULL,
};

static const struct attribute_group ttcm03921235_attr_group = {
	.attrs = ttcm03921235_attrs,
};

static int ttcm03921235_probe(struct mipi_dsi_device *dsi)
{
	struct backlight_properties props = { 0 };
	struct device *dev = &dsi->dev;
	struct ttcm03921235 *ctx;
	bool bootloader_initialized;
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

	ctx->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset),
				     "failed to get panel reset GPIO\n");

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET;

	drm_panel_init(&ctx->panel, dev, &ttcm03921235_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);

	ret = of_drm_get_panel_orientation(dev->of_node, &ctx->orientation);
	if (ret)
		ctx->orientation = DRM_MODE_PANEL_ORIENTATION_UNKNOWN;

	props.type = BACKLIGHT_RAW;
	props.brightness = TTCM03921235_MAX_BRIGHTNESS;
	props.max_brightness = TTCM03921235_MAX_BRIGHTNESS;
	ctx->backlight = devm_backlight_device_register(dev, dev_name(dev), dev,
						ctx, &ttcm03921235_bl_ops,
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

	ret = devm_device_add_group(dev, &ttcm03921235_attr_group);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add display controls\n");

	bootloader_initialized =
		of_property_read_bool(dev->of_node, "fpt,bootloader-initialized");
	if (bootloader_initialized) {
		ret = regulator_enable(ctx->supply);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to preserve bootloader panel power\n");
		ctx->prepared = true;
		ctx->enabled = true;
		ctx->display_on = true;
		ctx->skip_next_brightness = true;
	}

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&ctx->panel);
		if (bootloader_initialized)
			regulator_disable(ctx->supply);
		return dev_err_probe(dev, ret, "failed to attach DSI panel\n");
	}

	dev_info(dev, "FPT TTCM03921235 AMOLED panel initialized%s\n",
		 bootloader_initialized ? " with bootloader handoff" : "");
	return 0;
}

static void ttcm03921235_remove(struct mipi_dsi_device *dsi)
{
	struct ttcm03921235 *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static void ttcm03921235_shutdown(struct mipi_dsi_device *dsi)
{
	struct ttcm03921235 *ctx = mipi_dsi_get_drvdata(dsi);

	ttcm03921235_disable(&ctx->panel);
	ttcm03921235_unprepare(&ctx->panel);
}

static const struct of_device_id ttcm03921235_of_match[] = {
	{ .compatible = "fpt,ttcm03921235" },
	{ }
};
MODULE_DEVICE_TABLE(of, ttcm03921235_of_match);

static struct mipi_dsi_driver ttcm03921235_driver = {
	.probe = ttcm03921235_probe,
	.remove = ttcm03921235_remove,
	.shutdown = ttcm03921235_shutdown,
	.driver = {
		.name = "panel-fpt-ttcm03921235",
		.of_match_table = ttcm03921235_of_match,
	},
};
module_mipi_dsi_driver(ttcm03921235_driver);

MODULE_AUTHOR("Violoop");
MODULE_DESCRIPTION("FPT TTCM03921235 AMOLED panel driver");
MODULE_LICENSE("GPL");
