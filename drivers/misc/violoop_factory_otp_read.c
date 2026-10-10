// SPDX-License-Identifier: GPL-2.0-only
/* Built-in, read-only factory lifecycle state. No programming operation. */
#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/sysfs.h>
#include <linux/rockchip/nvmem.h>

static DEFINE_MUTEX(read_lock);
static struct kobject *otp_kobj;

static ssize_t read_flag(char *buf, unsigned int bit)
{
	__le32 word = 0;
	int ret;

	mutex_lock(&read_lock);
	ret = rockchip_read_oem_non_protected_otp(0, &word, sizeof(word));
	mutex_unlock(&read_lock);
	if (ret)
		return ret;
	return sysfs_emit(buf, "%u\n", !!(le32_to_cpu(word) & BIT(bit)));
}

static ssize_t status_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	return read_flag(buf, 0);
}
static ssize_t test_status_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	return read_flag(buf, 1);
}
static struct kobj_attribute status_attr = __ATTR_RO(status);
static struct kobj_attribute test_status_attr = __ATTR_RO(test_status);
static struct attribute *otp_attrs[] = { &status_attr.attr, &test_status_attr.attr, NULL };
static const struct attribute_group otp_group = { .attrs = otp_attrs };

static int __init violoop_otp_read_init(void)
{
	int ret;

	if (!of_machine_is_compatible("violoop,rk3576-vla11"))
		return -ENODEV;
	otp_kobj = kobject_create_and_add("violoop_factory_otp", kernel_kobj);
	if (!otp_kobj)
		return -ENOMEM;
	ret = sysfs_create_group(otp_kobj, &otp_group);
	if (ret)
		kobject_put(otp_kobj);
	return ret;
}
late_initcall(violoop_otp_read_init);
MODULE_LICENSE("GPL");
