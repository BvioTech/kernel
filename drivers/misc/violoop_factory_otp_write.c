// SPDX-License-Identifier: GPL-2.0-only
/* Factory-owned module: fixed bit0 (release) and bit1 (one-time test) only. */
#include <linux/bitops.h>
#include <linux/capability.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/sysfs.h>
#define VIOLOOP_OTP_WRITE_MODULE
#include "violoop_factory_otp_transport.h"

static DEFINE_MUTEX(write_lock);
static struct kobject *write_kobj;
static bool enabled;

static int flags_read(u32 *value)
{
	__le32 word;
	int ret = violoop_otp_transfer(&word, false);

	if (!ret)
		*value = le32_to_cpu(word);
	return ret;
}

/* No offset/mask/data argument is accepted from userspace. Preserve all other bits. */
static int commit_flag(unsigned int bit)
{
	u32 before, after, desired;
	__le32 word;
	int ret;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	if (bit > 1)
		return -EINVAL;
	mutex_lock(&write_lock);
	ret = flags_read(&before);
	if (ret)
		goto out;
	if (before & BIT(0)) {
		enabled = false;
		ret = bit == 0 ? 0 : -EPERM;
		goto out;
	}
	if (!enabled) {
		ret = -EACCES;
		goto out;
	}
	if (before & BIT(bit)) {
		ret = 0;
		goto out;
	}
	desired = before | BIT(bit);
	word = cpu_to_le32(desired);
	ret = violoop_otp_transfer(&word, true);
	/* Always read back, including ambiguous transport failures; never retry a burn. */
	if (!flags_read(&after)) {
		if (after & BIT(0))
			enabled = false;
		if (after == desired)
			ret = 0;
		else if (!ret)
			ret = -EIO;
	} else if (!ret) {
		ret = -EIO;
	}
out:
	mutex_unlock(&write_lock);
	return ret;
}

static ssize_t commit_store(struct kobject *k, struct kobj_attribute *a,
			   const char *buf, size_t count)
{
	int ret;
	if (!sysfs_streq(buf, "COMMIT_FACTORY_DONE_V1"))
		return -EINVAL;
	ret = commit_flag(0);
	return ret ? ret : count;
}
static ssize_t test_commit_store(struct kobject *k, struct kobj_attribute *a,
				const char *buf, size_t count)
{
	int ret;
	if (!sysfs_streq(buf, "COMMIT_FACTORY_TEST_BIT1_V1"))
		return -EINVAL;
	ret = commit_flag(1);
	return ret ? ret : count;
}
static ssize_t enabled_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(enabled));
}
static ssize_t enabled_store(struct kobject *k, struct kobj_attribute *a,
			    const char *buf, size_t count)
{
	u32 value;
	int ret = 0;
	bool want;

	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	if (sysfs_streq(buf, "0"))
		want = false;
	else if (sysfs_streq(buf, "1"))
		want = true;
	else
		return -EINVAL;
	mutex_lock(&write_lock);
	if (want) {
		ret = flags_read(&value);
		if (!ret && (value & BIT(0)))
			ret = -EPERM;
	}
	if (!ret)
		enabled = want;
	mutex_unlock(&write_lock);
	return ret ? ret : count;
}
static ssize_t api_show(struct kobject *k, struct kobj_attribute *a, char *buf)
{
	u32 value;
	int ret;
	if (!capable(CAP_SYS_RAWIO))
		return -EPERM;
	mutex_lock(&write_lock);
	ret = flags_read(&value);
	if (!ret && (value & BIT(0)))
		enabled = false;
	mutex_unlock(&write_lock);
	if (ret)
		return ret;
	return sysfs_emit(buf, "version=1 read_api=ok write_entry=present enabled=%u bit0=%u bit1=%u burn_test=not_performed\n",
		READ_ONCE(enabled), !!(value & BIT(0)), !!(value & BIT(1)));
}
static struct kobj_attribute commit_attr = __ATTR(commit, 0200, NULL, commit_store);
static struct kobj_attribute test_attr = __ATTR(test_commit, 0200, NULL, test_commit_store);
static struct kobj_attribute enabled_attr = __ATTR(enabled, 0600, enabled_show, enabled_store);
static struct kobj_attribute api_attr = __ATTR(api, 0400, api_show, NULL);
static struct attribute *attrs[] = { &commit_attr.attr, &test_attr.attr,
	&enabled_attr.attr, &api_attr.attr, NULL };
static const struct attribute_group group = { .attrs = attrs };

static int __init writer_init(void)
{
	u32 flags;
	int ret;
	if (!of_machine_is_compatible("rockchip,rk3576"))
		return -ENODEV;
	ret = flags_read(&flags);
	if (ret)
		return ret;
	if (flags & BIT(0))
		return -EPERM;
	write_kobj = kobject_create_and_add("violoop_factory_otp_write", kernel_kobj);
	if (!write_kobj)
		return -ENOMEM;
	enabled = true;
	ret = sysfs_create_group(write_kobj, &group);
	if (ret)
		kobject_put(write_kobj);
	return ret;
}
static void __exit writer_exit(void)
{
	sysfs_remove_group(write_kobj, &group);
	kobject_put(write_kobj);
}
module_init(writer_init);
module_exit(writer_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("VLA11 factory-only fixed OTP completion/test bits");
