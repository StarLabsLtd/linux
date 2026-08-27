// SPDX-License-Identifier: GPL-2.0-only
/*
 * coreboot CFR firmware attributes driver.
 *
 * Parses LB_TAG_CFR_ROOT records from the coreboot table and exposes
 * runtime options through the firmware-attributes class.
 */

#include <linux/array_size.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/compiler_attributes.h>
#include <linux/container_of.h>
#include <linux/crc32.h>
#include <linux/ctype.h>
#include <linux/device.h>
#include <linux/device-id/coreboot.h>
#include <linux/efi.h>
#include <linux/err.h>
#include <linux/firmware_attributes.h>
#include <linux/io.h>
#include <linux/kdev_t.h>
#include <linux/kobject.h>
#include <linux/kstrtox.h>
#include <linux/limits.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include "coreboot-cfr-private.h"
#if IS_ENABLED(CONFIG_COREBOOT_CFR_KUNIT_TEST)
#include "coreboot-cfr-kunit.h"
#endif
#include "coreboot_table.h"

#define DRIVER_NAME "coreboot-cfr"

#define LB_TAG_CFR_ROOT		0x47
#define LB_TAG_CFR_SETTINGS	0x4b
#define CFR_VERSION		0

enum cfr_tags {
	CFR_TAG_OPTION_FORM		= 1,
	CFR_TAG_ENUM_VALUE		= 2,
	CFR_TAG_OPTION_ENUM		= 3,
	CFR_TAG_OPTION_NUMBER		= 4,
	CFR_TAG_OPTION_BOOL		= 5,
	CFR_TAG_VARCHAR_OPT_NAME	= 7,
	CFR_TAG_VARCHAR_UI_NAME		= 8,
	CFR_TAG_DEP_VALUES		= 12,
	/* v13 EFI runtime apply metadata. */
	CFR_TAG_RUNTIME_APPLY		= 13,
	/* Atomic-service option token and permissions. */
	CFR_TAG_OPTION_ACCESS		= 14,
};

enum cfr_option_flags {
	CFR_OPTFLAG_READONLY	= BIT(0),
	CFR_OPTFLAG_INACTIVE	= BIT(1),
	CFR_OPTFLAG_SUPPRESS	= BIT(2),
	CFR_OPTFLAG_VOLATILE	= BIT(3),
	CFR_OPTFLAG_RUNTIME	= BIT(4),
};

enum cfr_runtime_apply_method {
	CFR_RUNTIME_APPLY_NONE		= 0,
	CFR_RUNTIME_APPLY_APM_CNT	= 1,
};

struct lb_cfr {
	u32 tag;
	u32 size;
	u32 version;
	u32 checksum;
} __packed;

struct lb_cfr_varbinary {
	u32 tag;
	u32 size;
	u32 data_length;
} __packed;

struct lb_cfr_enum_value {
	u32 tag;
	u32 size;
	u32 value;
} __packed;

struct lb_cfr_runtime_apply {
	u32 tag;
	u32 size;
	u32 method;
	u32 id;
} __packed;

struct lb_cfr_option_access {
	__le32 tag;
	__le32 size;
	__le32 version;
	__le32 token;
	__le32 permissions;
	__le32 reserved;
} __packed;

#define CFR_OPTION_ACCESS_VERSION	1
#define CFR_OPTION_ACCESS_READ		BIT(0)
#define CFR_OPTION_ACCESS_WRITE		BIT(1)
#define CFR_OPTION_ACCESS_PERMISSIONS_MASK \
	(CFR_OPTION_ACCESS_READ | CFR_OPTION_ACCESS_WRITE)

struct lb_cfr_numeric_option {
	u32 tag;
	u32 size;
	cb_u64 object_id;
	cb_u64 dependency_id;
	u32 flags;
	u32 default_value;
	u32 min;
	u32 max;
	u32 step;
	u32 display_flags;
} __packed;

struct lb_cfr_option_form {
	u32 tag;
	u32 size;
	cb_u64 object_id;
	cb_u64 dependency_id;
	u32 flags;
} __packed;

#define COREBOOT_CFR_OPT_READ_ONLY_FLAGS \
	(CFR_OPTFLAG_READONLY | CFR_OPTFLAG_INACTIVE | CFR_OPTFLAG_VOLATILE)

#define COREBOOT_CFR_EFI_ATTRS \
	(EFI_VARIABLE_NON_VOLATILE | EFI_VARIABLE_BOOTSERVICE_ACCESS | \
	 EFI_VARIABLE_RUNTIME_ACCESS)

#define COREBOOT_CFR_APM_CNT_PORT	0xb2
#define COREBOOT_CFR_APM_STS_PORT	0xb3
#define COREBOOT_CFR_APM_APPLY_CMD	0xe3
#define COREBOOT_CFR_MAX_FORM_DEPTH	16
#define COREBOOT_CFR_MAX_DEPENDENCY_DEPTH	64

enum coreboot_cfr_setting_type {
	COREBOOT_CFR_SETTING_ENUM,
	COREBOOT_CFR_SETTING_NUMBER,
	COREBOOT_CFR_SETTING_BOOL,
};

struct coreboot_cfr_enum {
	char *label;
	u32 value;
};

struct coreboot_cfr_dependency {
	struct list_head node;
	u64 id;
	u32 *values;
	unsigned int n_values;
	struct coreboot_cfr_setting *parent;
};

struct coreboot_cfr_setting {
	struct kobject kobj;
	struct list_head node;
	struct coreboot_cfr_drvdata *drvdata;
	u64 object_id;
	enum coreboot_cfr_setting_type type;
	char *name;
	char *display_name;
	struct coreboot_cfr_enum *values;
	unsigned int n_values;
	u32 default_value;
	u32 min;
	u32 max;
	u32 step;
	u32 runtime_apply_method;
	u32 runtime_apply_id;
	u32 access_token;
	u32 access_permissions;
	struct list_head dependencies;
	u8 dependency_visit;
	bool base_read_only;
	bool read_only;
	bool expose;
	bool needed;
	bool available;
	bool registered;
};

struct coreboot_cfr_drvdata {
	struct device *class_dev;
	struct kset *attrs_kset;
	struct list_head settings;
	const struct coreboot_cfr_backend_ops *backend;
	/* Serializes writes and matching state refreshes. */
	struct mutex lock;
	bool faulted;
	bool pending_reboot;
};

struct coreboot_cfr_iterator {
	const u8 *cursor;
	size_t remaining;
};

static struct coreboot_cfr_setting *to_coreboot_cfr_setting(struct kobject *kobj)
{
	return container_of(kobj, struct coreboot_cfr_setting, kobj);
}

static bool coreboot_cfr_string_is_valid_name(const char *name)
{
	return name && name[0] && !strchr(name, '/') &&
		strcmp(name, "pending_reboot") != 0;
}

static bool coreboot_cfr_string_is_valid_label(const char *label)
{
	return label && !strchr(label, ';') && !strchr(label, '\n');
}

static char *coreboot_cfr_string_dup(const struct lb_cfr_varbinary *str)
{
	const char *data = (const char *)(str + 1);

	return kmemdup_nul(data, str->data_length - 1, GFP_KERNEL);
}

static const struct coreboot_table_entry *
coreboot_cfr_next_entry(struct coreboot_cfr_iterator *iterator)
{
	const struct coreboot_table_entry *entry;

	if (!iterator->remaining)
		return NULL;

	if (iterator->remaining < sizeof(*entry))
		return ERR_PTR(-EINVAL);

	entry = (const struct coreboot_table_entry *)iterator->cursor;
	if (entry->size < sizeof(*entry) || entry->size > iterator->remaining)
		return ERR_PTR(-EINVAL);

	iterator->cursor += entry->size;
	iterator->remaining -= entry->size;

	return entry;
}

static const struct coreboot_table_entry *
coreboot_cfr_child_entry(const void *base, size_t len, u32 tag)
{
	struct coreboot_cfr_iterator iterator = {
		.cursor = base,
		.remaining = len,
	};
	const struct coreboot_table_entry *entry;

	for (;;) {
		entry = coreboot_cfr_next_entry(&iterator);
		if (IS_ERR_OR_NULL(entry))
			return entry;

		if (entry->tag == tag)
			return entry;
	}
}

static const struct lb_cfr_varbinary *
coreboot_cfr_child_varbinary(const void *base, size_t len, u32 tag)
{
	const struct lb_cfr_varbinary *value;
	const struct coreboot_table_entry *entry;

	entry = coreboot_cfr_child_entry(base, len, tag);
	if (IS_ERR(entry))
		return ERR_CAST(entry);
	if (!entry)
		return NULL;

	if (entry->size < sizeof(*value))
		return ERR_PTR(-EINVAL);

	value = (const struct lb_cfr_varbinary *)entry;
	if (value->data_length > entry->size - sizeof(*value))
		return ERR_PTR(-EINVAL);

	return value;
}

static const struct lb_cfr_varbinary *
coreboot_cfr_child_string(const void *base, size_t len, u32 tag)
{
	const struct lb_cfr_varbinary *str;

	str = coreboot_cfr_child_varbinary(base, len, tag);
	if (IS_ERR_OR_NULL(str))
		return str;

	if (!str->data_length || ((const u8 *)(str + 1))[str->data_length - 1] ||
	    memchr(str + 1, '\0', str->data_length - 1))
		return ERR_PTR(-EINVAL);

	return str;
}

static const struct lb_cfr_runtime_apply *
coreboot_cfr_child_runtime_apply(const void *base, size_t len)
{
	const struct lb_cfr_runtime_apply *runtime_apply;
	const struct coreboot_table_entry *entry;

	entry = coreboot_cfr_child_entry(base, len, CFR_TAG_RUNTIME_APPLY);
	if (IS_ERR(entry))
		return ERR_CAST(entry);
	if (!entry)
		return NULL;

	if (entry->size != sizeof(*runtime_apply))
		return ERR_PTR(-EINVAL);

	runtime_apply = (const struct lb_cfr_runtime_apply *)entry;
	if (runtime_apply->method == CFR_RUNTIME_APPLY_APM_CNT &&
	    runtime_apply->id > U8_MAX)
		return ERR_PTR(-EINVAL);

	return runtime_apply;
}

static const struct lb_cfr_option_access *
coreboot_cfr_child_option_access(const void *base, size_t len)
{
	const struct lb_cfr_option_access *access;
	const struct coreboot_table_entry *entry;

	entry = coreboot_cfr_child_entry(base, len, CFR_TAG_OPTION_ACCESS);
	if (IS_ERR(entry))
		return ERR_CAST(entry);
	if (!entry)
		return NULL;

	if (entry->size != sizeof(*access))
		return ERR_PTR(-EINVAL);

	access = (const struct lb_cfr_option_access *)entry;
	if (get_unaligned_le32(&access->tag) != CFR_TAG_OPTION_ACCESS ||
	    get_unaligned_le32(&access->size) != sizeof(*access) ||
	    get_unaligned_le32(&access->version) != CFR_OPTION_ACCESS_VERSION ||
	    !get_unaligned_le32(&access->token) ||
	    !(get_unaligned_le32(&access->permissions) & CFR_OPTION_ACCESS_READ) ||
	    get_unaligned_le32(&access->permissions) &
	    ~CFR_OPTION_ACCESS_PERMISSIONS_MASK ||
	    get_unaligned_le32(&access->reserved))
		return ERR_PTR(-EINVAL);

	return access;
}

static int coreboot_cfr_refresh_dependencies(struct coreboot_cfr_drvdata *data);

static bool
coreboot_cfr_setting_is_read_only(const struct coreboot_cfr_setting *setting)
{
	return setting->read_only || setting->drvdata->faulted;
}

u32 coreboot_cfr_setting_access_token(const struct coreboot_cfr_setting *setting)
{
	return setting->access_token;
}

bool coreboot_cfr_setting_access_writable(const struct coreboot_cfr_setting *setting)
{
	return setting->access_permissions & CFR_OPTION_ACCESS_WRITE;
}

#if IS_ENABLED(CONFIG_EFI)
static efi_guid_t coreboot_cfr_guid = EFI_GUID(0xceae4c1d, 0x335b, 0x4685,
					       0xa4, 0xa0, 0xfc, 0x4a,
					       0x94, 0xee, 0xa0, 0x85);

static efi_char16_t *coreboot_cfr_efi_name(const char *name)
{
	size_t len, i;

	len = strlen(name);
	if (len >= EFI_VAR_NAME_LEN)
		return ERR_PTR(-ENAMETOOLONG);

	efi_char16_t *efi_name __free(kfree) =
		kcalloc(len + 1, sizeof(*efi_name), GFP_KERNEL);
	if (!efi_name)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < len; i++) {
		if (!isascii(name[i]))
			return ERR_PTR(-EINVAL);
		efi_name[i] = name[i];
	}

	return no_free_ptr(efi_name);
}

/* Caller must hold the efivar lock. */
static int coreboot_cfr_read_efi_value(efi_char16_t *efi_name, u32 *value,
				       u32 *attrs)
{
	unsigned long size = sizeof(__le32);
	efi_status_t status;
	__le32 data;
	u32 attr;

	status = efivar_get_variable(efi_name, &coreboot_cfr_guid, &attr,
				     &size, &data);
	if (status != EFI_SUCCESS)
		return efi_status_to_err(status);

	if (size != sizeof(data))
		return -EINVAL;

	if (!(attr & EFI_VARIABLE_RUNTIME_ACCESS))
		return -EOPNOTSUPP;

	*value = le32_to_cpu(data);
	if (attrs)
		*attrs = attr;

	return 0;
}

static int coreboot_cfr_efi_read(struct coreboot_cfr_setting *setting, u32 *value,
				 struct coreboot_cfr_read_result *result)
{
	efi_char16_t *efi_name __free(kfree) =
		coreboot_cfr_efi_name(setting->name);
	int ret;

	(void)result;

	if (IS_ERR(efi_name))
		return PTR_ERR(no_free_ptr(efi_name));

	ret = efivar_lock();
	if (ret)
		return ret;

	ret = coreboot_cfr_read_efi_value(efi_name, value, NULL);
	efivar_unlock();

	return ret;
}

/* Caller must hold the efivar lock. */
static int coreboot_cfr_write_efi_value(efi_char16_t *efi_name, u32 value,
					u32 attrs)
{
	efi_status_t status;
	__le32 data;

	if (!efivar_supports_writes())
		return -EROFS;

	data = cpu_to_le32(value);
	status = efivar_set_variable_locked(efi_name, &coreboot_cfr_guid, attrs,
					    sizeof(data), &data, false);
	if (status != EFI_SUCCESS)
		return efi_status_to_err(status);

	return 0;
}

static int coreboot_cfr_efi_apply_runtime(struct coreboot_cfr_setting *setting)
{
	u8 status;

	if (setting->runtime_apply_method != CFR_RUNTIME_APPLY_APM_CNT)
		return -EOPNOTSUPP;

	outb((u8)setting->runtime_apply_id, COREBOOT_CFR_APM_STS_PORT);
	outb(COREBOOT_CFR_APM_APPLY_CMD, COREBOOT_CFR_APM_CNT_PORT);
	status = inb(COREBOOT_CFR_APM_STS_PORT);
	if (status)
		return -EIO;

	return 0;
}

static int coreboot_cfr_efi_write(struct coreboot_cfr_setting *setting, u32 value,
				  struct coreboot_cfr_write_result *result)
{
	efi_char16_t *efi_name;
	u32 attrs;
	u32 old;
	int restore_ret;
	int ret;

	efi_name = coreboot_cfr_efi_name(setting->name);
	if (IS_ERR(efi_name))
		return PTR_ERR(efi_name);

	ret = efivar_lock();
	if (ret)
		goto out_free_name;

	ret = coreboot_cfr_read_efi_value(efi_name, &old, &attrs);
	if (ret)
		goto out_unlock_efi;

	if ((attrs & COREBOOT_CFR_EFI_ATTRS) != COREBOOT_CFR_EFI_ATTRS) {
		ret = -EOPNOTSUPP;
		goto out_unlock_efi;
	}

	if (old == value) {
		result->outcome = COREBOOT_CFR_ATOMIC_UNCHANGED;
		goto out_unlock_efi;
	}

	ret = coreboot_cfr_write_efi_value(efi_name, value, attrs);
	if (ret)
		goto out_unlock_efi;

	ret = coreboot_cfr_efi_apply_runtime(setting);
	if (ret == -EOPNOTSUPP) {
		/* EFI changed; firmware will consume it after reboot. */
		setting->drvdata->pending_reboot = true;
		result->outcome = COREBOOT_CFR_ATOMIC_REBOOT_REQUIRED;
		ret = 0;
	} else if (!ret) {
		result->outcome = COREBOOT_CFR_ATOMIC_APPLIED;
	} else if (ret) {
		restore_ret = coreboot_cfr_write_efi_value(efi_name, old, attrs);
		if (restore_ret) {
			setting->drvdata->pending_reboot = true;
			result->outcome = COREBOOT_CFR_ATOMIC_REBOOT_REQUIRED;
			ret = restore_ret;
		} else {
			result->outcome = COREBOOT_CFR_ATOMIC_ROLLED_BACK;
			result->status = ret;
			goto out_unlock_efi;
		}
	}

	efivar_unlock();
	goto out_free_name;

out_unlock_efi:
	efivar_unlock();
out_free_name:
	kfree(efi_name);
	return ret;
}

static bool coreboot_cfr_efi_writes_supported(struct coreboot_cfr_drvdata *data)
{
	(void)data;
	return efivar_supports_writes();
}

static const struct coreboot_cfr_backend_ops coreboot_cfr_efi_backend = {
	.read = coreboot_cfr_efi_read,
	.write = coreboot_cfr_efi_write,
	.writes_supported = coreboot_cfr_efi_writes_supported,
};
#endif

static int coreboot_cfr_read_value(struct coreboot_cfr_setting *setting, u32 *value)
{
	struct coreboot_cfr_drvdata *data = setting->drvdata;
	struct coreboot_cfr_read_result result = { };
	int ret;

	ret = data->backend->read(setting, value, &result);
	if (!result.faulted)
		return ret;

	data->faulted = true;
	coreboot_cfr_refresh_dependencies(data);
	return ret;
}

static int coreboot_cfr_write_value(struct coreboot_cfr_setting *setting, u32 value)
{
	struct coreboot_cfr_drvdata *data = setting->drvdata;
	struct coreboot_cfr_write_result result = {
		.outcome = COREBOOT_CFR_ATOMIC_REJECTED,
	};
	int transaction_ret = 0;
	int ret;

	mutex_lock(&data->lock);
	if (coreboot_cfr_setting_is_read_only(setting)) {
		ret = -EACCES;
		goto out_unlock;
	}

	ret = data->backend->write(setting, value, &result);
	if (ret)
		goto out_unlock;

	if (data->backend->atomic) {
		switch (result.outcome) {
		case COREBOOT_CFR_ATOMIC_UNCHANGED:
		case COREBOOT_CFR_ATOMIC_APPLIED:
			break;
		case COREBOOT_CFR_ATOMIC_REBOOT_REQUIRED:
			data->pending_reboot = true;
			break;
		case COREBOOT_CFR_ATOMIC_ROLLED_BACK:
			transaction_ret = result.status ?: -EIO;
			break;
		case COREBOOT_CFR_ATOMIC_INDETERMINATE:
			data->faulted = true;
			coreboot_cfr_refresh_dependencies(data);
			ret = result.status ?: -EIO;
			goto out_unlock;
		case COREBOOT_CFR_ATOMIC_REJECTED:
		default:
			ret = result.status ?: -EIO;
			goto out_unlock;
		}
	}

	ret = coreboot_cfr_refresh_dependencies(data);
	if (!ret)
		if (!data->backend->atomic ||
		    result.outcome == COREBOOT_CFR_ATOMIC_APPLIED ||
		    result.outcome == COREBOOT_CFR_ATOMIC_REBOOT_REQUIRED)
			kobject_uevent(&data->class_dev->kobj, KOBJ_CHANGE);
	if (!ret)
		ret = transaction_ret;

out_unlock:
	mutex_unlock(&data->lock);
	return ret;
}

static const char *
coreboot_cfr_label_from_value(const struct coreboot_cfr_setting *setting,
			      u32 value)
{
	unsigned int i;

	for (i = 0; i < setting->n_values; i++) {
		if (setting->values[i].value == value)
			return setting->values[i].label;
	}

	return NULL;
}

static int coreboot_cfr_parse_value(struct coreboot_cfr_setting *setting,
				    const char *label, u32 *value_out)
{
	unsigned int i;

	for (i = 0; i < setting->n_values; i++) {
		if (!sysfs_streq(label, setting->values[i].label))
			continue;

		*value_out = setting->values[i].value;
		return 0;
	}

	return kstrtou32(label, 0, value_out);
}

static bool coreboot_cfr_value_is_valid(struct coreboot_cfr_setting *setting,
					u32 value)
{
	u32 delta;

	if (setting->type != COREBOOT_CFR_SETTING_NUMBER) {
		/* Enum and bool values are valid only if they have a label. */
		return coreboot_cfr_label_from_value(setting, value);
	}

	if (value < setting->min || value > setting->max)
		return false;

	if (!setting->step)
		return true;

	delta = value - setting->min;
	return delta % setting->step == 0;
}

static ssize_t type_show(struct kobject *kobj, struct kobj_attribute *attr,
			 char *buf)
{
	struct coreboot_cfr_setting *setting = to_coreboot_cfr_setting(kobj);

	if (setting->type == COREBOOT_CFR_SETTING_NUMBER)
		return sysfs_emit(buf, "integer\n");

	return sysfs_emit(buf, "enumeration\n");
}

static ssize_t display_name_language_code_show(struct kobject *kobj,
					       struct kobj_attribute *attr,
					       char *buf)
{
	return sysfs_emit(buf, "en_US.UTF-8\n");
}

static ssize_t display_name_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	struct coreboot_cfr_setting *setting = to_coreboot_cfr_setting(kobj);

	return sysfs_emit(buf, "%s\n", setting->display_name);
}

static ssize_t possible_values_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	struct coreboot_cfr_setting *setting = to_coreboot_cfr_setting(kobj);
	ssize_t len = 0;
	unsigned int i;

	for (i = 0; i < setting->n_values; i++) {
		len += sysfs_emit_at(buf, len, "%s%s", i ? ";" : "",
				     setting->values[i].label);
	}

	len += sysfs_emit_at(buf, len, "\n");
	return len;
}

static ssize_t min_value_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	struct coreboot_cfr_setting *setting = to_coreboot_cfr_setting(kobj);

	return sysfs_emit(buf, "%u\n", setting->min);
}

static ssize_t max_value_show(struct kobject *kobj, struct kobj_attribute *attr,
			      char *buf)
{
	struct coreboot_cfr_setting *setting = to_coreboot_cfr_setting(kobj);

	return sysfs_emit(buf, "%u\n", setting->max);
}

static ssize_t scalar_increment_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	struct coreboot_cfr_setting *setting = to_coreboot_cfr_setting(kobj);

	return sysfs_emit(buf, "%u\n", setting->step);
}

static ssize_t default_value_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	struct coreboot_cfr_setting *setting = to_coreboot_cfr_setting(kobj);
	const char *label;

	if (setting->type == COREBOOT_CFR_SETTING_NUMBER)
		return sysfs_emit(buf, "%u\n", setting->default_value);

	label = coreboot_cfr_label_from_value(setting, setting->default_value);
	if (!label)
		return sysfs_emit(buf, "%u\n", setting->default_value);

	return sysfs_emit(buf, "%s\n", label);
}

static ssize_t current_value_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf)
{
	struct coreboot_cfr_setting *setting = to_coreboot_cfr_setting(kobj);
	struct coreboot_cfr_drvdata *data = setting->drvdata;
	const char *label;
	ssize_t len;
	u32 value;
	int ret;

	mutex_lock(&data->lock);
	ret = coreboot_cfr_read_value(setting, &value);
	if (ret) {
		len = ret;
		goto out_unlock;
	}

	if (setting->type == COREBOOT_CFR_SETTING_NUMBER) {
		len = sysfs_emit(buf, "%u\n", value);
		goto out_unlock;
	}

	label = coreboot_cfr_label_from_value(setting, value);
	if (!label) {
		len = -EINVAL;
		goto out_unlock;
	}

	len = sysfs_emit(buf, "%s\n", label);

out_unlock:
	mutex_unlock(&data->lock);
	return len;
}

static ssize_t current_value_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	struct coreboot_cfr_setting *setting = to_coreboot_cfr_setting(kobj);
	u32 value;
	int ret;

	ret = coreboot_cfr_parse_value(setting, buf, &value);
	if (ret)
		return ret;

	if (!coreboot_cfr_value_is_valid(setting, value))
		return -EINVAL;

	ret = coreboot_cfr_write_value(setting, value);
	if (ret)
		return ret;

	return count;
}

static struct kobj_attribute type_attr = __ATTR_RO(type);
static struct kobj_attribute display_name_language_code_attr =
	__ATTR_RO(display_name_language_code);
static struct kobj_attribute display_name_attr = __ATTR_RO(display_name);
static struct kobj_attribute possible_values_attr = __ATTR_RO(possible_values);
static struct kobj_attribute min_value_attr = __ATTR_RO(min_value);
static struct kobj_attribute max_value_attr = __ATTR_RO(max_value);
static struct kobj_attribute scalar_increment_attr = __ATTR_RO(scalar_increment);
static struct kobj_attribute default_value_attr = __ATTR_RO(default_value);
static struct kobj_attribute current_value_attr = __ATTR_RW(current_value);

static struct attribute *coreboot_cfr_setting_attrs[] = {
	&type_attr.attr,
	&display_name_language_code_attr.attr,
	&display_name_attr.attr,
	&possible_values_attr.attr,
	&min_value_attr.attr,
	&max_value_attr.attr,
	&scalar_increment_attr.attr,
	&default_value_attr.attr,
	&current_value_attr.attr,
	NULL,
};

static umode_t coreboot_cfr_attr_is_visible(struct kobject *kobj,
					    struct attribute *attr, int n)
{
	struct coreboot_cfr_setting *setting = to_coreboot_cfr_setting(kobj);

	if (setting->type == COREBOOT_CFR_SETTING_NUMBER &&
	    attr == &possible_values_attr.attr)
		return 0;

	if (setting->type != COREBOOT_CFR_SETTING_NUMBER &&
	    (attr == &min_value_attr.attr || attr == &max_value_attr.attr ||
	     attr == &scalar_increment_attr.attr))
		return 0;

	if (coreboot_cfr_setting_is_read_only(setting) &&
	    attr == &current_value_attr.attr)
		return 0444;

	return attr->mode;
}

static const struct attribute_group coreboot_cfr_setting_group = {
	.attrs = coreboot_cfr_setting_attrs,
	.is_visible = coreboot_cfr_attr_is_visible,
};

static void coreboot_cfr_free_dependencies(struct list_head *dependencies)
{
	struct coreboot_cfr_dependency *dependency, *tmp;

	list_for_each_entry_safe(dependency, tmp, dependencies, node) {
		list_del(&dependency->node);
		kfree(dependency->values);
		kfree(dependency);
	}
}

static void coreboot_cfr_free_setting(struct coreboot_cfr_setting *setting)
{
	unsigned int i;

	for (i = 0; i < setting->n_values; i++)
		kfree(setting->values[i].label);

	kfree(setting->values);
	kfree(setting->display_name);
	kfree(setting->name);
	coreboot_cfr_free_dependencies(&setting->dependencies);
	kfree(setting);
}

static void coreboot_cfr_setting_release(struct kobject *kobj)
{
	coreboot_cfr_free_setting(to_coreboot_cfr_setting(kobj));
}

static const struct kobj_type coreboot_cfr_setting_ktype = {
	.release = coreboot_cfr_setting_release,
	.sysfs_ops = &kobj_sysfs_ops,
};

static ssize_t pending_reboot_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	struct device *dev = kobj_to_dev(kobj->parent);
	struct coreboot_cfr_drvdata *data;

	data = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", READ_ONCE(data->pending_reboot));
}

static struct kobj_attribute pending_reboot_attr = __ATTR_RO(pending_reboot);

static int coreboot_cfr_copy_bool_values(struct coreboot_cfr_setting *setting)
{
	static const struct coreboot_cfr_enum bool_values[] = {
		{ .label = "Disabled", .value = 0 },
		{ .label = "Enabled", .value = 1 },
	};
	unsigned int i;

	setting->values = kcalloc(ARRAY_SIZE(bool_values), sizeof(*setting->values),
				  GFP_KERNEL);
	if (!setting->values)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(bool_values); i++) {
		setting->values[i].label = kstrdup(bool_values[i].label,
						   GFP_KERNEL);
		if (!setting->values[i].label)
			return -ENOMEM;
		setting->values[i].value = bool_values[i].value;
		setting->n_values++;
	}

	return 0;
}

static int coreboot_cfr_count_enum_values(const void *base, size_t len)
{
	struct coreboot_cfr_iterator iterator = {
		.cursor = base,
		.remaining = len,
	};
	const struct coreboot_table_entry *entry;
	int count = 0;

	for (;;) {
		entry = coreboot_cfr_next_entry(&iterator);
		if (IS_ERR(entry))
			return PTR_ERR(entry);
		if (!entry)
			return count;

		if (entry->tag == CFR_TAG_ENUM_VALUE)
			count++;
	}
}

static int coreboot_cfr_copy_enum_values(struct coreboot_cfr_setting *setting,
					 const void *base, size_t len)
{
	struct coreboot_cfr_iterator iterator = {
		.cursor = base,
		.remaining = len,
	};
	const struct lb_cfr_enum_value *enum_value;
	const struct lb_cfr_varbinary *label;
	const struct coreboot_table_entry *entry;
	struct coreboot_cfr_enum *value;
	int count;

	count = coreboot_cfr_count_enum_values(base, len);
	if (count <= 0)
		return count ?: -EINVAL;

	setting->values = kcalloc(count, sizeof(*setting->values), GFP_KERNEL);
	if (!setting->values)
		return -ENOMEM;

	for (;;) {
		entry = coreboot_cfr_next_entry(&iterator);
		if (IS_ERR(entry))
			return PTR_ERR(entry);
		if (!entry)
			return 0;

		if (entry->tag != CFR_TAG_ENUM_VALUE)
			continue;

		if (entry->size < sizeof(*enum_value))
			return -EINVAL;

		enum_value = (const struct lb_cfr_enum_value *)entry;
		label = coreboot_cfr_child_string(enum_value + 1,
						  enum_value->size - sizeof(*enum_value),
						  CFR_TAG_VARCHAR_UI_NAME);
		if (IS_ERR(label))
			return PTR_ERR(label);
		if (!label)
			return -EINVAL;

		value = &setting->values[setting->n_values];
		value->label = coreboot_cfr_string_dup(label);
		if (!value->label)
			return -ENOMEM;
		if (!coreboot_cfr_string_is_valid_label(value->label)) {
			kfree(value->label);
			value->label = NULL;
			return -EINVAL;
		}

		value->value = enum_value->value;
		setting->n_values++;
	}
}

static bool
coreboot_cfr_possible_values_fit(const struct coreboot_cfr_setting *setting)
{
	size_t len = 1;		/* Trailing newline. */
	size_t label_len;
	unsigned int i;

	for (i = 0; i < setting->n_values; i++) {
		if (len >= PAGE_SIZE)
			return false;

		if (i)
			len++;

		label_len = strlen(setting->values[i].label);
		if (label_len >= PAGE_SIZE - len)
			return false;

		len += label_len;
	}

	return true;
}

static int coreboot_cfr_setting_is_usable(struct coreboot_cfr_setting *setting)
{
	u32 value;
	int ret;

	if (!setting->name)
		return -ENOENT;

	ret = coreboot_cfr_read_value(setting, &value);
	if (ret)
		return ret;

	if (!coreboot_cfr_value_is_valid(setting, value))
		return -EINVAL;

	return 0;
}

static int coreboot_cfr_add_dependency(struct coreboot_cfr_setting *setting,
				       u64 id, const void *base, size_t len)
{
	const struct lb_cfr_varbinary *values;
	struct coreboot_cfr_dependency *dependency;
	const u8 *raw;
	unsigned int i;

	if (!id)
		return 0;

	values = coreboot_cfr_child_varbinary(base, len, CFR_TAG_DEP_VALUES);
	if (IS_ERR(values))
		return PTR_ERR(values);

	dependency = kzalloc_obj(*dependency, GFP_KERNEL);
	if (!dependency)
		return -ENOMEM;

	dependency->id = id;
	if (values) {
		if (!values->data_length || values->data_length % sizeof(u32)) {
			kfree(dependency);
			return -EINVAL;
		}

		dependency->n_values = values->data_length / sizeof(u32);
		dependency->values = kmalloc_array(dependency->n_values,
						   sizeof(*dependency->values), GFP_KERNEL);
		if (!dependency->values) {
			kfree(dependency);
			return -ENOMEM;
		}

		raw = (const u8 *)(values + 1);
		for (i = 0; i < dependency->n_values; i++)
			dependency->values[i] = get_unaligned_le32(raw + i * sizeof(u32));
	}

	list_add_tail(&dependency->node, &setting->dependencies);
	return 0;
}

static int coreboot_cfr_copy_dependencies(struct coreboot_cfr_setting *setting,
					  const struct list_head *source)
{
	const struct coreboot_cfr_dependency *old;
	struct coreboot_cfr_dependency *new;

	list_for_each_entry(old, source, node) {
		new = kmemdup(old, sizeof(*new), GFP_KERNEL);
		if (!new)
			return -ENOMEM;
		new->values = kmemdup(old->values,
				      array_size(old->n_values, sizeof(*new->values)),
				      GFP_KERNEL);
		if (old->n_values && !new->values) {
			kfree(new);
			return -ENOMEM;
		}
		new->parent = NULL;
		list_add_tail(&new->node, &setting->dependencies);
	}

	return 0;
}

static struct coreboot_cfr_setting *
coreboot_cfr_find_setting(struct coreboot_cfr_drvdata *data, u64 object_id)
{
	struct coreboot_cfr_setting *setting;

	list_for_each_entry(setting, &data->settings, node) {
		if (setting->object_id == object_id)
			return setting;
	}

	return NULL;
}

static int coreboot_cfr_validate_dependency_visit(struct coreboot_cfr_setting *setting,
						  unsigned int depth)
{
	struct coreboot_cfr_dependency *dependency;
	int ret;

	if (depth > COREBOOT_CFR_MAX_DEPENDENCY_DEPTH)
		return -E2BIG;

	if (setting->dependency_visit == 1)
		return -ELOOP;
	if (setting->dependency_visit == 2)
		return 0;

	setting->dependency_visit = 1;
	list_for_each_entry(dependency, &setting->dependencies, node) {
		ret = coreboot_cfr_validate_dependency_visit(dependency->parent,
							     depth + 1);
		if (ret)
			return ret;
	}
	setting->dependency_visit = 2;

	return 0;
}

static int coreboot_cfr_validate_dependencies(struct coreboot_cfr_drvdata *data)
{
	struct coreboot_cfr_setting *setting, *other;
	struct coreboot_cfr_dependency *dependency;
	unsigned int i;
	int ret;

	list_for_each_entry(setting, &data->settings, node) {
		list_for_each_entry(other, &data->settings, node) {
			if (setting->object_id && setting != other &&
			    setting->object_id == other->object_id)
				return -EINVAL;
		}

		list_for_each_entry(dependency, &setting->dependencies, node) {
			dependency->parent = coreboot_cfr_find_setting(data, dependency->id);
			if (!dependency->parent)
				return -EINVAL;
			for (i = 0; i < dependency->n_values; i++) {
				if (!coreboot_cfr_value_is_valid(dependency->parent,
								 dependency->values[i]))
					return -EINVAL;
			}
		}
	}

	list_for_each_entry(setting, &data->settings, node) {
		setting->dependency_visit = 0;
		ret = coreboot_cfr_validate_dependency_visit(setting, 0);
		if (ret)
			return ret;
	}

	return 0;
}

static bool coreboot_cfr_skip_unusable_error(int ret)
{
	return ret == -ENOENT || ret == -EINVAL || ret == -EOPNOTSUPP ||
	       ret == -ENAMETOOLONG;
}

static int coreboot_cfr_mark_setting_needed(struct coreboot_cfr_setting *setting,
					    unsigned int depth)
{
	struct coreboot_cfr_dependency *dependency;
	int ret;

	if (depth > COREBOOT_CFR_MAX_DEPENDENCY_DEPTH)
		return -E2BIG;

	if (setting->needed)
		return 0;

	setting->needed = true;
	list_for_each_entry(dependency, &setting->dependencies, node) {
		ret = coreboot_cfr_mark_setting_needed(dependency->parent, depth + 1);
		if (ret)
			return ret;
	}

	return 0;
}

static int coreboot_cfr_prepare_settings(struct coreboot_cfr_drvdata *data)
{
	struct coreboot_cfr_setting *setting, *tmp;
	struct coreboot_cfr_dependency *dependency;
	bool changed;
	int ret;

	list_for_each_entry(setting, &data->settings, node) {
		setting->needed = false;
		setting->available = true;
	}

	list_for_each_entry(setting, &data->settings, node) {
		if (!setting->expose)
			continue;

		ret = coreboot_cfr_mark_setting_needed(setting, 0);
		if (ret)
			return ret;
	}

	list_for_each_entry(setting, &data->settings, node) {
		if (!setting->needed) {
			setting->available = false;
			continue;
		}

		ret = coreboot_cfr_setting_is_usable(setting);
		if (!ret)
			continue;

		if (!coreboot_cfr_skip_unusable_error(ret))
			return ret;

		setting->available = false;
	}

	do {
		changed = false;
		list_for_each_entry(setting, &data->settings, node) {
			if (!setting->available)
				continue;

			list_for_each_entry(dependency, &setting->dependencies, node) {
				if (dependency->parent->available)
					continue;
				setting->available = false;
				changed = true;
				break;
			}
		}
	} while (changed);

	list_for_each_entry_safe(setting, tmp, &data->settings, node) {
		if (setting->available)
			continue;

		list_del(&setting->node);
		coreboot_cfr_free_setting(setting);
	}

	return 0;
}

static int coreboot_cfr_refresh_dependencies(struct coreboot_cfr_drvdata *data)
{
	struct coreboot_cfr_setting *setting;
	struct coreboot_cfr_dependency *dependency;
	u32 value;
	unsigned int i;
	bool active;
	int ret;

	if (data->faulted) {
		list_for_each_entry(setting, &data->settings, node) {
			setting->read_only = true;
			if (setting->registered) {
				ret = sysfs_chmod_file(&setting->kobj,
						       &current_value_attr.attr, 0444);
				if (ret)
					return ret;
			}
		}
		return 0;
	}

	list_for_each_entry(setting, &data->settings, node) {
		active = true;
		list_for_each_entry(dependency, &setting->dependencies, node) {
			ret = coreboot_cfr_read_value(dependency->parent, &value);
			if (ret)
				return ret;

			if (!dependency->n_values) {
				active &= value != 0;
				continue;
			}

			for (i = 0; i < dependency->n_values; i++) {
				if (value == dependency->values[i])
					break;
			}
			active &= i != dependency->n_values;
		}

		setting->read_only = setting->base_read_only || !active;
		if (setting->registered) {
			ret = sysfs_chmod_file(&setting->kobj, &current_value_attr.attr,
					       setting->read_only ? 0444 : 0644);
			if (ret)
				return ret;
		}
	}

	return 0;
}

static int coreboot_cfr_register_setting(struct coreboot_cfr_drvdata *data,
					 struct coreboot_cfr_setting *setting)
{
	int ret;

	ret = kobject_init_and_add(&setting->kobj, &coreboot_cfr_setting_ktype,
				   &data->attrs_kset->kobj, "%s", setting->name);
	if (ret)
		goto err_put_kobj;

	ret = sysfs_create_group(&setting->kobj, &coreboot_cfr_setting_group);
	if (ret)
		goto err_put_kobj;

	setting->registered = true;
	return 0;

err_put_kobj:
	list_del_init(&setting->node);
	kobject_put(&setting->kobj);
	return ret;
}

static int coreboot_cfr_add_numeric_option(struct coreboot_cfr_drvdata *data,
					   const struct lb_cfr_numeric_option *option,
					   bool parent_read_only,
					   bool parent_expose,
					   const struct list_head *parent_dependencies)
{
	const struct lb_cfr_varbinary *name;
	const struct lb_cfr_varbinary *display_name;
	const struct lb_cfr_runtime_apply *runtime_apply;
	const struct lb_cfr_option_access *access = NULL;
	const void *child_base = option + 1;
	struct coreboot_cfr_setting *setting;
	size_t child_len = option->size - sizeof(*option);
	int ret;

	setting = kzalloc_obj(*setting, GFP_KERNEL);
	if (!setting)
		return -ENOMEM;

	INIT_LIST_HEAD(&setting->node);
	INIT_LIST_HEAD(&setting->dependencies);
	setting->drvdata = data;
	setting->object_id = option->object_id;
	setting->default_value = option->default_value;
	setting->min = option->min;
	setting->max = option->max;
	setting->step = option->step ?: 1;
	setting->expose = parent_expose &&
		(option->flags & CFR_OPTFLAG_RUNTIME) &&
		!(option->flags & CFR_OPTFLAG_SUPPRESS);
	ret = coreboot_cfr_copy_dependencies(setting, parent_dependencies);
	if (ret)
		goto err_put_setting;

	access = coreboot_cfr_child_option_access(child_base, child_len);
	if (IS_ERR(access)) {
		ret = PTR_ERR(access);
		goto err_put_setting;
	}
	if (access) {
		setting->access_token = get_unaligned_le32(&access->token);
		setting->access_permissions =
			get_unaligned_le32(&access->permissions);
	}

	if (!data->backend->atomic) {
		runtime_apply = coreboot_cfr_child_runtime_apply(child_base, child_len);
		if (IS_ERR(runtime_apply)) {
			ret = PTR_ERR(runtime_apply);
			goto err_put_setting;
		}

		if (runtime_apply && runtime_apply->method == CFR_RUNTIME_APPLY_APM_CNT) {
			setting->runtime_apply_method = runtime_apply->method;
			setting->runtime_apply_id = runtime_apply->id;
		}
	}

	setting->base_read_only =
		(option->flags & COREBOOT_CFR_OPT_READ_ONLY_FLAGS) ||
		!data->backend->writes_supported(data) || parent_read_only ||
		(data->backend->atomic && !access) ||
		(access && !coreboot_cfr_setting_access_writable(setting));
	setting->read_only = setting->base_read_only;

	ret = coreboot_cfr_add_dependency(setting, option->dependency_id,
					  child_base, child_len);
	if (ret)
		goto err_put_setting;

	name = coreboot_cfr_child_string(child_base, child_len,
					 CFR_TAG_VARCHAR_OPT_NAME);
	if (IS_ERR(name)) {
		ret = PTR_ERR(name);
		goto err_put_setting;
	}
	if (!name && setting->expose) {
		ret = -EINVAL;
		goto err_put_setting;
	}

	if (name) {
		setting->name = coreboot_cfr_string_dup(name);
		if (!setting->name) {
			ret = -ENOMEM;
			goto err_put_setting;
		}

		if (!coreboot_cfr_string_is_valid_name(setting->name)) {
			ret = -EINVAL;
			goto err_put_setting;
		}
	}

	if (setting->expose) {
		display_name = coreboot_cfr_child_string(child_base, child_len,
							 CFR_TAG_VARCHAR_UI_NAME);
		if (IS_ERR(display_name)) {
			ret = PTR_ERR(display_name);
			goto err_put_setting;
		}
		if (display_name)
			setting->display_name = coreboot_cfr_string_dup(display_name);
		else
			setting->display_name = kstrdup(setting->name, GFP_KERNEL);
		if (!setting->display_name) {
			ret = -ENOMEM;
			goto err_put_setting;
		}
	}

	switch (option->tag) {
	case CFR_TAG_OPTION_BOOL:
		setting->type = COREBOOT_CFR_SETTING_BOOL;
		setting->min = 0;
		setting->max = 1;
		setting->step = 1;
		ret = coreboot_cfr_copy_bool_values(setting);
		break;
	case CFR_TAG_OPTION_ENUM:
		setting->type = COREBOOT_CFR_SETTING_ENUM;
		ret = coreboot_cfr_copy_enum_values(setting, child_base,
						    child_len);
		break;
	case CFR_TAG_OPTION_NUMBER:
		setting->type = COREBOOT_CFR_SETTING_NUMBER;
		if (setting->max < setting->min)
			ret = -EINVAL;
		else
			ret = 0;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret)
		goto err_put_setting;
	/* possible_values must be returned completely in one sysfs read. */
	if (setting->expose && setting->type != COREBOOT_CFR_SETTING_NUMBER &&
	    !coreboot_cfr_possible_values_fit(setting)) {
		ret = 0;
		goto err_put_setting;
	}

	list_add_tail(&setting->node, &data->settings);
	return 0;

err_put_setting:
	coreboot_cfr_free_setting(setting);
	return ret;
}

static int coreboot_cfr_parse_records(struct coreboot_cfr_drvdata *data,
					      const void *base, size_t len,
					      unsigned int depth,
					      bool parent_read_only,
					      bool parent_expose,
					      const struct list_head *parent_dependencies)
{
	struct coreboot_cfr_iterator iterator = {
		.cursor = base,
		.remaining = len,
	};
	const struct lb_cfr_option_form *form;
	const struct lb_cfr_numeric_option *option;
	const struct coreboot_table_entry *entry;
	const void *child_base;
	size_t child_len;
	int ret;

	for (;;) {
		entry = coreboot_cfr_next_entry(&iterator);
		if (IS_ERR(entry))
			return PTR_ERR(entry);
		if (!entry)
			return 0;

		switch (entry->tag) {
		case CFR_TAG_OPTION_FORM:
		{
			struct coreboot_cfr_setting context = { };

			if (entry->size < sizeof(struct lb_cfr_option_form))
				return -EINVAL;

			form = (const struct lb_cfr_option_form *)entry;
			if (depth >= COREBOOT_CFR_MAX_FORM_DEPTH)
				return -E2BIG;

			child_base = form + 1;
			child_len = entry->size - sizeof(struct lb_cfr_option_form);
			INIT_LIST_HEAD(&context.dependencies);
			ret = coreboot_cfr_copy_dependencies(&context, parent_dependencies);
			if (!ret)
				ret = coreboot_cfr_add_dependency(
					&context, form->dependency_id, child_base, child_len);
			if (!ret)
				ret = coreboot_cfr_parse_records(data, child_base,
								 child_len, depth + 1,
								 parent_read_only ||
								 (form->flags &
								  COREBOOT_CFR_OPT_READ_ONLY_FLAGS),
								 parent_expose && !(form->flags &
								 CFR_OPTFLAG_SUPPRESS),
								 &context.dependencies);
			coreboot_cfr_free_dependencies(&context.dependencies);
			if (ret)
				return ret;
			break;
		}
		case CFR_TAG_OPTION_ENUM:
		case CFR_TAG_OPTION_NUMBER:
		case CFR_TAG_OPTION_BOOL:
			option = (const struct lb_cfr_numeric_option *)entry;
			if (entry->size < sizeof(*option))
				return -EINVAL;
			ret = coreboot_cfr_add_numeric_option(data, option,
							      parent_read_only,
							      parent_expose,
							      parent_dependencies);
			if (ret)
				return ret;
			break;
		default:
			/* Ignore unsupported, child-only and future record types. */
			break;
		}
	}
}

static int coreboot_cfr_register_settings(struct coreboot_cfr_drvdata *data)
{
	struct coreboot_cfr_setting *setting;
	bool registered = false;
	int ret;

	ret = coreboot_cfr_validate_dependencies(data);
	if (ret)
		return ret;

	ret = coreboot_cfr_prepare_settings(data);
	if (ret)
		return ret;

	ret = coreboot_cfr_refresh_dependencies(data);
	if (ret)
		return ret;

	list_for_each_entry(setting, &data->settings, node) {
		if (!setting->expose)
			continue;

		ret = coreboot_cfr_register_setting(data, setting);
		if (ret)
			return ret;
		registered = true;
	}

	return registered ? 0 : -ENODEV;
}

static void coreboot_cfr_unregister_settings(struct coreboot_cfr_drvdata *data)
{
	struct coreboot_cfr_setting *setting, *tmp;

	list_for_each_entry_safe(setting, tmp, &data->settings, node) {
		list_del(&setting->node);
		if (setting->registered) {
			sysfs_remove_group(&setting->kobj, &coreboot_cfr_setting_group);
			kobject_put(&setting->kobj);
		} else {
			coreboot_cfr_free_setting(setting);
		}
	}
}

static int coreboot_cfr_root_probe(struct coreboot_device *dev)
{
	const struct lb_cfr *root = (const struct lb_cfr *)dev->raw;
	struct coreboot_cfr_drvdata *data;
	LIST_HEAD(root_dependencies);
	size_t payload_len;
	int ret;

	if (dev->entry.size < sizeof(*root))
		return -EINVAL;

	if (root->tag != LB_TAG_CFR_ROOT || root->version != CFR_VERSION)
		return -EINVAL;

	if (root->size < sizeof(*root) || root->size > dev->entry.size)
		return -EINVAL;

	payload_len = root->size - sizeof(*root);
	if (crc32_be(0, root + 1, payload_len) != root->checksum)
		return -EBADMSG;

	data = devm_kzalloc(&dev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	INIT_LIST_HEAD(&data->settings);
	ret = devm_mutex_init(&dev->dev, &data->lock);
	if (ret)
		return ret;

	data->backend = coreboot_cfr_atomic_backend_get(dev);
	if (IS_ERR(data->backend))
		return PTR_ERR(data->backend);
	#if IS_ENABLED(CONFIG_EFI)
	if (!data->backend && efivar_is_available())
		data->backend = &coreboot_cfr_efi_backend;
	#endif
	if (!data->backend)
		return -ENODEV;

	dev_set_drvdata(&dev->dev, data);

	data->class_dev = device_create(&firmware_attributes_class, NULL,
					MKDEV(0, 0), NULL, DRIVER_NAME);
	if (IS_ERR(data->class_dev))
		return PTR_ERR(data->class_dev);
	dev_set_drvdata(data->class_dev, data);

	data->attrs_kset = kset_create_and_add("attributes", NULL,
					       &data->class_dev->kobj);
	if (!data->attrs_kset) {
		ret = -ENOMEM;
		goto err_unregister_dev;
	}

	ret = sysfs_create_file(&data->attrs_kset->kobj,
				&pending_reboot_attr.attr);
	if (ret)
		goto err_unregister_attrs;

	ret = coreboot_cfr_parse_records(data, root + 1, payload_len, 0, false, true,
					 &root_dependencies);
	if (ret)
		goto err_unregister_settings;

	if (list_empty(&data->settings)) {
		ret = -ENODEV;
		goto err_unregister_settings;
	}

	ret = coreboot_cfr_register_settings(data);
	if (ret)
		goto err_unregister_settings;

	return 0;

err_unregister_settings:
	coreboot_cfr_unregister_settings(data);
	sysfs_remove_file(&data->attrs_kset->kobj, &pending_reboot_attr.attr);
err_unregister_attrs:
	kset_unregister(data->attrs_kset);
err_unregister_dev:
	device_unregister(data->class_dev);
	return ret;
}

static void coreboot_cfr_root_remove(struct coreboot_device *dev)
{
	struct coreboot_cfr_drvdata *data = dev_get_drvdata(&dev->dev);

	coreboot_cfr_unregister_settings(data);
	sysfs_remove_file(&data->attrs_kset->kobj, &pending_reboot_attr.attr);
	kset_unregister(data->attrs_kset);
	device_unregister(data->class_dev);
}

static const struct coreboot_device_id coreboot_cfr_ids[] = {
	{ .tag = LB_TAG_CFR_ROOT },
#if IS_ENABLED(CONFIG_X86)
	{ .tag = LB_TAG_CFR_SETTINGS },
#endif
	{ }
};
MODULE_DEVICE_TABLE(coreboot, coreboot_cfr_ids);

static int coreboot_cfr_probe(struct coreboot_device *dev)
{
	switch (dev->entry.tag) {
	case LB_TAG_CFR_ROOT:
		return coreboot_cfr_root_probe(dev);
	case LB_TAG_CFR_SETTINGS:
		return coreboot_cfr_atomic_service_probe(dev);
	default:
		return -ENODEV;
	}
}

static void coreboot_cfr_remove(struct coreboot_device *dev)
{
	switch (dev->entry.tag) {
	case LB_TAG_CFR_ROOT:
		coreboot_cfr_root_remove(dev);
		break;
	case LB_TAG_CFR_SETTINGS:
		coreboot_cfr_atomic_service_remove(dev);
		break;
	}
}

#if IS_ENABLED(CONFIG_COREBOOT_CFR_KUNIT_TEST)
static int coreboot_cfr_kunit_read(struct coreboot_cfr_setting *setting, u32 *value,
				   struct coreboot_cfr_read_result *result)
{
	(void)result;
	*value = setting->default_value;
	return 0;
}

static bool coreboot_cfr_kunit_writes_supported(struct coreboot_cfr_drvdata *data)
{
	(void)data;
	return true;
}

static const struct coreboot_cfr_backend_ops coreboot_cfr_kunit_backend = {
	.read = coreboot_cfr_kunit_read,
	.writes_supported = coreboot_cfr_kunit_writes_supported,
};

static int
coreboot_cfr_kunit_run(const struct coreboot_cfr_kunit_setting *settings,
			unsigned int n_settings, bool prepare)
{
	struct coreboot_cfr_drvdata data = { };
	struct coreboot_cfr_setting *setting;
	struct coreboot_cfr_dependency *dependency;
	unsigned int i, j, remaining = 0;
	int ret;

	if (!settings || !n_settings)
		return -EINVAL;

	INIT_LIST_HEAD(&data.settings);
	data.backend = &coreboot_cfr_kunit_backend;
	for (i = 0; i < n_settings; i++) {
		setting = kzalloc_obj(*setting, GFP_KERNEL);
		if (!setting) {
			ret = -ENOMEM;
			goto out_free_settings;
		}

		INIT_LIST_HEAD(&setting->node);
		INIT_LIST_HEAD(&setting->dependencies);
		setting->object_id = settings[i].object_id;
		setting->drvdata = &data;
		setting->type = COREBOOT_CFR_SETTING_NUMBER;
		setting->min = settings[i].min;
		setting->max = settings[i].max;
		setting->step = settings[i].step ?: 1;
		setting->default_value = setting->min;
		setting->expose = settings[i].expose;
		setting->name = kstrdup("kunit", GFP_KERNEL);
		if (!setting->name) {
			ret = -ENOMEM;
			goto out_free_setting;
		}

		for (j = 0; j < settings[i].n_dependencies; j++) {
			const struct coreboot_cfr_kunit_dependency *input =
				&settings[i].dependencies[j];

			if (input->n_values && !input->values) {
				ret = -EINVAL;
				goto out_free_setting;
			}

			dependency = kzalloc_obj(*dependency, GFP_KERNEL);
			if (!dependency) {
				ret = -ENOMEM;
				goto out_free_setting;
			}
			dependency->id = input->id;
			dependency->n_values = input->n_values;
			dependency->values = kmemdup(input->values,
					array_size(input->n_values,
						   sizeof(*dependency->values)),
					GFP_KERNEL);
			if (input->n_values && !dependency->values) {
				kfree(dependency);
				ret = -ENOMEM;
				goto out_free_setting;
			}
			list_add_tail(&dependency->node, &setting->dependencies);
		}

		list_add_tail(&setting->node, &data.settings);
		continue;

out_free_setting:
		coreboot_cfr_free_setting(setting);
		goto out_free_settings;
	}

	ret = coreboot_cfr_validate_dependencies(&data);
	if (!ret && prepare) {
		ret = coreboot_cfr_prepare_settings(&data);
		if (!ret) {
			list_for_each_entry(setting, &data.settings, node)
				remaining++;
			if (remaining != n_settings)
				ret = -EUCLEAN;
		}
	}
out_free_settings:
	coreboot_cfr_unregister_settings(&data);
	return ret;
}

int
coreboot_cfr_kunit_validate_dependencies(const struct coreboot_cfr_kunit_setting *settings,
					  unsigned int n_settings)
{
	return coreboot_cfr_kunit_run(settings, n_settings, false);
}

int
coreboot_cfr_kunit_prepare_dependencies(const struct coreboot_cfr_kunit_setting *settings,
					 unsigned int n_settings)
{
	return coreboot_cfr_kunit_run(settings, n_settings, true);
}

int coreboot_cfr_kunit_parse_dependency_values(const void *base, size_t len)
{
	struct coreboot_cfr_setting *setting;
	int ret;

	setting = kzalloc_obj(*setting, GFP_KERNEL);
	if (!setting)
		return -ENOMEM;

	INIT_LIST_HEAD(&setting->dependencies);
	ret = coreboot_cfr_add_dependency(setting, 1, base, len);
	coreboot_cfr_free_setting(setting);
	return ret;
}

int coreboot_cfr_kunit_parse_runtime_apply_and_access(const void *base, size_t len)
{
	const struct lb_cfr_runtime_apply *runtime_apply;
	const struct lb_cfr_option_access *access;

	runtime_apply = coreboot_cfr_child_runtime_apply(base, len);
	if (IS_ERR(runtime_apply))
		return PTR_ERR(runtime_apply);
	access = coreboot_cfr_child_option_access(base, len);
	if (IS_ERR(access))
		return PTR_ERR(access);

	return runtime_apply && access ? 0 : -EINVAL;
}

int coreboot_cfr_kunit_parse_string(const void *base, size_t len)
{
	const struct lb_cfr_varbinary *string;
	char *copy;

	string = coreboot_cfr_child_string(base, len, CFR_TAG_VARCHAR_OPT_NAME);
	if (IS_ERR(string))
		return PTR_ERR(string);
	if (!string)
		return -EINVAL;

	copy = coreboot_cfr_string_dup(string);
	if (!copy)
		return -ENOMEM;
	kfree(copy);
	return 0;
}
#endif

static struct coreboot_driver coreboot_cfr_driver = {
	.probe = coreboot_cfr_probe,
	.remove = coreboot_cfr_remove,
	.drv = {
		.name = DRIVER_NAME,
	},
	.id_table = coreboot_cfr_ids,
};

static int __init coreboot_cfr_init(void)
{
	return coreboot_driver_register(&coreboot_cfr_driver);
}

static void __exit coreboot_cfr_exit(void)
{
	coreboot_driver_unregister(&coreboot_cfr_driver);
}

module_init(coreboot_cfr_init);
module_exit(coreboot_cfr_exit);

MODULE_AUTHOR("Sean Rhodes <sean@starlabs.systems>");
MODULE_DESCRIPTION("coreboot CFR firmware attributes driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("EFIVAR");
