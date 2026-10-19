/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __COREBOOT_CFR_KUNIT_H
#define __COREBOOT_CFR_KUNIT_H

#include <linux/types.h>

struct coreboot_cfr_kunit_dependency {
	u64 id;
	const u32 *values;
	unsigned int n_values;
};

struct coreboot_cfr_kunit_setting {
	u64 object_id;
	u32 min;
	u32 max;
	u32 step;
	bool expose;
	const struct coreboot_cfr_kunit_dependency *dependencies;
	unsigned int n_dependencies;
};

int coreboot_cfr_kunit_validate_dependencies(
	const struct coreboot_cfr_kunit_setting *settings, unsigned int n_settings);
int coreboot_cfr_kunit_prepare_dependencies(
	const struct coreboot_cfr_kunit_setting *settings, unsigned int n_settings);
int coreboot_cfr_kunit_parse_dependency_values(const void *base, size_t len);
int coreboot_cfr_kunit_parse_runtime_apply_and_access(const void *base, size_t len);
int coreboot_cfr_kunit_parse_string(const void *base, size_t len);

#endif /* __COREBOOT_CFR_KUNIT_H */
