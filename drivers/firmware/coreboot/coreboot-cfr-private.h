/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __COREBOOT_CFR_PRIVATE_H
#define __COREBOOT_CFR_PRIVATE_H

#include <linux/types.h>

struct coreboot_cfr_drvdata;
struct coreboot_cfr_setting;
struct coreboot_device;

/* The coreboot settings-service transport is private to the CFR driver. */
enum coreboot_cfr_atomic_outcome {
	COREBOOT_CFR_ATOMIC_UNCHANGED,
	COREBOOT_CFR_ATOMIC_APPLIED,
	COREBOOT_CFR_ATOMIC_REBOOT_REQUIRED,
	COREBOOT_CFR_ATOMIC_ROLLED_BACK,
	COREBOOT_CFR_ATOMIC_REJECTED,
	COREBOOT_CFR_ATOMIC_INDETERMINATE,
};

/*
 * An atomic backend returns zero once firmware has reported a final outcome.
 * status carries a negative errno for a rejected or rolled-back transaction.
 */
struct coreboot_cfr_write_result {
	enum coreboot_cfr_atomic_outcome outcome;
	int status;
};

struct coreboot_cfr_read_result {
	bool faulted;
};

struct coreboot_cfr_backend_ops {
	int (*read)(struct coreboot_cfr_setting *setting, u32 *value,
		    struct coreboot_cfr_read_result *result);
	int (*write)(struct coreboot_cfr_setting *setting, u32 value,
		     struct coreboot_cfr_write_result *result);
	bool (*writes_supported)(struct coreboot_cfr_drvdata *data);
	bool atomic;
};

const struct coreboot_cfr_backend_ops *
coreboot_cfr_atomic_backend_get(struct coreboot_device *dev);
int coreboot_cfr_atomic_service_probe(struct coreboot_device *dev);
void coreboot_cfr_atomic_service_remove(struct coreboot_device *dev);
u32 coreboot_cfr_setting_access_token(const struct coreboot_cfr_setting *setting);
bool coreboot_cfr_setting_access_writable(const struct coreboot_cfr_setting *setting);

#endif /* __COREBOOT_CFR_PRIVATE_H */
