/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _LINUX_I2C_HID_H
#define _LINUX_I2C_HID_H

#include <linux/errno.h>
#include <linux/kconfig.h>

struct hid_device;

/*
 * Caller pins and locks the HID, client and controller, holds client/controller
 * PM usage references and excludes system sleep. End only a successful begin,
 * in the same task before dropping those protections. Success retains both
 * transport mutexes; do not nest access or re-enter native HID commands.
 * Reset-history -EBUSY persists until the transport is reprobed.
 */
#if IS_REACHABLE(CONFIG_I2C_HID_CORE)
int i2c_hid_begin_external_access(struct hid_device *hid);
void i2c_hid_end_external_access(struct hid_device *hid);
#else
static inline int i2c_hid_begin_external_access(struct hid_device *hid)
{
	return -EOPNOTSUPP;
}

static inline void i2c_hid_end_external_access(struct hid_device *hid)
{
}
#endif

#endif /* _LINUX_I2C_HID_H */
