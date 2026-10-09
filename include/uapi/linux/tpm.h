/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_TPM_H
#define _UAPI_LINUX_TPM_H

#include <linux/ioctl.h>

/* Raw TPM files only: sticky, before any write; no argument. */
#define TPM_IOC_SET_NO_RETRY _IO(0xA1, 0x01)

#endif /* _UAPI_LINUX_TPM_H */
