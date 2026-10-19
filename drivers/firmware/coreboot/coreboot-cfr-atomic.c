// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/overflow.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "coreboot-cfr-private.h"
#include "coreboot_table.h"

#if IS_ENABLED(CONFIG_X86)

#define LB_TAG_MEMORY			0x0001
#define LB_TAG_CFR_ROOT			0x0047
#define LB_TAG_CFR_SETTINGS		0x004b
#define LB_MEM_TABLE			16
#define CFR_SETTINGS_VERSION		1
#define CFR_SETTINGS_MAILBOX_SIZE	64
#define COREBOOT_CFR_APM_CNT_PORT	0xb2

enum cfr_settings_command {
	CFR_SETTINGS_CMD_GET = 1,
	CFR_SETTINGS_CMD_SET = 2,
};

enum cfr_settings_status {
	CFR_SETTINGS_STATUS_VALUE = 2,
	CFR_SETTINGS_STATUS_UNCHANGED = 3,
	CFR_SETTINGS_STATUS_APPLIED = 4,
	CFR_SETTINGS_STATUS_REBOOT_REQUIRED = 5,
	CFR_SETTINGS_STATUS_ROLLED_BACK = 6,
	CFR_SETTINGS_STATUS_INDETERMINATE = 7,
	CFR_SETTINGS_STATUS_INVALID_MAILBOX = 8,
	CFR_SETTINGS_STATUS_INVALID_COMMAND = 9,
	CFR_SETTINGS_STATUS_INVALID_TOKEN = 10,
	CFR_SETTINGS_STATUS_DENIED = 11,
	CFR_SETTINGS_STATUS_CONFLICT = 12,
	CFR_SETTINGS_STATUS_INVALID_VALUE = 13,
	CFR_SETTINGS_STATUS_DEPENDENCY_FAILED = 14,
	CFR_SETTINGS_STATUS_STORAGE_ERROR = 15,
};

#define CFR_SETTINGS_RESP_FAULTED	BIT(0)
#define CFR_SETTINGS_RESP_CURRENT_VALID	BIT(1)
#define CFR_SETTINGS_RESP_FLAGS_MASK \
	(CFR_SETTINGS_RESP_FAULTED | CFR_SETTINGS_RESP_CURRENT_VALID)

struct lb_cfr_settings {
	__le32 tag;
	__le32 size;
	__le32 version;
	__le64 mailbox_address;
	__le32 mailbox_size;
	u8 apm_cmd;
	u8 reserved[3];
} __packed;

struct coreboot_cfr_lb_memory_range {
	__le64 start;
	__le64 size;
	__le32 type;
} __packed;

struct coreboot_cfr_lb_memory {
	__le32 tag;
	__le32 size;
	struct coreboot_cfr_lb_memory_range ranges[];
} __packed;

struct cfr_settings_mailbox {
	__le32 version;
	__le32 size;
	__le64 sequence;
	__le32 command;
	__le32 status;
	__le32 token;
	__le32 expected_value;
	__le32 value;
	__le32 current_value;
	__le32 response_flags;
	__le32 reserved[5];
} __packed;

static_assert(sizeof(struct cfr_settings_mailbox) == CFR_SETTINGS_MAILBOX_SIZE);

struct coreboot_cfr_atomic_service {
	struct coreboot_device *dev;
	void *mailbox;
	u8 apm_cmd;
	u64 sequence;
	int error;
};

static DEFINE_MUTEX(coreboot_cfr_atomic_service_lock);
static struct coreboot_cfr_atomic_service coreboot_cfr_atomic_service;

struct coreboot_cfr_mailbox_range {
	u64 address;
	u64 end;
	bool found;
	int error;
};

static u64 coreboot_cfr_atomic_initial_sequence(const void *mailbox)
{
	struct cfr_settings_mailbox response;

	memcpy(&response, mailbox, sizeof(response));
	if (get_unaligned_le32(&response.version) != CFR_SETTINGS_VERSION ||
	    get_unaligned_le32(&response.size) != sizeof(response) ||
	    memchr_inv(response.reserved, 0, sizeof(response.reserved)))
		return 0;

	return get_unaligned_le64(&response.sequence);
}

static int coreboot_cfr_atomic_status_errno(u32 status)
{
	switch (status) {
	case CFR_SETTINGS_STATUS_INVALID_MAILBOX:
	case CFR_SETTINGS_STATUS_INVALID_COMMAND:
		return -EPROTO;
	case CFR_SETTINGS_STATUS_INVALID_TOKEN:
		return -EOPNOTSUPP;
	case CFR_SETTINGS_STATUS_DENIED:
		return -EACCES;
	case CFR_SETTINGS_STATUS_CONFLICT:
	case CFR_SETTINGS_STATUS_DEPENDENCY_FAILED:
		return -EAGAIN;
	case CFR_SETTINGS_STATUS_INVALID_VALUE:
		return -EINVAL;
	case CFR_SETTINGS_STATUS_STORAGE_ERROR:
	default:
		return -EIO;
	}
}

static int coreboot_cfr_atomic_execute(u32 command, u32 token,
				       u32 expected_value, u32 value,
				       struct cfr_settings_mailbox *response)
{
	struct coreboot_cfr_atomic_service *service = &coreboot_cfr_atomic_service;
	struct cfr_settings_mailbox request = {
		.version = cpu_to_le32(CFR_SETTINGS_VERSION),
		.size = cpu_to_le32(sizeof(request)),
		.command = cpu_to_le32(command),
		.token = cpu_to_le32(token),
		.expected_value = cpu_to_le32(expected_value),
		.value = cpu_to_le32(value),
	};
	int ret = 0;

	mutex_lock(&coreboot_cfr_atomic_service_lock);
	if (!service->mailbox) {
		ret = -ENODEV;
		goto out_unlock;
	}

	if (!++service->sequence)
		service->sequence++;
	request.sequence = cpu_to_le64(service->sequence);
	memcpy(service->mailbox, &request, sizeof(request));
	/* Publish every request field before the SMI reads the mailbox. */
	wmb();
	outb(service->apm_cmd, COREBOOT_CFR_APM_CNT_PORT);
	/* Firmware has completed the command before its response is consumed. */
	mb();
	memcpy(response, service->mailbox, sizeof(*response));

	if (get_unaligned_le32(&response->version) != CFR_SETTINGS_VERSION ||
	    get_unaligned_le32(&response->size) != sizeof(*response) ||
	    get_unaligned_le64(&response->sequence) != service->sequence ||
	    get_unaligned_le32(&response->command) != command ||
	    get_unaligned_le32(&response->token) != token ||
	    get_unaligned_le32(&response->expected_value) != expected_value ||
	    get_unaligned_le32(&response->value) != value ||
	    get_unaligned_le32(&response->response_flags) &
	    ~CFR_SETTINGS_RESP_FLAGS_MASK ||
	    memchr_inv(response->reserved, 0, sizeof(response->reserved)))
		ret = -EPROTO;
out_unlock:
	mutex_unlock(&coreboot_cfr_atomic_service_lock);
	return ret;
}

static int coreboot_cfr_atomic_read(struct coreboot_cfr_setting *setting, u32 *value,
				    struct coreboot_cfr_read_result *result)
{
	struct cfr_settings_mailbox response;
	u32 token = coreboot_cfr_setting_access_token(setting);
	u32 response_flags;
	u32 status;
	int ret;

	if (!token)
		return -EOPNOTSUPP;

	ret = coreboot_cfr_atomic_execute(CFR_SETTINGS_CMD_GET, token, 0, 0,
					 &response);
	if (ret)
		return ret;
	response_flags = get_unaligned_le32(&response.response_flags);
	status = get_unaligned_le32(&response.status);
	if (response_flags & CFR_SETTINGS_RESP_FAULTED)
		result->faulted = true;

	if (status == CFR_SETTINGS_STATUS_INDETERMINATE)
		return -EIO;
	if (status != CFR_SETTINGS_STATUS_VALUE ||
	    !(response_flags & CFR_SETTINGS_RESP_CURRENT_VALID))
		return coreboot_cfr_atomic_status_errno(status);

	*value = get_unaligned_le32(&response.current_value);
	return 0;
}

static int coreboot_cfr_atomic_write(struct coreboot_cfr_setting *setting, u32 value,
					     struct coreboot_cfr_write_result *result)
{
	struct cfr_settings_mailbox response;
	struct coreboot_cfr_read_result read_result = { };
	u32 token = coreboot_cfr_setting_access_token(setting);
	u32 current_value;
	u32 response_flags;
	u32 status;
	int ret;

	if (!token)
		return -EOPNOTSUPP;

	ret = coreboot_cfr_atomic_read(setting, &current_value, &read_result);
	if (read_result.faulted) {
		result->outcome = COREBOOT_CFR_ATOMIC_INDETERMINATE;
		result->status = ret ?: -EIO;
		return 0;
	}
	if (ret)
		return ret;

	ret = coreboot_cfr_atomic_execute(CFR_SETTINGS_CMD_SET, token, current_value, value,
					 &response);
	if (ret)
		return ret;

	response_flags = get_unaligned_le32(&response.response_flags);
	status = get_unaligned_le32(&response.status);
	if (response_flags & CFR_SETTINGS_RESP_FAULTED ||
	    status == CFR_SETTINGS_STATUS_INDETERMINATE) {
		result->outcome = COREBOOT_CFR_ATOMIC_INDETERMINATE;
		result->status = -EIO;
		return 0;
	}

	switch (status) {
	case CFR_SETTINGS_STATUS_UNCHANGED:
		result->outcome = COREBOOT_CFR_ATOMIC_UNCHANGED;
		return 0;
	case CFR_SETTINGS_STATUS_APPLIED:
		result->outcome = COREBOOT_CFR_ATOMIC_APPLIED;
		return 0;
	case CFR_SETTINGS_STATUS_REBOOT_REQUIRED:
		result->outcome = COREBOOT_CFR_ATOMIC_REBOOT_REQUIRED;
		return 0;
	case CFR_SETTINGS_STATUS_ROLLED_BACK:
		result->outcome = COREBOOT_CFR_ATOMIC_ROLLED_BACK;
		result->status = -EIO;
		return 0;
	default:
		result->outcome = COREBOOT_CFR_ATOMIC_REJECTED;
		result->status = coreboot_cfr_atomic_status_errno(status);
		return 0;
	}
}

static bool coreboot_cfr_atomic_writes_supported(struct coreboot_cfr_drvdata *data)
{
	(void)data;
	return true;
}

static const struct coreboot_cfr_backend_ops coreboot_cfr_atomic_backend = {
	.read = coreboot_cfr_atomic_read,
	.write = coreboot_cfr_atomic_write,
	.writes_supported = coreboot_cfr_atomic_writes_supported,
	.atomic = true,
};

static int coreboot_cfr_atomic_find_service(struct device *dev, void *unused)
{
	struct coreboot_device *coreboot_dev = dev_to_coreboot_device(dev);

	(void)unused;

	return coreboot_dev->entry.tag == LB_TAG_CFR_SETTINGS;
}

const struct coreboot_cfr_backend_ops *
coreboot_cfr_atomic_backend_get(struct coreboot_device *dev)
{
	int error;

	mutex_lock(&coreboot_cfr_atomic_service_lock);
	if (coreboot_cfr_atomic_service.mailbox) {
		mutex_unlock(&coreboot_cfr_atomic_service_lock);
		return &coreboot_cfr_atomic_backend;
	}
	error = coreboot_cfr_atomic_service.error;
	mutex_unlock(&coreboot_cfr_atomic_service_lock);
	if (error)
		return ERR_PTR(error);

	if (device_for_each_child(dev->dev.parent, NULL,
				  coreboot_cfr_atomic_find_service))
		return ERR_PTR(-EPROBE_DEFER);

	return NULL;
}

static int coreboot_cfr_atomic_find_mailbox_range(struct device *dev, void *data)
{
	struct coreboot_cfr_mailbox_range *mailbox = data;
	struct coreboot_device *coreboot_dev = dev_to_coreboot_device(dev);
	struct coreboot_cfr_lb_memory *memory;
	struct coreboot_cfr_lb_memory_range *range;
	u64 range_start;
	u64 range_size;
	u64 range_end;
	u32 entry_size;
	unsigned int i;
	unsigned int n_ranges;

	if (coreboot_dev->entry.tag != LB_TAG_MEMORY)
		return 0;

	memory = (struct coreboot_cfr_lb_memory *)coreboot_dev->raw;
	entry_size = get_unaligned_le32(&memory->size);
	if (entry_size < sizeof(*memory) || entry_size > coreboot_dev->entry.size ||
	    (entry_size - sizeof(*memory)) % sizeof(*range)) {
		mailbox->error = -EINVAL;
		return 1;
	}

	n_ranges = (entry_size - sizeof(*memory)) / sizeof(*range);
	for (i = 0; i < n_ranges; i++) {
		range = &memory->ranges[i];
		if (get_unaligned_le32(&range->type) != LB_MEM_TABLE)
			continue;

		range_start = get_unaligned_le64(&range->start);
		range_size = get_unaligned_le64(&range->size);
		if (!range_size ||
		    check_add_overflow(range_start, range_size - 1, &range_end)) {
			mailbox->error = -ERANGE;
			return 1;
		}

		if (mailbox->address >= range_start && mailbox->end <= range_end) {
			mailbox->found = true;
			return 1;
		}
	}

	return 0;
}

static int cfr_atomic_check_mailbox_range(struct coreboot_device *dev,
					  u64 address, u64 end)
{
	struct coreboot_cfr_mailbox_range mailbox = {
		.address = address,
		.end = end,
	};

	device_for_each_child(dev->dev.parent, &mailbox,
			      coreboot_cfr_atomic_find_mailbox_range);
	if (mailbox.error)
		return mailbox.error;

	return mailbox.found ? 0 : -ERANGE;
}

static int coreboot_cfr_atomic_validate_service(struct coreboot_device *dev,
					 struct lb_cfr_settings *settings,
					 phys_addr_t *mailbox_address)
{
	u64 address;
	u64 end;
	int ret;

	if (dev->entry.size < sizeof(*settings) ||
	    get_unaligned_le32(&settings->size) < sizeof(*settings) ||
	    get_unaligned_le32(&settings->size) > dev->entry.size ||
	    get_unaligned_le32(&settings->tag) != LB_TAG_CFR_SETTINGS ||
	    get_unaligned_le32(&settings->version) != CFR_SETTINGS_VERSION ||
	    get_unaligned_le32(&settings->mailbox_size) != CFR_SETTINGS_MAILBOX_SIZE ||
	    !settings->apm_cmd ||
	    memchr_inv(settings->reserved, 0, sizeof(settings->reserved)))
		return -EINVAL;

	address = get_unaligned_le64(&settings->mailbox_address);
	if (!address || check_add_overflow(address,
					  (u64)get_unaligned_le32(&settings->mailbox_size) - 1,
					  &end) ||
	    address > PHYS_ADDR_MAX || end > PHYS_ADDR_MAX)
		return -ERANGE;
	ret = cfr_atomic_check_mailbox_range(dev, address, end);
	if (ret)
		return ret;

	*mailbox_address = address;
	return 0;
}

static int coreboot_cfr_atomic_reprobe_root(struct device *dev, void *unused)
{
	struct coreboot_device *coreboot_dev = dev_to_coreboot_device(dev);
	int ret;

	(void)unused;

	if (coreboot_dev->entry.tag != LB_TAG_CFR_ROOT)
		return 0;

	/* One root's probe result must not stop sibling discovery. */
	ret = device_reprobe(dev);
	if (ret)
		dev_dbg(dev, "CFR root reprobe failed: %d\n", ret);

	return 0;
}

static int coreboot_cfr_atomic_detach_root(struct device *dev, void *unused)
{
	struct coreboot_device *coreboot_dev = dev_to_coreboot_device(dev);

	(void)unused;

	if (coreboot_dev->entry.tag == LB_TAG_CFR_ROOT && dev->driver)
		device_release_driver(dev);

	return 0;
}

int coreboot_cfr_atomic_service_probe(struct coreboot_device *dev)
{
	struct lb_cfr_settings *settings = (struct lb_cfr_settings *)dev->raw;
	phys_addr_t mailbox_address;
	void *mailbox;
	int ret;

	ret = coreboot_cfr_atomic_validate_service(dev, settings, &mailbox_address);
	if (ret) {
		mutex_lock(&coreboot_cfr_atomic_service_lock);
		coreboot_cfr_atomic_service.error = ret;
		mutex_unlock(&coreboot_cfr_atomic_service_lock);
		device_for_each_child(dev->dev.parent, NULL,
				      coreboot_cfr_atomic_reprobe_root);
		return ret;
	}

	mailbox = memremap(mailbox_address, get_unaligned_le32(&settings->mailbox_size),
			    MEMREMAP_WB);
	if (!mailbox) {
		mutex_lock(&coreboot_cfr_atomic_service_lock);
		coreboot_cfr_atomic_service.error = -ENOMEM;
		mutex_unlock(&coreboot_cfr_atomic_service_lock);
		device_for_each_child(dev->dev.parent, NULL,
				      coreboot_cfr_atomic_reprobe_root);
		return -ENOMEM;
	}

	mutex_lock(&coreboot_cfr_atomic_service_lock);
	if (coreboot_cfr_atomic_service.dev) {
		ret = -EBUSY;
		goto out_unmap;
	}
	coreboot_cfr_atomic_service.dev = dev;
	coreboot_cfr_atomic_service.mailbox = mailbox;
	coreboot_cfr_atomic_service.apm_cmd = settings->apm_cmd;
	coreboot_cfr_atomic_service.sequence =
		coreboot_cfr_atomic_initial_sequence(mailbox);
	coreboot_cfr_atomic_service.error = 0;
	mutex_unlock(&coreboot_cfr_atomic_service_lock);

	device_for_each_child(dev->dev.parent, NULL, coreboot_cfr_atomic_reprobe_root);
	return 0;

out_unmap:
	mutex_unlock(&coreboot_cfr_atomic_service_lock);
	memunmap(mailbox);
	return ret;
}

void coreboot_cfr_atomic_service_remove(struct coreboot_device *dev)
{
	void *mailbox = NULL;

	device_for_each_child(dev->dev.parent, NULL, coreboot_cfr_atomic_detach_root);

	mutex_lock(&coreboot_cfr_atomic_service_lock);
	if (coreboot_cfr_atomic_service.dev == dev) {
		mailbox = coreboot_cfr_atomic_service.mailbox;
		coreboot_cfr_atomic_service.dev = NULL;
		coreboot_cfr_atomic_service.mailbox = NULL;
		coreboot_cfr_atomic_service.apm_cmd = 0;
		coreboot_cfr_atomic_service.sequence = 0;
		coreboot_cfr_atomic_service.error = -ENODEV;
	}
	mutex_unlock(&coreboot_cfr_atomic_service_lock);

	if (mailbox)
		memunmap(mailbox);
}

#else

const struct coreboot_cfr_backend_ops *
coreboot_cfr_atomic_backend_get(struct coreboot_device *dev)
{
	(void)dev;
	return NULL;
}

int coreboot_cfr_atomic_service_probe(struct coreboot_device *dev)
{
	(void)dev;
	return -ENODEV;
}

void coreboot_cfr_atomic_service_remove(struct coreboot_device *dev)
{
	(void)dev;
}

#endif
