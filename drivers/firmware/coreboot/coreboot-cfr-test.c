// SPDX-License-Identifier: GPL-2.0-only

#include <kunit/test.h>

#include "coreboot-cfr-kunit.h"

static void coreboot_cfr_test_duplicate_id(struct kunit *test)
{
	const struct coreboot_cfr_kunit_setting settings[] = {
		{ .object_id = 1, .max = 2, .step = 1 },
		{ .object_id = 1, .max = 2, .step = 1 },
	};

	KUNIT_EXPECT_EQ(test, coreboot_cfr_kunit_validate_dependencies(settings,
								ARRAY_SIZE(settings)), -EINVAL);
}

static void coreboot_cfr_test_missing_dependency(struct kunit *test)
{
	const struct coreboot_cfr_kunit_dependency dependencies[] = {
		{ .id = 2 },
	};
	const struct coreboot_cfr_kunit_setting settings[] = {
		{
			.object_id = 1,
			.max = 2,
			.step = 1,
			.dependencies = dependencies,
			.n_dependencies = ARRAY_SIZE(dependencies),
		},
	};

	KUNIT_EXPECT_EQ(test, coreboot_cfr_kunit_validate_dependencies(settings,
								ARRAY_SIZE(settings)), -EINVAL);
}

static void coreboot_cfr_test_cyclic_dependency(struct kunit *test)
{
	const struct coreboot_cfr_kunit_dependency first_dependencies[] = {
		{ .id = 2 },
	};
	const struct coreboot_cfr_kunit_dependency second_dependencies[] = {
		{ .id = 1 },
	};
	const struct coreboot_cfr_kunit_setting settings[] = {
		{
			.object_id = 1,
			.max = 2,
			.step = 1,
			.dependencies = first_dependencies,
			.n_dependencies = ARRAY_SIZE(first_dependencies),
		}, {
			.object_id = 2,
			.max = 2,
			.step = 1,
			.dependencies = second_dependencies,
			.n_dependencies = ARRAY_SIZE(second_dependencies),
		},
	};

	KUNIT_EXPECT_EQ(test, coreboot_cfr_kunit_validate_dependencies(settings,
								ARRAY_SIZE(settings)), -ELOOP);
}

static void coreboot_cfr_test_invalid_dependency_value(struct kunit *test)
{
	const u32 values[] = { 3 };
	const struct coreboot_cfr_kunit_dependency dependencies[] = {
		{ .id = 1, .values = values, .n_values = ARRAY_SIZE(values) },
	};
	const struct coreboot_cfr_kunit_setting settings[] = {
		{ .object_id = 1, .max = 2, .step = 1 }, {
			.object_id = 2,
			.max = 2,
			.step = 1,
			.dependencies = dependencies,
			.n_dependencies = ARRAY_SIZE(dependencies),
		},
	};

	KUNIT_EXPECT_EQ(test, coreboot_cfr_kunit_validate_dependencies(settings,
								ARRAY_SIZE(settings)), -EINVAL);
}

static void coreboot_cfr_test_malformed_dependency_values(struct kunit *test)
{
	struct {
		u32 tag;
		u32 size;
		u32 data_length;
		u8 data[3];
	} __packed malformed = {
		.tag = 12,
		.size = sizeof(malformed),
		.data_length = sizeof(malformed.data),
	};

	KUNIT_EXPECT_EQ(test, coreboot_cfr_kunit_parse_dependency_values(&malformed,
								    sizeof(malformed)), -EINVAL);
}

static void coreboot_cfr_test_binary_dependency_values(struct kunit *test)
{
	struct {
		u32 tag;
		u32 size;
		u32 data_length;
		__le32 data[2];
	} __packed record = {
		.tag = 12,
		.size = sizeof(record),
		.data_length = sizeof(record.data),
		.data = { cpu_to_le32(1), cpu_to_le32(2) },
	};

	KUNIT_EXPECT_EQ(test,
			coreboot_cfr_kunit_parse_dependency_values(&record, sizeof(record)), 0);
}

static void coreboot_cfr_test_hidden_dependency_parent(struct kunit *test)
{
	const struct coreboot_cfr_kunit_dependency dependencies[] = {
		{ .id = 1 },
	};
	const struct coreboot_cfr_kunit_setting settings[] = {
		{ .object_id = 1, .max = 1, .step = 1 }, {
			.object_id = 2,
			.max = 1,
			.step = 1,
			.expose = true,
			.dependencies = dependencies,
			.n_dependencies = ARRAY_SIZE(dependencies),
		},
	};

	KUNIT_EXPECT_EQ(test, coreboot_cfr_kunit_prepare_dependencies(settings,
							    ARRAY_SIZE(settings)), 0);
}

static void coreboot_cfr_test_runtime_apply_and_access_coexist(struct kunit *test)
{
	struct {
		struct {
			u32 tag;
			u32 size;
			u32 method;
			u32 id;
		} __packed runtime_apply;
		struct {
			__le32 tag;
			__le32 size;
			__le32 version;
			__le32 token;
			__le32 permissions;
			__le32 reserved;
		} __packed access;
	} __packed records = {
		.runtime_apply = {
			.tag = 13,
			.size = sizeof(records.runtime_apply),
			.method = 1,
			.id = 0xe3,
		},
		.access = {
			.tag = cpu_to_le32(14),
			.size = cpu_to_le32(sizeof(records.access)),
			.version = cpu_to_le32(1),
			.token = cpu_to_le32(1),
			.permissions = cpu_to_le32(BIT(0) | BIT(1)),
		},
	};

	KUNIT_EXPECT_EQ(test,
			coreboot_cfr_kunit_parse_runtime_apply_and_access(&records,
								       sizeof(records)), 0);
}

static void coreboot_cfr_test_string_termination(struct kunit *test)
{
	struct {
		u32 tag;
		u32 size;
		u32 data_length;
		u8 data[4];
	} __packed record = {
		.tag = 7,
		.size = sizeof(record),
		.data_length = sizeof(record.data),
		.data = { 'n', 'a', 'm', '\0' },
	};

	KUNIT_EXPECT_EQ(test, coreboot_cfr_kunit_parse_string(&record, sizeof(record)), 0);
	record.data[3] = 'e';
	KUNIT_EXPECT_EQ(test, coreboot_cfr_kunit_parse_string(&record, sizeof(record)),
			-EINVAL);
	record.data[1] = '\0';
	record.data[3] = '\0';
	KUNIT_EXPECT_EQ(test, coreboot_cfr_kunit_parse_string(&record, sizeof(record)),
			-EINVAL);
	record.data_length = 0;
	KUNIT_EXPECT_EQ(test, coreboot_cfr_kunit_parse_string(&record, sizeof(record)),
			-EINVAL);
}

static struct kunit_case coreboot_cfr_test_cases[] = {
	KUNIT_CASE(coreboot_cfr_test_duplicate_id),
	KUNIT_CASE(coreboot_cfr_test_missing_dependency),
	KUNIT_CASE(coreboot_cfr_test_cyclic_dependency),
	KUNIT_CASE(coreboot_cfr_test_invalid_dependency_value),
	KUNIT_CASE(coreboot_cfr_test_malformed_dependency_values),
	KUNIT_CASE(coreboot_cfr_test_binary_dependency_values),
	KUNIT_CASE(coreboot_cfr_test_hidden_dependency_parent),
	KUNIT_CASE(coreboot_cfr_test_runtime_apply_and_access_coexist),
	KUNIT_CASE(coreboot_cfr_test_string_termination),
	{}
};

static struct kunit_suite coreboot_cfr_test_suite = {
	.name = "coreboot-cfr-dependencies",
	.test_cases = coreboot_cfr_test_cases,
};
kunit_test_suite(coreboot_cfr_test_suite);

MODULE_LICENSE("GPL");
