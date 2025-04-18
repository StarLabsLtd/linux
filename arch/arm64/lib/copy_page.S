/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 ARM Ltd.
 */

#include <linux/linkage.h>
#include <linux/const.h>
#include <asm/assembler.h>
#include <asm/page.h>
#include <asm/cpufeature.h>
#include <asm/alternative.h>

/*
 * Copy a page from src to dest (both are page aligned)
 *
 * Parameters:
 *	x0 - dest
 *	x1 - src
 */
SYM_FUNC_START(__pi_copy_page)
#ifdef CONFIG_AS_HAS_MOPS
	.arch_extension mops
alternative_if_not ARM64_HAS_MOPS
	b	.Lno_mops
alternative_else_nop_endif

	mov	x2, #PAGE_SIZE
	cpypwn	[x0]!, [x1]!, x2!
	cpymwn	[x0]!, [x1]!, x2!
	cpyewn	[x0]!, [x1]!, x2!
	ret
.Lno_mops:
#endif
	ldp	x2, x3, [x1]
	ldp	x4, x5, [x1, #16]
	ldp	x6, x7, [x1, #32]
	ldp	x8, x9, [x1, #48]
	ldp	x10, x11, [x1, #64]
	ldp	x12, x13, [x1, #80]
	ldp	x14, x15, [x1, #96]
	ldp	x16, x17, [x1, #112]

	add	x0, x0, #256
	add	x1, x1, #128
1:
	tst	x0, #(PAGE_SIZE - 1)

	stnp	x2, x3, [x0, #-256]
	ldp	x2, x3, [x1]
	stnp	x4, x5, [x0, #16 - 256]
	ldp	x4, x5, [x1, #16]
	stnp	x6, x7, [x0, #32 - 256]
	ldp	x6, x7, [x1, #32]
	stnp	x8, x9, [x0, #48 - 256]
	ldp	x8, x9, [x1, #48]
	stnp	x10, x11, [x0, #64 - 256]
	ldp	x10, x11, [x1, #64]
	stnp	x12, x13, [x0, #80 - 256]
	ldp	x12, x13, [x1, #80]
	stnp	x14, x15, [x0, #96 - 256]
	ldp	x14, x15, [x1, #96]
	stnp	x16, x17, [x0, #112 - 256]
	ldp	x16, x17, [x1, #112]

	add	x0, x0, #128
	add	x1, x1, #128

	b.ne	1b

	stnp	x2, x3, [x0, #-256]
	stnp	x4, x5, [x0, #16 - 256]
	stnp	x6, x7, [x0, #32 - 256]
	stnp	x8, x9, [x0, #48 - 256]
	stnp	x10, x11, [x0, #64 - 256]
	stnp	x12, x13, [x0, #80 - 256]
	stnp	x14, x15, [x0, #96 - 256]
	stnp	x16, x17, [x0, #112 - 256]

	ret
SYM_FUNC_END(__pi_copy_page)
SYM_FUNC_ALIAS(copy_page, __pi_copy_page)
EXPORT_SYMBOL(copy_page)
