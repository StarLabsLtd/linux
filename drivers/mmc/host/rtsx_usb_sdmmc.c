// SPDX-License-Identifier: GPL-2.0-only
/* Realtek USB SD/MMC Card Interface driver
 *
 * Copyright(c) 2009-2013 Realtek Semiconductor Corp. All rights reserved.
 *
 * Author:
 *   Roger Tseng <rogerable@realtek.com>
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/fs.h>
#include <linux/bitops.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sd.h>
#include <linux/mmc/card.h>
#include <linux/scatterlist.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/vmalloc.h>

#include <linux/rtsx_usb.h>
#include <linux/unaligned.h>

#if defined(CONFIG_LEDS_CLASS) || (defined(CONFIG_LEDS_CLASS_MODULE) && \
		defined(CONFIG_MMC_REALTEK_USB_MODULE))
#include <linux/leds.h>
#define RTSX_USB_USE_LEDS_CLASS
#endif

#define RTSX_USB_SD_POLL_INTERVAL	msecs_to_jiffies(50)
#define RTSX_USB_SD_IDLE_POLL_INTERVAL	msecs_to_jiffies(1000)
#define RTSX_USB_SD_IDLE_WAIT_MAX	10
#define RTSX_USB_SD_SEQ_WAIT_MAX	2
#define RTSX_USB_MMC_CMD_RETRIES	3

#define RTSX_USB_IOC_MAGIC		0x39

struct rtsx_usb_ioc_sd_direct {
	u8 cmnd[12];
	__u64 buf;
	__s32 buf_len;
} __packed;

struct rtsx_usb_ioc_sd_rsp {
	__u64 rsp;
	__s32 rsp_len;
} __packed;

#define RTSX_USB_IOC_SD_DIRECT	_IOWR(RTSX_USB_IOC_MAGIC, 0xA0, \
				struct rtsx_usb_ioc_sd_direct)
#define RTSX_USB_IOC_SD_GET_RSP	_IOWR(RTSX_USB_IOC_MAGIC, 0xA1, \
				struct rtsx_usb_ioc_sd_rsp)

#ifdef CONFIG_COMPAT
struct rtsx_usb_ioc_sd_direct32 {
	u8 cmnd[12];
	compat_uptr_t buf;
	__s32 buf_len;
} __packed;

struct rtsx_usb_ioc_sd_rsp32 {
	compat_uptr_t rsp;
	__s32 rsp_len;
} __packed;
#define RTSX_USB_IOC_SD_DIRECT32	_IOWR(RTSX_USB_IOC_MAGIC, 0xA0, \
				struct rtsx_usb_ioc_sd_direct32)
#define RTSX_USB_IOC_SD_GET_RSP32	_IOWR(RTSX_USB_IOC_MAGIC, 0xA1, \
				struct rtsx_usb_ioc_sd_rsp32)
#endif

struct rtsx_usb_sdmmc {
	struct platform_device	*pdev;
	struct rtsx_ucr	*ucr;
	struct mmc_host		*mmc;
	struct mmc_request	*mrq;

	struct mutex		host_mutex;
	struct delayed_work	card_poll;
	unsigned long		poll_interval;
	unsigned int		idle_counter;
	unsigned int		idle_wait_max;
	bool			idle;
	bool			seq_mode;
	bool			seq_read;
	unsigned int		seq_counter;
	unsigned int		seq_wait_max;

	u8			ssc_depth;
	unsigned int		clock;
	bool			vpclk;
	bool			double_clk;
	bool			host_removal;
	bool			card_exist;
	bool			initial_mode;
	bool			ddr_mode;

	unsigned char		power_mode;
	u16			ocp_stat;
	struct mutex		cprm_lock;
	u8			cprm_rsp[16];
	size_t			cprm_rsp_len;
	bool			cprm_rsp_valid;
	u8			cprm_rsp_type;
	struct miscdevice	cprm_miscdev;
	char			*cprm_name;
	bool			cprm_registered;
#ifdef RTSX_USB_USE_LEDS_CLASS
	struct led_classdev	led;
	char			led_name[32];
	struct work_struct	led_work;
#endif
};

static inline struct device *sdmmc_dev(struct rtsx_usb_sdmmc *host)
{
	return &(host->pdev->dev);
}

static inline void sd_clear_error(struct rtsx_usb_sdmmc *host)
{
	struct rtsx_ucr *ucr = host->ucr;
	rtsx_usb_ep0_write_register(ucr, CARD_STOP,
				  SD_STOP | SD_CLR_ERR,
				  SD_STOP | SD_CLR_ERR);

	rtsx_usb_clear_dma_err(ucr);
	rtsx_usb_clear_fsm_err(ucr);
}

static void sdmmc_stop_seq_mode(struct rtsx_usb_sdmmc *host)
{
	struct mmc_command stop = { };

	if (!host->seq_mode)
		return;

	stop.opcode = MMC_STOP_TRANSMISSION;
	stop.arg = 0;
	stop.flags = MMC_RSP_R1B | MMC_CMD_AC;

	sd_send_cmd_get_rsp(host, &stop);
	if (stop.error)
		dev_dbg(sdmmc_dev(host), "stop transmission err %d\n",
			stop.error);

	rtsx_usb_write_register(host->ucr, MC_FIFO_CTL,
				      FIFO_FLUSH, FIFO_FLUSH);
	host->seq_mode = false;
	host->seq_read = false;
	host->seq_counter = 0;
}

static void sdmmc_enter_idle(struct rtsx_usb_sdmmc *host)
{
	struct rtsx_ucr *ucr = host->ucr;
	int err;

	if (host->idle)
		return;

	err = rtsx_usb_write_register(ucr, CLK_DIV, CLK_CHANGE, CLK_CHANGE);
	if (err)
		goto out_dbg;

	err = rtsx_usb_write_register(ucr, FPDCTL,
					      SSC_POWER_MASK, SSC_POWER_DOWN);
	if (!err)
		host->idle = true;

out_dbg:
	if (err)
		dev_dbg(sdmmc_dev(host), "enter idle failed %d\n", err);
	else
		host->idle_counter = host->idle_wait_max;
}

static void sdmmc_leave_idle(struct rtsx_usb_sdmmc *host)
{
	struct rtsx_ucr *ucr = host->ucr;
	int err;

	if (!host->idle)
		return;

	err = rtsx_usb_write_register(ucr, FPDCTL,
					      SSC_POWER_MASK, SSC_POWER_ON);
	if (err)
		goto out_dbg;

	usleep_range(100, 150);
	err = rtsx_usb_write_register(ucr, CLK_DIV, CLK_CHANGE, 0);
	if (!err)
		host->idle = false;

out_dbg:
	if (err)
		dev_dbg(sdmmc_dev(host), "leave idle failed %d\n", err);
	else
		host->idle_counter = 0;
}

#ifdef DEBUG
static void sd_print_debug_regs(struct rtsx_usb_sdmmc *host)
{
	struct rtsx_ucr *ucr = host->ucr;
	u8 val = 0;

	rtsx_usb_ep0_read_register(ucr, SD_STAT1, &val);
	dev_dbg(sdmmc_dev(host), "SD_STAT1: 0x%x\n", val);
	rtsx_usb_ep0_read_register(ucr, SD_STAT2, &val);
	dev_dbg(sdmmc_dev(host), "SD_STAT2: 0x%x\n", val);
	rtsx_usb_ep0_read_register(ucr, SD_BUS_STAT, &val);
	dev_dbg(sdmmc_dev(host), "SD_BUS_STAT: 0x%x\n", val);
}
#else
#define sd_print_debug_regs(host)
#endif /* DEBUG */

static int sd_read_data(struct rtsx_usb_sdmmc *host, struct mmc_command *cmd,
	       u16 byte_cnt, u8 *buf, int buf_len, int timeout)
{
	struct rtsx_ucr *ucr = host->ucr;
	int err;
	u8 trans_mode;

	if (!buf)
		buf_len = 0;

	rtsx_usb_init_cmd(ucr);
	if (cmd != NULL) {
		dev_dbg(sdmmc_dev(host), "%s: SD/MMC CMD%d\n", __func__
				, cmd->opcode);
		if (cmd->opcode == MMC_SEND_TUNING_BLOCK)
			trans_mode = SD_TM_AUTO_TUNING;
		else
			trans_mode = SD_TM_NORMAL_READ;

		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_CMD0, 0xFF, (u8)(cmd->opcode) | 0x40);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_CMD1, 0xFF, (u8)(cmd->arg >> 24));
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_CMD2, 0xFF, (u8)(cmd->arg >> 16));
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_CMD3, 0xFF, (u8)(cmd->arg >> 8));
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_CMD4, 0xFF, (u8)cmd->arg);
	} else {
		trans_mode = SD_TM_AUTO_READ_3;
	}

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BYTE_CNT_L, 0xFF, (u8)byte_cnt);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BYTE_CNT_H,
			0xFF, (u8)(byte_cnt >> 8));
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BLOCK_CNT_L, 0xFF, 1);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BLOCK_CNT_H, 0xFF, 0);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CFG2, 0xFF,
			SD_CALCULATE_CRC7 | SD_CHECK_CRC16 |
			SD_NO_WAIT_BUSY_END | SD_CHECK_CRC7 | SD_RSP_LEN_6);
	if (trans_mode != SD_TM_AUTO_TUNING)
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				CARD_DATA_SOURCE, 0x01, PINGPONG_BUFFER);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_TRANSFER,
			0xFF, trans_mode | SD_TRANSFER_START);
	rtsx_usb_add_cmd(ucr, CHECK_REG_CMD, SD_TRANSFER,
			SD_TRANSFER_END, SD_TRANSFER_END);

	if (cmd != NULL) {
		rtsx_usb_add_cmd(ucr, READ_REG_CMD, SD_CMD1, 0, 0);
		rtsx_usb_add_cmd(ucr, READ_REG_CMD, SD_CMD2, 0, 0);
		rtsx_usb_add_cmd(ucr, READ_REG_CMD, SD_CMD3, 0, 0);
		rtsx_usb_add_cmd(ucr, READ_REG_CMD, SD_CMD4, 0, 0);
	}

	err = rtsx_usb_send_cmd(ucr, MODE_CR, timeout);
	if (err) {
		dev_dbg(sdmmc_dev(host),
			"rtsx_usb_send_cmd failed (err = %d)\n", err);
		return err;
	}

	err = rtsx_usb_get_rsp(ucr, !cmd ? 1 : 5, timeout);
	if (err || (ucr->rsp_buf[0] & SD_TRANSFER_ERR)) {
		sd_print_debug_regs(host);

		if (!err) {
			dev_dbg(sdmmc_dev(host),
				"Transfer failed (SD_TRANSFER = %02x)\n",
				ucr->rsp_buf[0]);
			err = -EIO;
		} else {
			dev_dbg(sdmmc_dev(host),
				"rtsx_usb_get_rsp failed (err = %d)\n", err);
		}

		return err;
	}

	if (cmd != NULL) {
		cmd->resp[0] = get_unaligned_be32(ucr->rsp_buf + 1);
		dev_dbg(sdmmc_dev(host), "cmd->resp[0] = 0x%08x\n",
				cmd->resp[0]);
	}

	if (buf && buf_len) {
		/* 2-byte aligned part */
		err = rtsx_usb_read_ppbuf(ucr, buf, byte_cnt - (byte_cnt % 2));
		if (err) {
			dev_dbg(sdmmc_dev(host),
				"rtsx_usb_read_ppbuf failed (err = %d)\n", err);
			return err;
		}

		/* unaligned byte */
		if (byte_cnt % 2)
			return rtsx_usb_read_register(ucr,
					PPBUF_BASE2 + byte_cnt,
					buf + byte_cnt - 1);
	}

	return 0;
}

static int sd_write_data(struct rtsx_usb_sdmmc *host, struct mmc_command *cmd,
		u16 byte_cnt, u8 *buf, int buf_len, int timeout)
{
	struct rtsx_ucr *ucr = host->ucr;
	int err;
	u8 trans_mode;

	if (!buf)
		buf_len = 0;

	if (buf && buf_len) {
		err = rtsx_usb_write_ppbuf(ucr, buf, buf_len);
		if (err) {
			dev_dbg(sdmmc_dev(host),
				"rtsx_usb_write_ppbuf failed (err = %d)\n",
				err);
			return err;
		}
	}

	trans_mode = (cmd != NULL) ? SD_TM_AUTO_WRITE_2 : SD_TM_AUTO_WRITE_3;
	rtsx_usb_init_cmd(ucr);

	if (cmd != NULL) {
		dev_dbg(sdmmc_dev(host), "%s: SD/MMC CMD%d\n", __func__,
				cmd->opcode);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_CMD0, 0xFF, (u8)(cmd->opcode) | 0x40);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_CMD1, 0xFF, (u8)(cmd->arg >> 24));
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_CMD2, 0xFF, (u8)(cmd->arg >> 16));
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_CMD3, 0xFF, (u8)(cmd->arg >> 8));
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_CMD4, 0xFF, (u8)cmd->arg);
	}

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BYTE_CNT_L, 0xFF, (u8)byte_cnt);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BYTE_CNT_H,
			0xFF, (u8)(byte_cnt >> 8));
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BLOCK_CNT_L, 0xFF, 1);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BLOCK_CNT_H, 0xFF, 0);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CFG2, 0xFF,
		SD_CALCULATE_CRC7 | SD_CHECK_CRC16 |
		SD_NO_WAIT_BUSY_END | SD_CHECK_CRC7 | SD_RSP_LEN_6);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
			CARD_DATA_SOURCE, 0x01, PINGPONG_BUFFER);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_TRANSFER, 0xFF,
			trans_mode | SD_TRANSFER_START);
	rtsx_usb_add_cmd(ucr, CHECK_REG_CMD, SD_TRANSFER,
			SD_TRANSFER_END, SD_TRANSFER_END);

	if (cmd != NULL) {
		rtsx_usb_add_cmd(ucr, READ_REG_CMD, SD_CMD1, 0, 0);
		rtsx_usb_add_cmd(ucr, READ_REG_CMD, SD_CMD2, 0, 0);
		rtsx_usb_add_cmd(ucr, READ_REG_CMD, SD_CMD3, 0, 0);
		rtsx_usb_add_cmd(ucr, READ_REG_CMD, SD_CMD4, 0, 0);
	}

	err = rtsx_usb_send_cmd(ucr, MODE_CR, timeout);
	if (err) {
		dev_dbg(sdmmc_dev(host),
			"rtsx_usb_send_cmd failed (err = %d)\n", err);
		return err;
	}

	err = rtsx_usb_get_rsp(ucr, !cmd ? 1 : 5, timeout);
	if (err) {
		sd_print_debug_regs(host);
		dev_dbg(sdmmc_dev(host),
			"rtsx_usb_get_rsp failed (err = %d)\n", err);
		return err;
	}

	if (cmd != NULL) {
		cmd->resp[0] = get_unaligned_be32(ucr->rsp_buf + 1);
		dev_dbg(sdmmc_dev(host), "cmd->resp[0] = 0x%08x\n",
				cmd->resp[0]);
	}

	return 0;
}

static void sd_send_cmd_get_rsp(struct rtsx_usb_sdmmc *host,
		struct mmc_command *cmd)
{
	struct rtsx_ucr *ucr = host->ucr;
	u8 cmd_idx = (u8)cmd->opcode;
	u32 arg = cmd->arg;
	int err = 0;
	int timeout = 100;
	int i;
	u8 *ptr;
	int stat_idx = 0;
	int len = 2;
	u8 rsp_type;

	dev_dbg(sdmmc_dev(host), "%s: SD/MMC CMD %d, arg = 0x%08x\n",
			__func__, cmd_idx, arg);

	/* Response type:
	 * R0
	 * R1, R5, R6, R7
	 * R1b
	 * R2
	 * R3, R4
	 */
	switch (mmc_resp_type(cmd)) {
	case MMC_RSP_NONE:
		rsp_type = SD_RSP_TYPE_R0;
		break;
	case MMC_RSP_R1:
		rsp_type = SD_RSP_TYPE_R1;
		break;
	case MMC_RSP_R1B:
		rsp_type = SD_RSP_TYPE_R1b;
		break;
	case MMC_RSP_R2:
		rsp_type = SD_RSP_TYPE_R2;
		break;
	case MMC_RSP_R3:
		rsp_type = SD_RSP_TYPE_R3;
		break;
	default:
		dev_dbg(sdmmc_dev(host), "cmd->flag is not valid\n");
		err = -EINVAL;
		goto out;
	}

	if (rsp_type == SD_RSP_TYPE_R1b)
		timeout = cmd->busy_timeout ? cmd->busy_timeout : 3000;

	if (cmd->opcode == SD_SWITCH_VOLTAGE) {
		err = rtsx_usb_write_register(ucr, SD_BUS_STAT,
				SD_CLK_TOGGLE_EN | SD_CLK_FORCE_STOP,
				SD_CLK_TOGGLE_EN);
		if (err)
			goto out;
	}

	rtsx_usb_init_cmd(ucr);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CMD0, 0xFF, 0x40 | cmd_idx);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CMD1, 0xFF, (u8)(arg >> 24));
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CMD2, 0xFF, (u8)(arg >> 16));
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CMD3, 0xFF, (u8)(arg >> 8));
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CMD4, 0xFF, (u8)arg);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CFG2, 0xFF, rsp_type);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_DATA_SOURCE,
			0x01, PINGPONG_BUFFER);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_TRANSFER,
			0xFF, SD_TM_CMD_RSP | SD_TRANSFER_START);
	rtsx_usb_add_cmd(ucr, CHECK_REG_CMD, SD_TRANSFER,
		     SD_TRANSFER_END | SD_STAT_IDLE,
		     SD_TRANSFER_END | SD_STAT_IDLE);

	if (rsp_type == SD_RSP_TYPE_R2) {
		/* Read data from ping-pong buffer */
		for (i = PPBUF_BASE2; i < PPBUF_BASE2 + 16; i++)
			rtsx_usb_add_cmd(ucr, READ_REG_CMD, (u16)i, 0, 0);
		stat_idx = 16;
	} else if (rsp_type != SD_RSP_TYPE_R0) {
		/* Read data from SD_CMDx registers */
		for (i = SD_CMD0; i <= SD_CMD4; i++)
			rtsx_usb_add_cmd(ucr, READ_REG_CMD, (u16)i, 0, 0);
		stat_idx = 5;
	}
	len += stat_idx;

	rtsx_usb_add_cmd(ucr, READ_REG_CMD, SD_STAT1, 0, 0);

	err = rtsx_usb_send_cmd(ucr, MODE_CR, 100);
	if (err) {
		dev_dbg(sdmmc_dev(host),
			"rtsx_usb_send_cmd error (err = %d)\n", err);
		goto out;
	}

	err = rtsx_usb_get_rsp(ucr, len, timeout);
	if (err || (ucr->rsp_buf[0] & SD_TRANSFER_ERR)) {
		sd_print_debug_regs(host);
		sd_clear_error(host);

		if (!err) {
			dev_dbg(sdmmc_dev(host),
				"Transfer failed (SD_TRANSFER = %02x)\n",
					ucr->rsp_buf[0]);
			err = -EIO;
		} else {
			dev_dbg(sdmmc_dev(host),
				"rtsx_usb_get_rsp failed (err = %d)\n", err);
		}

		goto out;
	}

	if (rsp_type == SD_RSP_TYPE_R0) {
		err = 0;
		goto out;
	}

	/* Skip result of CHECK_REG_CMD */
	ptr = ucr->rsp_buf + 1;

	/* Check (Start,Transmission) bit of Response */
	if ((ptr[0] & 0xC0) != 0) {
		err = -EILSEQ;
		dev_dbg(sdmmc_dev(host), "Invalid response bit\n");
		goto out;
	}

	/* Check CRC7 */
	if (!(rsp_type & SD_NO_CHECK_CRC7)) {
		if (ptr[stat_idx] & SD_CRC7_ERR) {
			err = -EILSEQ;
			dev_dbg(sdmmc_dev(host), "CRC7 error\n");
			goto out;
		}
	}

	if (rsp_type == SD_RSP_TYPE_R2) {
		/*
		 * The controller offloads the last byte {CRC-7, end bit 1'b1}
		 * of response type R2. Assign dummy CRC, 0, and end bit to the
		 * byte(ptr[16], goes into the LSB of resp[3] later).
		 */
		ptr[16] = 1;

		for (i = 0; i < 4; i++) {
			cmd->resp[i] = get_unaligned_be32(ptr + 1 + i * 4);
			dev_dbg(sdmmc_dev(host), "cmd->resp[%d] = 0x%08x\n",
					i, cmd->resp[i]);
		}
	} else {
		cmd->resp[0] = get_unaligned_be32(ptr + 1);
		dev_dbg(sdmmc_dev(host), "cmd->resp[0] = 0x%08x\n",
				cmd->resp[0]);
	}

out:
	cmd->error = err;
}

static int sd_rw_multi(struct rtsx_usb_sdmmc *host, struct mmc_request *mrq)
{
	struct rtsx_ucr *ucr = host->ucr;
	struct mmc_data *data = mrq->data;
	int read = (data->flags & MMC_DATA_READ) ? 1 : 0;
	u8 cfg2, trans_mode;
	int err;
	u8 flag;
	size_t data_len = data->blksz * data->blocks;
	unsigned int pipe;

	if (read) {
		dev_dbg(sdmmc_dev(host), "%s: read %zu bytes\n",
				__func__, data_len);
		cfg2 = SD_CALCULATE_CRC7 | SD_CHECK_CRC16 |
			SD_NO_WAIT_BUSY_END | SD_CHECK_CRC7 | SD_RSP_LEN_0;
		trans_mode = SD_TM_AUTO_READ_3;
	} else {
		dev_dbg(sdmmc_dev(host), "%s: write %zu bytes\n",
				__func__, data_len);
		cfg2 = SD_NO_CALCULATE_CRC7 | SD_CHECK_CRC16 |
			SD_NO_WAIT_BUSY_END | SD_NO_CHECK_CRC7 | SD_RSP_LEN_0;
		trans_mode = SD_TM_AUTO_WRITE_3;
	}

	rtsx_usb_init_cmd(ucr);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BYTE_CNT_L, 0xFF, 0x00);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BYTE_CNT_H, 0xFF, 0x02);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BLOCK_CNT_L,
			0xFF, (u8)data->blocks);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BLOCK_CNT_H,
			0xFF, (u8)(data->blocks >> 8));

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_DATA_SOURCE,
			0x01, RING_BUFFER);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, MC_DMA_TC3,
			0xFF, (u8)(data_len >> 24));
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, MC_DMA_TC2,
			0xFF, (u8)(data_len >> 16));
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, MC_DMA_TC1,
			0xFF, (u8)(data_len >> 8));
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, MC_DMA_TC0,
			0xFF, (u8)data_len);
	if (read) {
		flag = MODE_CDIR;
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, MC_DMA_CTL,
				0x03 | DMA_PACK_SIZE_MASK,
				DMA_DIR_FROM_CARD | DMA_EN | DMA_512);
	} else {
		flag = MODE_CDOR;
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, MC_DMA_CTL,
				0x03 | DMA_PACK_SIZE_MASK,
				DMA_DIR_TO_CARD | DMA_EN | DMA_512);
	}

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CFG2, 0xFF, cfg2);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_TRANSFER, 0xFF,
			trans_mode | SD_TRANSFER_START);
	rtsx_usb_add_cmd(ucr, CHECK_REG_CMD, SD_TRANSFER,
			SD_TRANSFER_END, SD_TRANSFER_END);

	err = rtsx_usb_send_cmd(ucr, flag, 100);
	if (err) {
		host->seq_mode = false;
		host->seq_counter = 0;
		return err;
	}

	if (read)
		pipe = usb_rcvbulkpipe(ucr->pusb_dev, EP_BULK_IN);
	else
		pipe = usb_sndbulkpipe(ucr->pusb_dev, EP_BULK_OUT);

	err = rtsx_usb_transfer_data(ucr, pipe, data->sg, data_len,
			data->sg_len,  NULL, 10000);
	if (err) {
		dev_dbg(sdmmc_dev(host), "rtsx_usb_transfer_data error %d\n"
				, err);
		sd_clear_error(host);
		host->seq_mode = false;
		host->seq_counter = 0;
		return err;
	}

	err = rtsx_usb_get_rsp(ucr, 1, 2000);
	if (!err) {
		host->seq_mode = true;
		host->seq_read = read;
		host->seq_counter = 0;
	} else {
		dev_dbg(sdmmc_dev(host), "sd multi rsp err %d\n", err);
		host->seq_mode = false;
		host->seq_counter = 0;
	}

	return err;
}

static inline void sd_enable_initial_mode(struct rtsx_usb_sdmmc *host)
{
	rtsx_usb_write_register(host->ucr, SD_CFG1,
			SD_CLK_DIVIDE_MASK, SD_CLK_DIVIDE_128);
}

static inline void sd_disable_initial_mode(struct rtsx_usb_sdmmc *host)
{
	rtsx_usb_write_register(host->ucr, SD_CFG1,
			SD_CLK_DIVIDE_MASK, SD_CLK_DIVIDE_0);
}

static void sd_normal_rw(struct rtsx_usb_sdmmc *host,
		struct mmc_request *mrq)
{
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	u8 *buf;

	buf = kzalloc(data->blksz, GFP_NOIO);
	if (!buf) {
		cmd->error = -ENOMEM;
		return;
	}

	if (data->flags & MMC_DATA_READ) {
		if (host->initial_mode)
			sd_disable_initial_mode(host);

		cmd->error = sd_read_data(host, cmd, (u16)data->blksz, buf,
				data->blksz, 200);

		if (host->initial_mode)
			sd_enable_initial_mode(host);

		sg_copy_from_buffer(data->sg, data->sg_len, buf, data->blksz);
	} else {
		sg_copy_to_buffer(data->sg, data->sg_len, buf, data->blksz);

		cmd->error = sd_write_data(host, cmd, (u16)data->blksz, buf,
				data->blksz, 200);
	}

	kfree(buf);
}

static int sd_change_phase(struct rtsx_usb_sdmmc *host, u8 sample_point, int tx)
{
	struct rtsx_ucr *ucr = host->ucr;

	dev_dbg(sdmmc_dev(host), "%s: %s sample_point = %d\n",
			__func__, tx ? "TX" : "RX", sample_point);

	rtsx_usb_init_cmd(ucr);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CLK_DIV, CLK_CHANGE, CLK_CHANGE);

	if (tx)
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_VPCLK0_CTL,
				0x0F, sample_point);
	else
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_VPCLK1_CTL,
				0x0F, sample_point);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_VPCLK0_CTL, PHASE_NOT_RESET, 0);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_VPCLK0_CTL,
			PHASE_NOT_RESET, PHASE_NOT_RESET);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CLK_DIV, CLK_CHANGE, 0);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CFG1, SD_ASYNC_FIFO_RST, 0);

	return rtsx_usb_send_cmd(ucr, MODE_C, 100);
}

static inline u32 get_phase_point(u32 phase_map, unsigned int idx)
{
	idx &= MAX_PHASE;
	return phase_map & (1 << idx);
}

static int get_phase_len(u32 phase_map, unsigned int idx)
{
	int i;

	for (i = 0; i < MAX_PHASE + 1; i++) {
		if (get_phase_point(phase_map, idx + i) == 0)
			return i;
	}
	return MAX_PHASE + 1;
}

static u8 sd_search_final_phase(struct rtsx_usb_sdmmc *host, u32 phase_map)
{
	int start = 0, len = 0;
	int start_final = 0, len_final = 0;
	u8 final_phase = 0xFF;

	if (phase_map == 0) {
		dev_dbg(sdmmc_dev(host), "Phase: [map:%x]\n", phase_map);
		return final_phase;
	}

	while (start < MAX_PHASE + 1) {
		len = get_phase_len(phase_map, start);
		if (len_final < len) {
			start_final = start;
			len_final = len;
		}
		start += len ? len : 1;
	}

	final_phase = (start_final + len_final / 2) & MAX_PHASE;
	dev_dbg(sdmmc_dev(host), "Phase: [map:%x] [maxlen:%d] [final:%d]\n",
		phase_map, len_final, final_phase);

	return final_phase;
}

static void sd_wait_data_idle(struct rtsx_usb_sdmmc *host)
{
	int i;
	u8 val = 0;

	for (i = 0; i < 100; i++) {
		rtsx_usb_ep0_read_register(host->ucr, SD_DATA_STATE, &val);
		if (val & SD_DATA_IDLE)
			return;

		usleep_range(100, 1000);
	}
}

static int sd_tuning_rx_cmd(struct rtsx_usb_sdmmc *host,
		u8 opcode, u8 sample_point)
{
	int err;
	struct mmc_command cmd = {};

	err = sd_change_phase(host, sample_point, 0);
	if (err)
		return err;

	cmd.opcode = MMC_SEND_TUNING_BLOCK;
	err = sd_read_data(host, &cmd, 0x40, NULL, 0, 100);
	if (err) {
		/* Wait till SD DATA IDLE */
		sd_wait_data_idle(host);
		sd_clear_error(host);
		return err;
	}

	return 0;
}

static void sd_tuning_phase(struct rtsx_usb_sdmmc *host,
		u8 opcode, u16 *phase_map)
{
	int err, i;
	u16 raw_phase_map = 0;

	for (i = MAX_PHASE; i >= 0; i--) {
		err = sd_tuning_rx_cmd(host, opcode, (u8)i);
		if (!err)
			raw_phase_map |= 1 << i;
	}

	if (phase_map)
		*phase_map = raw_phase_map;
}

static int sd_tuning_rx(struct rtsx_usb_sdmmc *host, u8 opcode)
{
	int err, i;
	u16 raw_phase_map[RX_TUNING_CNT] = {0}, phase_map;
	u8 final_phase;

	/* setting fixed default TX phase */
	err = sd_change_phase(host, 0x01, 1);
	if (err) {
		dev_dbg(sdmmc_dev(host), "TX phase setting failed\n");
		return err;
	}

	/* tuning RX phase */
	for (i = 0; i < RX_TUNING_CNT; i++) {
		sd_tuning_phase(host, opcode, &(raw_phase_map[i]));

		if (raw_phase_map[i] == 0)
			break;
	}

	phase_map = 0xFFFF;
	for (i = 0; i < RX_TUNING_CNT; i++) {
		dev_dbg(sdmmc_dev(host), "RX raw_phase_map[%d] = 0x%04x\n",
				i, raw_phase_map[i]);
		phase_map &= raw_phase_map[i];
	}
	dev_dbg(sdmmc_dev(host), "RX phase_map = 0x%04x\n", phase_map);

	if (phase_map) {
		final_phase = sd_search_final_phase(host, phase_map);
		if (final_phase == 0xFF)
			return -EINVAL;

		err = sd_change_phase(host, final_phase, 0);
		if (err)
			return err;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int sdmmc_get_ro(struct mmc_host *mmc)
{
	struct rtsx_usb_sdmmc *host = mmc_priv(mmc);
	struct rtsx_ucr *ucr = host->ucr;
	int err;
	u16 val;

	if (host->host_removal)
		return -ENOMEDIUM;

	mutex_lock(&ucr->dev_mutex);

	/* Check SD card detect */
	err = rtsx_usb_get_card_status(ucr, &val);

	mutex_unlock(&ucr->dev_mutex);


	/* Treat failed detection as non-ro */
	if (err)
		return 0;

	if (val & SD_WP)
		return 1;

	return 0;
}

static int sdmmc_get_cd(struct mmc_host *mmc)
{
	struct rtsx_usb_sdmmc *host = mmc_priv(mmc);
	struct rtsx_ucr *ucr = host->ucr;
	int err;
	u16 val;

	if (host->host_removal)
		return -ENOMEDIUM;

	mutex_lock(&ucr->dev_mutex);

	/* Check SD card detect */
	err = rtsx_usb_get_card_status(ucr, &val);

	mutex_unlock(&ucr->dev_mutex);

	/* Treat failed detection as non-exist */
	if (err)
		goto no_card;

	/* get OCP status */
	host->ocp_stat = (val >> 4) & 0x03;

	if (val & SD_CD) {
		host->card_exist = true;
		return 1;
	}

no_card:
	/* clear OCP status */
	if (host->ocp_stat & (MS_OCP_NOW | MS_OCP_EVER)) {
		rtsx_usb_write_register(ucr, OCPCTL, MS_OCP_CLEAR, MS_OCP_CLEAR);
		host->ocp_stat = 0;
	}
	host->card_exist = false;
	return 0;
}

static void rtsx_usb_sdmmc_poll_card(struct work_struct *work)
{
	struct rtsx_usb_sdmmc *host = container_of(work,
			struct rtsx_usb_sdmmc, card_poll.work);
	struct rtsx_ucr *ucr = host->ucr;
	struct device *dev = sdmmc_dev(host);
	bool present = READ_ONCE(host->card_exist);
	bool changed = false;
	bool prev;
	int err;
	u16 status = 0;
	u8 pend;

	if (host->host_removal)
		return;

	err = pm_runtime_get_sync(dev);
	if (err < 0) {
		pm_runtime_put_noidle(dev);
		goto requeue;
	}

	mutex_lock(&ucr->dev_mutex);

	prev = READ_ONCE(host->card_exist);

	err = rtsx_usb_get_card_status(ucr, &status);
	if (!err) {
		host->ocp_stat = (status >> 4) & 0x03;
		present = !!(status & SD_CD);
		if (host->ocp_stat & (MS_OCP_NOW | MS_OCP_EVER)) {
			sdmmc_leave_idle(host);
			rtsx_usb_write_register(ucr, CARD_OE,
					SD_OUTPUT_EN, 0);
		}
	}

	if (!err) {
		err = rtsx_usb_read_register(ucr, CARD_INT_PEND, &pend);
		if (!err && (pend & SD_INT)) {
			rtsx_usb_write_register(ucr, CARD_INT_PEND,
				SD_INT, SD_INT);
			if (prev && present)
				present = false;
		}
	}

	if (!err && !present &&
	    (host->ocp_stat & (MS_OCP_NOW | MS_OCP_EVER))) {
		rtsx_usb_write_register(ucr, OCPCTL,
				MS_OCP_CLEAR, MS_OCP_CLEAR);
		host->ocp_stat = 0;
	}

	if (!err) {
		bool busy = READ_ONCE(host->mrq) != NULL;

		changed = present != prev;
		WRITE_ONCE(host->card_exist, present);

		if (busy) {
			host->idle_counter = 0;
			if (host->idle)
				sdmmc_leave_idle(host);
		} else if (!present) {
			if (host->idle_counter < host->idle_wait_max)
				host->idle_counter++;
			if (!host->idle && host->idle_counter >= host->idle_wait_max)
				sdmmc_enter_idle(host);
		} else {
			host->idle_counter = 0;
			if (host->idle)
				sdmmc_leave_idle(host);
		}

		if (!present) {
			if (host->seq_mode)
				sdmmc_stop_seq_mode(host);
			host->seq_mode = false;
			host->seq_read = false;
			host->seq_counter = 0;
		} else if (host->seq_mode) {
			if (busy) {
				host->seq_counter = 0;
			} else if (++host->seq_counter >= host->seq_wait_max) {
				sdmmc_stop_seq_mode(host);
			}
		} else {
			host->seq_counter = 0;
		}
	}

	mutex_unlock(&ucr->dev_mutex);

	pm_runtime_put_sync_suspend(dev);

	if (changed)
		mmc_detect_change(host->mmc, 0);

requeue:
	if (!host->host_removal) {
		bool request_active = READ_ONCE(host->mrq) != NULL;
		bool card_present = READ_ONCE(host->card_exist);
		unsigned long delay = card_present || request_active ?
			RTSX_USB_SD_POLL_INTERVAL : RTSX_USB_SD_IDLE_POLL_INTERVAL;

		host->poll_interval = delay;
		queue_delayed_work(system_wq, &host->card_poll, delay);
	}
}

static void sdmmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct rtsx_usb_sdmmc *host = mmc_priv(mmc);
	struct rtsx_ucr *ucr = host->ucr;
	struct mmc_command *cmd = mrq->cmd;
	struct mmc_data *data = mrq->data;
	unsigned int data_size = 0;

	dev_dbg(sdmmc_dev(host), "%s\n", __func__);

	if (host->host_removal) {
		cmd->error = -ENOMEDIUM;
		goto finish;
	}

	if ((!host->card_exist)) {
		cmd->error = -ENOMEDIUM;
		goto finish_detect_card;
	}
	/* check OCP stat */
	if (host->ocp_stat & (MS_OCP_NOW | MS_OCP_EVER)) {
		cmd->error = -ENOMEDIUM;
		goto finish_detect_card;
	}
	mutex_lock(&ucr->dev_mutex);
	if (host->seq_mode)
		sdmmc_stop_seq_mode(host);

	mutex_lock(&host->host_mutex);
	host->mrq = mrq;
	mutex_unlock(&host->host_mutex);
	sdmmc_leave_idle(host);
	host->idle_counter = 0;

	if (mrq->data)
		data_size = data->blocks * data->blksz;

	if (!data_size) {
		sd_send_cmd_get_rsp(host, cmd);
	} else if ((!(data_size % 512) && cmd->opcode != MMC_SEND_EXT_CSD) ||
		   mmc_op_multi(cmd->opcode)) {
		sd_send_cmd_get_rsp(host, cmd);

		if (!cmd->error) {
			sd_rw_multi(host, mrq);

			if (mmc_op_multi(cmd->opcode) && mrq->stop) {
				sd_send_cmd_get_rsp(host, mrq->stop);
				rtsx_usb_write_register(ucr, MC_FIFO_CTL,
						FIFO_FLUSH, FIFO_FLUSH);
			}
		}
	} else {
		sd_normal_rw(host, mrq);
	}

	if (mrq->data) {
		if (cmd->error || data->error)
			data->bytes_xfered = 0;
		else
			data->bytes_xfered = data->blocks * data->blksz;
	}

	mutex_unlock(&ucr->dev_mutex);

finish_detect_card:
	if (cmd->error) {
		/*
		 * detect card when fail to update card existence state and
		 * speed up card removal when retry
		 */
		sdmmc_get_cd(mmc);
		dev_dbg(sdmmc_dev(host), "cmd->error = %d\n", cmd->error);
	}

finish:
	mutex_lock(&host->host_mutex);
	host->mrq = NULL;
	mutex_unlock(&host->host_mutex);

	mmc_request_done(mmc, mrq);
}

static int rtsx_usb_cprm_rsp_config(u8 rsp_code, unsigned int *flags,
				 u8 *rsp_len)
{
	unsigned int cmd_flags;
	u8 len;

	switch (rsp_code) {
	case 0x04: /* R0 */
		cmd_flags = MMC_RSP_NONE;
		len = 0;
		break;
	case 0x01: /* R1/R5/R6/R7 */
		cmd_flags = MMC_RSP_R1;
		len = 4;
		break;
	case 0x09: /* R1b */
		cmd_flags = MMC_RSP_R1B;
		len = 4;
		break;
	case 0x02: /* R2 */
		cmd_flags = MMC_RSP_R2;
		len = 16;
		break;
	case 0x05: /* R3/R4 */
		cmd_flags = MMC_RSP_R3;
		len = 4;
		break;
	default:
		return -EINVAL;
	}

	if (flags)
		*flags = cmd_flags;
	if (rsp_len)
		*rsp_len = len;

	return 0;
}

static void rtsx_usb_cprm_store_resp(struct rtsx_usb_sdmmc *host,
				    struct mmc_command *cmd, u8 rsp_code,
				    u8 rsp_len)
{
	int i;

	host->cprm_rsp_type = rsp_code;
	host->cprm_rsp_len = rsp_len;
	host->cprm_rsp_valid = false;

	if (!rsp_len)
		goto out_valid;

	if (rsp_len == 16) {
		for (i = 0; i < 4; i++)
			put_unaligned_be32(cmd->resp[i], host->cprm_rsp + i * 4);
	} else {
		put_unaligned_be32(cmd->resp[0], host->cprm_rsp);
	}

out_valid:
	host->cprm_rsp_valid = true;
}

static int rtsx_usb_mmc_deselect_cards(struct mmc_host *host)
{
	struct mmc_command cmd = { };

	if (!host)
		return -EINVAL;

	cmd.opcode = MMC_SELECT_CARD;
	cmd.arg = 0;
	cmd.flags = MMC_RSP_NONE | MMC_CMD_AC;

	return mmc_wait_for_cmd(host, &cmd, RTSX_USB_MMC_CMD_RETRIES);
}

static int rtsx_usb_mmc_select_card(struct mmc_card *card)
{
	struct mmc_command cmd = { };

	if (!card)
		return -EINVAL;

	cmd.opcode = MMC_SELECT_CARD;
	cmd.arg = card->rca << 16;
	cmd.flags = MMC_RSP_R1 | MMC_CMD_AC;

	return mmc_wait_for_cmd(card->host, &cmd, RTSX_USB_MMC_CMD_RETRIES);
}

static int rtsx_usb_cprm_exec_direct(struct rtsx_usb_sdmmc *host,
				         const struct rtsx_usb_ioc_sd_direct *req)
{
	struct device *dev = sdmmc_dev(host);
	struct mmc_host *mmc = host->mmc;
	struct mmc_card *card = mmc->card;
	struct mmc_request mrq = {};
	struct mmc_command cmd = {};
	struct mmc_command stop = {};
	struct mmc_data data = {};
	struct scatterlist sg;
	unsigned int rsp_flags;
	u8 rsp_len;
	void *kbuf = NULL;
	void __user *user_buf;
	u32 arg, data_len;
	unsigned int blksz = 512;
	unsigned int blocks = 0;
	bool restore_blklen = false;
	bool send_stop, standby, acmd;
	u8 dir;
	int ret = 0;

	mutex_lock(&host->cprm_lock);
	host->cprm_rsp_valid = false;

	if (!card) {
		ret = -ENOMEDIUM;
		goto out_unlock;
	}

	dir = (req->cmnd[0] >> 3) & 0x03;
	send_stop = req->cmnd[0] & BIT(2);
	standby = req->cmnd[0] & BIT(1);
	acmd = req->cmnd[0] & BIT(0);

	cmd.opcode = req->cmnd[1];
	arg = ((u32)req->cmnd[2] << 24) |
	      ((u32)req->cmnd[3] << 16) |
	      ((u32)req->cmnd[4] << 8) |
	      (u32)req->cmnd[5];
	cmd.arg = arg;
	data_len = ((u32)req->cmnd[6] << 16) |
		   ((u32)req->cmnd[7] << 8) |
		   (u32)req->cmnd[8];
	cmd.error = 0;

	user_buf = u64_to_user_ptr(req->buf);

	if (req->buf_len < 0) {
		ret = -EINVAL;
		goto out_unlock;
	}
	switch (dir) {
	case 0:
		if (data_len) {
			ret = -EINVAL;
			goto out_unlock;
		}
		break;
	case 1:
	case 2:
		if (!data_len || req->buf_len < data_len) {
			ret = -EINVAL;
			goto out_unlock;
		}
		if (!user_buf) {
			ret = -EFAULT;
			goto out_unlock;
		}
		break;
	default:
		ret = -EINVAL;
		goto out_unlock;
	}

	ret = rtsx_usb_cprm_rsp_config(req->cmnd[9], &rsp_flags, &rsp_len);
	if (ret)
		goto out_unlock;

	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		pm_runtime_put_noidle(dev);
		goto out_unlock;
	}

	mmc_get_card(card);
	mmc_claim_host(mmc);

	if (standby) {
		ret = rtsx_usb_mmc_deselect_cards(mmc);
		if (ret)
			goto out_release;
	}

	if (acmd) {
		ret = mmc_app_cmd(mmc, card);
		if (ret)
			goto out_restore_select;
	}

	if (dir) {
		if (data_len < 512) {
			ret = mmc_set_blocklen(card, data_len);
			if (ret)
				goto out_restore_select;
			blksz = data_len;
			blocks = 1;
			restore_blklen = true;
		} else if (data_len % 512) {
			ret = -EINVAL;
			goto out_restore_select;
		} else {
			blocks = data_len / 512;
		}

		kbuf = kvmalloc(data_len, GFP_KERNEL);
		if (!kbuf) {
			ret = -ENOMEM;
			goto out_restore_blklen;
		}

		if (dir == 2) {
			if (copy_from_user(kbuf, user_buf, data_len)) {
				ret = -EFAULT;
				goto out_restore_blklen;
			}
		}

		data.blksz = blksz;
		data.blocks = blocks ? blocks : 1;
		data.flags = (dir == 1) ? MMC_DATA_READ : MMC_DATA_WRITE;
		data.sg = &sg;
		data.sg_len = 1;
		data.error = 0;
		sg_init_one(&sg, kbuf, data_len);
		mmc_set_data_timeout(&data, card);
		mrq.data = &data;
		cmd.flags = rsp_flags | MMC_CMD_ADTC;
	} else {
		cmd.flags = rsp_flags | MMC_CMD_AC;
	}

	mrq.cmd = &cmd;

	if (dir && send_stop) {
		stop.opcode = MMC_STOP_TRANSMISSION;
		stop.arg = 0;
		stop.flags = MMC_RSP_R1B | MMC_CMD_AC;
		mrq.stop = &stop;
	}

	mmc_wait_for_req(mmc, &mrq);

	if (cmd.error)
		ret = cmd.error;
	else if (mrq.data && mrq.data->error)
		ret = mrq.data->error;
	else if (mrq.stop && mrq.stop->error)
		ret = mrq.stop->error;
	else
		ret = 0;

	if (restore_blklen) {
		int err = mmc_set_blocklen(card, 512);

		if (!ret && err)
			ret = err;
	}

	if (standby) {
		int err = rtsx_usb_mmc_select_card(card);

		if (!ret && err)
			ret = err;
	}

	mmc_release_host(mmc);
	mmc_put_card(card);
	pm_runtime_put(dev);

	if (ret)
		goto out_copy_err;

	if (dir == 1) {
		if (copy_to_user(user_buf, kbuf, data_len)) {
			ret = -EFAULT;
			goto out_copy_err;
		}
	}

	rtsx_usb_cprm_store_resp(host, &cmd, req->cmnd[9], rsp_len);

	ret = 0;

out_copy_err:
	if (ret)
		host->cprm_rsp_valid = false;
	kvfree(kbuf);
	mutex_unlock(&host->cprm_lock);
	return ret;

out_restore_blklen:
	if (restore_blklen) {
		int err_restore = mmc_set_blocklen(card, 512);

		if (!ret && err_restore)
			ret = err_restore;
	}
out_restore_select:
	if (standby) {
		int err_select = rtsx_usb_mmc_select_card(card);

		if (!ret && err_select)
			ret = err_select;
	}
out_release:
	mmc_release_host(mmc);
	mmc_put_card(card);
	pm_runtime_put(dev);
out_unlock:
	kvfree(kbuf);
	mutex_unlock(&host->cprm_lock);
	return ret;
}

static int rtsx_usb_cprm_get_response(struct rtsx_usb_sdmmc *host,
				      struct rtsx_usb_ioc_sd_rsp *req)
{
	void __user *user_rsp = u64_to_user_ptr(req->rsp);
	size_t to_copy;
	int ret = 0;

	mutex_lock(&host->cprm_lock);
	if (!host->cprm_rsp_valid) {
		ret = -EINVAL;
		goto out_unlock;
	}
	if (req->rsp_len < 0) {
		ret = -EINVAL;
		goto out_unlock;
	}

	to_copy = min_t(size_t, host->cprm_rsp_len,
		       (size_t)req->rsp_len);

	if (to_copy && copy_to_user(user_rsp, host->cprm_rsp, to_copy)) {
		ret = -EFAULT;
		goto out_unlock;
	}

	req->rsp_len = to_copy;

out_unlock:
	mutex_unlock(&host->cprm_lock);
	return ret;
}

static int rtsx_usb_cprm_open(struct inode *inode, struct file *file)
{
	struct rtsx_usb_sdmmc *host;

	host = container_of(file->private_data,
			    struct rtsx_usb_sdmmc, cprm_miscdev);
	if (!host || host->host_removal)
		return -ENODEV;

	file->private_data = host;
	return nonseekable_open(inode, file);
}

static long rtsx_usb_cprm_unlocked_ioctl(struct file *file, unsigned int cmd,
					   unsigned long arg)
{
	struct rtsx_usb_sdmmc *host = file->private_data;
	struct rtsx_usb_ioc_sd_direct direct;
	struct rtsx_usb_ioc_sd_rsp rsp;
	void __user *argp = (void __user *)arg;
	int ret;

	if (host->host_removal)
		return -ENODEV;

	switch (cmd) {
	case RTSX_USB_IOC_SD_DIRECT:
		if (copy_from_user(&direct, argp, sizeof(direct)))
			return -EFAULT;
		ret = rtsx_usb_cprm_exec_direct(host, &direct);
		return ret;
	case RTSX_USB_IOC_SD_GET_RSP:
		if (copy_from_user(&rsp, argp, sizeof(rsp)))
			return -EFAULT;
		ret = rtsx_usb_cprm_get_response(host, &rsp);
		if (ret)
			return ret;
		if (copy_to_user(argp, &rsp, sizeof(rsp)))
			return -EFAULT;
		return 0;
	default:
		return -ENOTTY;
	}
}

#ifdef CONFIG_COMPAT
static long rtsx_usb_cprm_compat_ioctl(struct file *file, unsigned int cmd,
					     unsigned long arg)
{
	struct rtsx_usb_sdmmc *host = file->private_data;
	void __user *argp = compat_ptr(arg);

	if (host->host_removal)
		return -ENODEV;

	switch (cmd) {
	case RTSX_USB_IOC_SD_DIRECT32:
	{
		struct rtsx_usb_ioc_sd_direct32 direct32;
		struct rtsx_usb_ioc_sd_direct direct;

		if (copy_from_user(&direct32, argp, sizeof(direct32)))
			return -EFAULT;
		memcpy(direct.cmnd, direct32.cmnd, sizeof(direct.cmnd));
		direct.buf = direct32.buf;
		direct.buf_len = direct32.buf_len;
		return rtsx_usb_cprm_exec_direct(host, &direct);
	}
	case RTSX_USB_IOC_SD_GET_RSP32:
	{
		struct rtsx_usb_ioc_sd_rsp32 rsp32;
		struct rtsx_usb_ioc_sd_rsp rsp;
		int ret;

		if (copy_from_user(&rsp32, argp, sizeof(rsp32)))
			return -EFAULT;
		rsp.rsp = rsp32.rsp;
		rsp.rsp_len = rsp32.rsp_len;
		ret = rtsx_usb_cprm_get_response(host, &rsp);
		if (ret)
			return ret;
		rsp32.rsp_len = rsp.rsp_len;
		if (copy_to_user(argp, &rsp32, sizeof(rsp32)))
			return -EFAULT;
		return 0;
	}
	default:
		return -ENOTTY;
	}
}
#endif

static const struct file_operations rtsx_usb_cprm_fops = {
	.owner		= THIS_MODULE,
	.open		= rtsx_usb_cprm_open,
	.unlocked_ioctl = rtsx_usb_cprm_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= rtsx_usb_cprm_compat_ioctl,
#endif
	.llseek		= no_llseek,
};

static int sd_set_bus_width(struct rtsx_usb_sdmmc *host,
		unsigned char bus_width)
{
	int err = 0;
	static const u8 width[] = {
		[MMC_BUS_WIDTH_1] = SD_BUS_WIDTH_1BIT,
		[MMC_BUS_WIDTH_4] = SD_BUS_WIDTH_4BIT,
		[MMC_BUS_WIDTH_8] = SD_BUS_WIDTH_8BIT,
	};

	if (bus_width <= MMC_BUS_WIDTH_8)
		err = rtsx_usb_write_register(host->ucr, SD_CFG1,
				0x03, width[bus_width]);

	return err;
}

static int sd_pull_ctl_disable_lqfp48(struct rtsx_ucr *ucr)
{
	rtsx_usb_init_cmd(ucr);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL1, 0xFF, 0x55);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL2, 0xFF, 0x55);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL3, 0xFF, 0x95);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL4, 0xFF, 0x55);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL5, 0xFF, 0x55);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL6, 0xFF, 0xA5);

	return rtsx_usb_send_cmd(ucr, MODE_C, 100);
}

static int sd_pull_ctl_disable_qfn24(struct rtsx_ucr *ucr)
{
	rtsx_usb_init_cmd(ucr);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL1, 0xFF, 0x65);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL2, 0xFF, 0x55);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL3, 0xFF, 0x95);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL4, 0xFF, 0x55);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL5, 0xFF, 0x56);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL6, 0xFF, 0x59);

	return rtsx_usb_send_cmd(ucr, MODE_C, 100);
}

static int sd_pull_ctl_enable_lqfp48(struct rtsx_ucr *ucr)
{
	rtsx_usb_init_cmd(ucr);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL1, 0xFF, 0xAA);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL2, 0xFF, 0xAA);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL3, 0xFF, 0xA9);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL4, 0xFF, 0x55);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL5, 0xFF, 0x55);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL6, 0xFF, 0xA5);

	return rtsx_usb_send_cmd(ucr, MODE_C, 100);
}

static int sd_pull_ctl_enable_qfn24(struct rtsx_ucr *ucr)
{
	rtsx_usb_init_cmd(ucr);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL1, 0xFF, 0xA5);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL2, 0xFF, 0x9A);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL3, 0xFF, 0xA5);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL4, 0xFF, 0x9A);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL5, 0xFF, 0x65);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PULL_CTL6, 0xFF, 0x5A);

	return rtsx_usb_send_cmd(ucr, MODE_C, 100);
}

static int sd_power_on(struct rtsx_usb_sdmmc *host)
{
	struct rtsx_ucr *ucr = host->ucr;
	int err;

	if (host->ocp_stat & (MS_OCP_NOW | MS_OCP_EVER)) {
		dev_dbg(sdmmc_dev(host), "over current\n");
		return -EIO;
	}
	dev_dbg(sdmmc_dev(host), "%s\n", __func__);
	rtsx_usb_init_cmd(ucr);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_SELECT, 0x07, SD_MOD_SEL);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_SHARE_MODE,
			CARD_SHARE_MASK, CARD_SHARE_SD);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_CLK_EN,
			SD_CLK_EN, SD_CLK_EN);
	err = rtsx_usb_send_cmd(ucr, MODE_C, 100);
	if (err)
		return err;

	if (CHECK_PKG(ucr, LQFP48))
		err = sd_pull_ctl_enable_lqfp48(ucr);
	else
		err = sd_pull_ctl_enable_qfn24(ucr);
	if (err)
		return err;

	err = rtsx_usb_write_register(ucr, CARD_PWR_CTL,
			POWER_MASK, PARTIAL_POWER_ON);
	if (err)
		return err;

	usleep_range(800, 1000);

	rtsx_usb_init_cmd(ucr);
	/* WA OCP issue: after OCP, there were problems with reopen card power */
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PWR_CTL, POWER_MASK, POWER_ON);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, FPDCTL, SSC_POWER_MASK, SSC_POWER_DOWN);
	err = rtsx_usb_send_cmd(ucr, MODE_C, 100);
	if (err)
		return err;
	msleep(20);
	rtsx_usb_write_register(ucr, FPDCTL, SSC_POWER_MASK, SSC_POWER_ON);
	usleep_range(180, 200);
	rtsx_usb_init_cmd(ucr);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PWR_CTL,
			LDO3318_PWR_MASK, LDO_ON);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_OE,
			SD_OUTPUT_EN, SD_OUTPUT_EN);

	return rtsx_usb_send_cmd(ucr, MODE_C, 100);
}

static int sd_power_off(struct rtsx_usb_sdmmc *host)
{
	struct rtsx_ucr *ucr = host->ucr;
	int err;

	dev_dbg(sdmmc_dev(host), "%s\n", __func__);
	rtsx_usb_init_cmd(ucr);

	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_CLK_EN, SD_CLK_EN, 0);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_OE, SD_OUTPUT_EN, 0);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PWR_CTL,
			POWER_MASK, POWER_OFF);
	rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_PWR_CTL,
			POWER_MASK|LDO3318_PWR_MASK, POWER_OFF|LDO_SUSPEND);

	err = rtsx_usb_send_cmd(ucr, MODE_C, 100);
	if (err)
		return err;

	if (CHECK_PKG(ucr, LQFP48))
			return sd_pull_ctl_disable_lqfp48(ucr);
	return sd_pull_ctl_disable_qfn24(ucr);
}

static void sd_set_power_mode(struct rtsx_usb_sdmmc *host,
		unsigned char power_mode)
{
	int err;
	struct rtsx_ucr *ucr = host->ucr;

	if (power_mode == host->power_mode)
		return;

	switch (power_mode) {
	case MMC_POWER_OFF:
		err = sd_power_off(host);
		if (err)
			dev_dbg(sdmmc_dev(host), "power-off (err = %d)\n", err);
		pm_runtime_put_noidle(sdmmc_dev(host));
		sdmmc_enter_idle(host);
		host->idle_counter = 0;
		break;

	case MMC_POWER_UP:
		pm_runtime_get_noresume(sdmmc_dev(host));
		err = sd_power_on(host);
		if (err)
			dev_dbg(sdmmc_dev(host), "power-on (err = %d)\n", err);
		/* issue the clock signals to card at least 74 clocks */
		rtsx_usb_write_register(ucr, SD_BUS_STAT, SD_CLK_TOGGLE_EN, SD_CLK_TOGGLE_EN);
		sdmmc_leave_idle(host);
		host->idle_counter = 0;
		break;

	case MMC_POWER_ON:
		/* stop to send the clock signals */
		rtsx_usb_write_register(ucr, SD_BUS_STAT, SD_CLK_TOGGLE_EN, 0x00);
		sdmmc_leave_idle(host);
		host->idle_counter = 0;
		break;

	case MMC_POWER_UNDEFINED:
		break;

	default:
		break;
	}

	host->power_mode = power_mode;
}

static int sd_set_timing(struct rtsx_usb_sdmmc *host,
		unsigned char timing, bool *ddr_mode)
{
	struct rtsx_ucr *ucr = host->ucr;

	*ddr_mode = false;

	rtsx_usb_init_cmd(ucr);

	switch (timing) {
	case MMC_TIMING_UHS_SDR104:
	case MMC_TIMING_UHS_SDR50:
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CFG1,
				0x0C | SD_ASYNC_FIFO_RST,
				SD_30_MODE | SD_ASYNC_FIFO_RST);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_CLK_SOURCE, 0xFF,
				CRC_VAR_CLK0 | SD30_FIX_CLK | SAMPLE_VAR_CLK1);
		break;

	case MMC_TIMING_UHS_DDR50:
	case MMC_TIMING_MMC_DDR52:
		*ddr_mode = true;

		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CFG1,
				0x0C | SD_ASYNC_FIFO_RST,
				SD_DDR_MODE | SD_ASYNC_FIFO_RST);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_CLK_SOURCE, 0xFF,
				CRC_VAR_CLK0 | SD30_FIX_CLK | SAMPLE_VAR_CLK1);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_PUSH_POINT_CTL,
				DDR_VAR_TX_CMD_DAT, DDR_VAR_TX_CMD_DAT);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_SAMPLE_POINT_CTL,
				DDR_VAR_RX_DAT | DDR_VAR_RX_CMD,
				DDR_VAR_RX_DAT | DDR_VAR_RX_CMD);
		break;

	case MMC_TIMING_MMC_HS:
	case MMC_TIMING_SD_HS:
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_CFG1,
				0x0C, SD_20_MODE);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_CLK_SOURCE, 0xFF,
				CRC_FIX_CLK | SD30_VAR_CLK0 | SAMPLE_VAR_CLK1);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_PUSH_POINT_CTL,
				SD20_TX_SEL_MASK, SD20_TX_14_AHEAD);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_SAMPLE_POINT_CTL,
				SD20_RX_SEL_MASK, SD20_RX_14_DELAY);
		break;

	default:
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_CFG1, 0x0C, SD_20_MODE);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, CARD_CLK_SOURCE, 0xFF,
				CRC_FIX_CLK | SD30_VAR_CLK0 | SAMPLE_VAR_CLK1);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD,
				SD_PUSH_POINT_CTL, 0xFF, 0);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_SAMPLE_POINT_CTL,
				SD20_RX_SEL_MASK, SD20_RX_POS_EDGE);
		break;
	}

	return rtsx_usb_send_cmd(ucr, MODE_C, 100);
}

static void sdmmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct rtsx_usb_sdmmc *host = mmc_priv(mmc);
	struct rtsx_ucr *ucr = host->ucr;

	dev_dbg(sdmmc_dev(host), "%s\n", __func__);
	mutex_lock(&ucr->dev_mutex);

	sd_set_power_mode(host, ios->power_mode);
	sd_set_bus_width(host, ios->bus_width);
	sd_set_timing(host, ios->timing, &host->ddr_mode);

	host->vpclk = false;
	host->double_clk = true;

	switch (ios->timing) {
	case MMC_TIMING_UHS_SDR104:
	case MMC_TIMING_UHS_SDR50:
		host->ssc_depth = SSC_DEPTH_2M;
		host->vpclk = true;
		host->double_clk = false;
		break;
	case MMC_TIMING_UHS_DDR50:
	case MMC_TIMING_MMC_DDR52:
	case MMC_TIMING_UHS_SDR25:
		host->ssc_depth = SSC_DEPTH_1M;
		break;
	default:
		host->ssc_depth = SSC_DEPTH_512K;
		break;
	}

	host->initial_mode = (ios->clock <= 1000000) ? true : false;
	host->clock = ios->clock;

	rtsx_usb_switch_clock(host->ucr, host->clock, host->ssc_depth,
			host->initial_mode, host->double_clk, host->vpclk);

	mutex_unlock(&ucr->dev_mutex);
	dev_dbg(sdmmc_dev(host), "%s end\n", __func__);
}

static int sdmmc_switch_voltage(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct rtsx_usb_sdmmc *host = mmc_priv(mmc);
	struct rtsx_ucr *ucr = host->ucr;
	int err = 0;

	dev_dbg(sdmmc_dev(host), "%s: signal_voltage = %d\n",
			__func__, ios->signal_voltage);

	if (host->host_removal)
		return -ENOMEDIUM;

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_120)
		return -EPERM;

	mutex_lock(&ucr->dev_mutex);

	err = rtsx_usb_card_exclusive_check(ucr, RTSX_USB_SD_CARD);
	if (err) {
		mutex_unlock(&ucr->dev_mutex);
		return err;
	}

	/* Let mmc core do the busy checking, simply stop the forced-toggle
	 * clock(while issuing CMD11) and switch voltage.
	 */
	rtsx_usb_init_cmd(ucr);

	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330) {
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_PAD_CTL,
				SD_IO_USING_1V8, SD_IO_USING_3V3);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, LDO_POWER_CFG,
				TUNE_SD18_MASK, TUNE_SD18_3V3);
	} else {
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_BUS_STAT,
				SD_CLK_TOGGLE_EN | SD_CLK_FORCE_STOP,
				SD_CLK_FORCE_STOP);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, SD_PAD_CTL,
				SD_IO_USING_1V8, SD_IO_USING_1V8);
		rtsx_usb_add_cmd(ucr, WRITE_REG_CMD, LDO_POWER_CFG,
				TUNE_SD18_MASK, TUNE_SD18_1V8);
	}

	err = rtsx_usb_send_cmd(ucr, MODE_C, 100);
	mutex_unlock(&ucr->dev_mutex);

	return err;
}

static int sdmmc_card_busy(struct mmc_host *mmc)
{
	struct rtsx_usb_sdmmc *host = mmc_priv(mmc);
	struct rtsx_ucr *ucr = host->ucr;
	int err;
	u8 stat;
	u8 mask = SD_DAT3_STATUS | SD_DAT2_STATUS | SD_DAT1_STATUS
		| SD_DAT0_STATUS;

	dev_dbg(sdmmc_dev(host), "%s\n", __func__);

	mutex_lock(&ucr->dev_mutex);

	err = rtsx_usb_write_register(ucr, SD_BUS_STAT,
			SD_CLK_TOGGLE_EN | SD_CLK_FORCE_STOP,
			SD_CLK_TOGGLE_EN);
	if (err)
		goto out;

	mdelay(1);

	err = rtsx_usb_read_register(ucr, SD_BUS_STAT, &stat);
	if (err)
		goto out;

	err = rtsx_usb_write_register(ucr, SD_BUS_STAT,
			SD_CLK_TOGGLE_EN | SD_CLK_FORCE_STOP, 0);
out:
	mutex_unlock(&ucr->dev_mutex);

	if (err)
		return err;

	/* check if any pin between dat[0:3] is low */
	if ((stat & mask) != mask)
		return 1;
	else
		return 0;
}

static int sdmmc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct rtsx_usb_sdmmc *host = mmc_priv(mmc);
	struct rtsx_ucr *ucr = host->ucr;
	int err = 0;

	if (host->host_removal)
		return -ENOMEDIUM;

	mutex_lock(&ucr->dev_mutex);

	if (!host->ddr_mode)
		err = sd_tuning_rx(host, MMC_SEND_TUNING_BLOCK);

	mutex_unlock(&ucr->dev_mutex);

	return err;
}

static const struct mmc_host_ops rtsx_usb_sdmmc_ops = {
	.request = sdmmc_request,
	.set_ios = sdmmc_set_ios,
	.get_ro = sdmmc_get_ro,
	.get_cd = sdmmc_get_cd,
	.start_signal_voltage_switch = sdmmc_switch_voltage,
	.card_busy = sdmmc_card_busy,
	.execute_tuning = sdmmc_execute_tuning,
};

#ifdef RTSX_USB_USE_LEDS_CLASS
static void rtsx_usb_led_control(struct led_classdev *led,
	enum led_brightness brightness)
{
	struct rtsx_usb_sdmmc *host = container_of(led,
			struct rtsx_usb_sdmmc, led);

	if (host->host_removal)
		return;

	host->led.brightness = brightness;
	schedule_work(&host->led_work);
}

static void rtsx_usb_update_led(struct work_struct *work)
{
	struct rtsx_usb_sdmmc *host =
		container_of(work, struct rtsx_usb_sdmmc, led_work);
	struct rtsx_ucr *ucr = host->ucr;

	pm_runtime_get_noresume(sdmmc_dev(host));
	mutex_lock(&ucr->dev_mutex);

	if (host->power_mode == MMC_POWER_OFF)
		goto out;

	if (host->led.brightness == LED_OFF)
		rtsx_usb_turn_off_led(ucr);
	else
		rtsx_usb_turn_on_led(ucr);

out:
	mutex_unlock(&ucr->dev_mutex);
	pm_runtime_put_sync_suspend(sdmmc_dev(host));
}
#endif

static void rtsx_usb_init_host(struct rtsx_usb_sdmmc *host)
{
	struct mmc_host *mmc = host->mmc;

	mmc->f_min = 250000;
	mmc->f_max = 208000000;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_165_195;
	mmc->caps = MMC_CAP_4_BIT_DATA | MMC_CAP_SD_HIGHSPEED |
		MMC_CAP_MMC_HIGHSPEED | MMC_CAP_BUS_WIDTH_TEST |
		MMC_CAP_UHS_SDR12 | MMC_CAP_UHS_SDR25 |
		MMC_CAP_SYNC_RUNTIME_PM;
	if (host->ucr->supports_sdr50)
		mmc->caps |= MMC_CAP_UHS_SDR50;
	if (host->ucr->supports_ddr50)
		mmc->caps |= MMC_CAP_UHS_DDR50;
	if (host->ucr->supports_mmc_ddr)
		mmc->caps |= MMC_CAP_1_8V_DDR;
	mmc->caps2 = MMC_CAP2_NO_PRESCAN_POWERUP | MMC_CAP2_FULL_PWR_CYCLE |
		MMC_CAP2_NO_SDIO;

	mmc->max_current_330 = 400;
	mmc->max_current_180 = 800;
	mmc->ops = &rtsx_usb_sdmmc_ops;
	mmc->max_segs = 256;
	mmc->max_seg_size = 65536;
	mmc->max_blk_size = 512;
	mmc->max_blk_count = 65535;
	mmc->max_req_size = 524288;

	host->power_mode = MMC_POWER_OFF;
	host->ocp_stat = 0;
	host->idle_counter = 0;
	host->idle_wait_max = RTSX_USB_SD_IDLE_WAIT_MAX;
	host->idle = false;
	host->seq_mode = false;
	host->seq_read = false;
	host->seq_counter = 0;
	host->seq_wait_max = RTSX_USB_SD_SEQ_WAIT_MAX;
	mutex_init(&host->cprm_lock);
	host->cprm_rsp_len = 0;
	host->cprm_rsp_valid = false;
	host->cprm_rsp_type = 0;
	host->cprm_registered = false;
	host->cprm_name = NULL;
}

static int rtsx_usb_sdmmc_drv_probe(struct platform_device *pdev)
{
	struct mmc_host *mmc;
	struct rtsx_usb_sdmmc *host;
	struct rtsx_ucr *ucr;
#ifdef RTSX_USB_USE_LEDS_CLASS
	int err;
#endif
	int ret;

	ucr = usb_get_intfdata(to_usb_interface(pdev->dev.parent));
	if (!ucr)
		return -ENXIO;

	dev_dbg(&(pdev->dev), ": Realtek USB SD/MMC controller found\n");

	mmc = devm_mmc_alloc_host(&pdev->dev, sizeof(*host));
	if (!mmc)
		return -ENOMEM;

	host = mmc_priv(mmc);
	host->ucr = ucr;
	host->mmc = mmc;
	host->pdev = pdev;
	platform_set_drvdata(pdev, host);

	mutex_init(&host->host_mutex);
	rtsx_usb_init_host(host);
	host->poll_interval = RTSX_USB_SD_POLL_INTERVAL;
	INIT_DELAYED_WORK(&host->card_poll, rtsx_usb_sdmmc_poll_card);
	pm_runtime_enable(&pdev->dev);

#ifdef RTSX_USB_USE_LEDS_CLASS
	snprintf(host->led_name, sizeof(host->led_name),
		"%s::", mmc_hostname(mmc));
	host->led.name = host->led_name;
	host->led.brightness = LED_OFF;
	host->led.default_trigger = mmc_hostname(mmc);
	host->led.brightness_set = rtsx_usb_led_control;

	err = led_classdev_register(mmc_dev(mmc), &host->led);
	if (err)
		dev_err(&(pdev->dev),
				"Failed to register LED device: %d\n", err);
	INIT_WORK(&host->led_work, rtsx_usb_update_led);

#endif
	ret = mmc_add_host(mmc);
	if (ret) {
#ifdef RTSX_USB_USE_LEDS_CLASS
		led_classdev_unregister(&host->led);
#endif
		pm_runtime_disable(&pdev->dev);
		return ret;
	}

	host->cprm_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s_cprm",
				      mmc_hostname(mmc));
	if (host->cprm_name) {
		host->cprm_miscdev.minor = MISC_DYNAMIC_MINOR;
		host->cprm_miscdev.name = host->cprm_name;
		host->cprm_miscdev.fops = &rtsx_usb_cprm_fops;
		host->cprm_miscdev.parent = &pdev->dev;
		host->cprm_miscdev.mode = 0600;
		ret = misc_register(&host->cprm_miscdev);
		if (ret) {
			dev_warn(&(pdev->dev),
				 "Failed to register CPRM interface: %d\n", ret);
		} else {
			host->cprm_registered = true;
			dev_set_drvdata(host->cprm_miscdev.this_device, host);
		}
	} else {
		dev_warn(&(pdev->dev),
			 "Failed to allocate CPRM interface name\n");
	}

	queue_delayed_work(system_wq, &host->card_poll,
			 host->poll_interval);

	return 0;
}

static void rtsx_usb_sdmmc_drv_remove(struct platform_device *pdev)
{
	struct rtsx_usb_sdmmc *host = platform_get_drvdata(pdev);
	struct mmc_host *mmc;

	if (!host)
		return;

	mmc = host->mmc;
	host->host_removal = true;
	cancel_delayed_work_sync(&host->card_poll);
	if (host->cprm_registered) {
		dev_set_drvdata(host->cprm_miscdev.this_device, NULL);
		misc_deregister(&host->cprm_miscdev);
		host->cprm_registered = false;
	}

	mutex_lock(&host->host_mutex);
	if (host->mrq) {
		dev_dbg(&(pdev->dev),
			"%s: Controller removed during transfer\n",
			mmc_hostname(mmc));
		host->mrq->cmd->error = -ENOMEDIUM;
		if (host->mrq->stop)
			host->mrq->stop->error = -ENOMEDIUM;
		mmc_request_done(mmc, host->mrq);
	}
	mutex_unlock(&host->host_mutex);

	mmc_remove_host(mmc);

#ifdef RTSX_USB_USE_LEDS_CLASS
	cancel_work_sync(&host->led_work);
	led_classdev_unregister(&host->led);
#endif

	pm_runtime_disable(&pdev->dev);
	platform_set_drvdata(pdev, NULL);

	dev_dbg(&(pdev->dev),
		": Realtek USB SD/MMC module has been removed\n");
}

static int rtsx_usb_sdmmc_runtime_suspend(struct device *dev)
{
	struct rtsx_usb_sdmmc *host = dev_get_drvdata(dev);

	host->mmc->caps &= ~MMC_CAP_NEEDS_POLL;
	return 0;
}

static int rtsx_usb_sdmmc_runtime_resume(struct device *dev)
{
	struct rtsx_usb_sdmmc *host = dev_get_drvdata(dev);

	host->mmc->caps |= MMC_CAP_NEEDS_POLL;
	if (sdmmc_get_cd(host->mmc) == 1)
		mmc_detect_change(host->mmc, 0);
	return 0;
}

static const struct dev_pm_ops rtsx_usb_sdmmc_dev_pm_ops = {
	RUNTIME_PM_OPS(rtsx_usb_sdmmc_runtime_suspend, rtsx_usb_sdmmc_runtime_resume, NULL)
};

static const struct platform_device_id rtsx_usb_sdmmc_ids[] = {
	{
		.name = "rtsx_usb_sdmmc",
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, rtsx_usb_sdmmc_ids);

static struct platform_driver rtsx_usb_sdmmc_driver = {
	.probe		= rtsx_usb_sdmmc_drv_probe,
	.remove		= rtsx_usb_sdmmc_drv_remove,
	.id_table       = rtsx_usb_sdmmc_ids,
	.driver		= {
		.name	= "rtsx_usb_sdmmc",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.pm	= pm_ptr(&rtsx_usb_sdmmc_dev_pm_ops),
	},
};
module_platform_driver(rtsx_usb_sdmmc_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Roger Tseng <rogerable@realtek.com>");
MODULE_DESCRIPTION("Realtek USB SD/MMC Card Host Driver");
