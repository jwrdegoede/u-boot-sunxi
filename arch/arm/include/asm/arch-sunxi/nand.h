/*
 * (C) Copyright 2015 Roy Spliet <r.spliet@ultimaker.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _SUNXI_NAND_H
#define _SUNXI_NAND_H

#include <linux/types.h>

struct sunxi_nand
{
	u32 ctl;		/* 0x000 Configure and control */
	u32 st;			/* 0x004 Status information */
	u32 intr;		/* 0x008 Interrupt control */
	u32 timing_ctl;		/* 0x00C Timing control */
	u32 timing_cfg;		/* 0x010 Timing configure */
	u32 addr_low;		/* 0x014 Low word address */
	u32 addr_high;		/* 0x018 High word address */
	u32 block_num;		/* 0x01C Data block number */
	u32 data_cnt;		/* 0x020 Data counter for transfer */
	u32 cmd;		/* 0x024 NDFC commands */
	u32 rcmd_set;		/* 0x028 Read command set for vendor NAND mem */
	u32 wcmd_set;		/* 0x02C Write command set */
	u32 io_data;		/* 0x030 IO data */
	u32 ecc_ctl;		/* 0x034 ECC configure and control */
	u32 ecc_st;		/* 0x038 ECC status and operation info */
	u32 efr;		/* 0x03C Enhanced feature */
	u32 err_cnt[4];		/* 0x040[4] Corrected error bit counter 0 */
	u32 user_data[16];	/* 0x050[16] User data field */
	u32 efnand_st;		/* 0x090 EFNAND status */
	u32 res0[3];
	u32 spare_area;		/* 0x0A0 Spare area configure */
	u32 pat_id;		/* 0x0A4 Pattern ID register */
	u32 rdata_sta_ctl;	/* 0x0A8 Read data status control */
	u32 rdata_sta_0;	/* 0x0AC Read data status 0 */
	u32 rdata_sta_1;	/* 0x0B0 Read data status 1 */
	u32 res1[3];
	u32 mdma_addr;		/* 0x0C0 MBUS DMA Address */
	u32 mdma_cnt;		/* 0x0C4 MBUS DMA data counter */
	u32 res2[206];
	u32 ram0_base;
};

#define SUNXI_NAND_CTL_EN			(1 << 0)
#define SUNXI_NAND_CTL_RST			(1 << 1)
#define SUNXI_NAND_BUS_WIDTH			(1 << 2)
#define SUNXI_NAND_CTL_RB_SEL_MASK		(0x3 << 3)
#define SUNXI_NAND_CTL_RB_SEL(a)		((a) << 3)
#define SUNXI_NAND_CTL_CE_ACT			(1 << 6)
#define SUNXI_NAND_CTL_PAGE_SIZE_MASK		(0xf << 8)
#define SUNXI_NAND_CTL_PAGE_SIZE(a)		((fls(a) - 11) << 8)
#define SUNXI_NAND_CTL_RAM_METHOD_DMA		(1 << 14)
#define SUNXI_NAND_CTL_CE_SEL_MASK		(0xf << 24)
#define SUNXI_NAND_CTL_CE_SEL(a)		(a << 24)
#define SUNXI_NAND_CTL_DEBUG			(1 << 31)

#define SUNXI_NAND_ST_RB_B2R			(1 << 0)
#define SUNXI_NAND_ST_CMD_INT			(1 << 1)
#define SUNXI_NAND_ST_DMA_INT			(1 << 2)
#define SUNXI_NAND_ST_FIFO_FULL			(1 << 3)
#define SUNXI_NAND_ST_BUSY			(1 << 4)
#define SUNXI_NAND_ST_RB_STATE0			(1 << 8)
#define SUNXI_NAND_ST_RB_STATE1			(1 << 9)
#define SUNXI_NAND_ST_RB_STATE2			(1 << 10)
#define SUNXI_NAND_ST_RB_STATE3			(1 << 11)


#define SUNXI_NAND_INT_B2R_ENABLE		(1 << 0)
#define SUNXI_NAND_INT_CMD_ENABLE		(1 << 1)
#define SUNXI_NAND_INT_DMA_ENABLE		(1 << 2)
#define SUNXI_NAND_INT_MASK	(SUNXI_NAND_INT_B2R_ENABLE | \
				 SUNXI_NAND_INT_CMD_ENABLE | \
				 SUNXI_NAND_INT_DMA_ENABLE)

#define SUNXI_NAND_CMD_LOW_BYTE(a)		(a & 0xff)
#define SUNXI_NAND_CMD_HIGH_BYTE(a)		((a & 0xff) << 8)
#define SUNXI_NAND_CMD_ADDR_CYCLES(a)		((a - 1) << 16);
#define SUNXI_NAND_CMD_SEND_ADR			(1 << 19)
#define SUNXI_NAND_CMD_ACCESS_RD		0
#define SUNXI_NAND_CMD_ACCESS_WR		(1 << 20)
#define SUNXI_NAND_CMD_DATA_TRANS		(1 << 21)
#define SUNXI_NAND_CMD_SEND_CMD1		(1 << 22)
#define SUNXI_NAND_CMD_WAIT_FLAG		(1 << 23)
#define SUNXI_NAND_CMD_SEND_CMD2		(1 << 24)
#define SUNXI_NAND_CMD_ORDER_INTERLEAVE		0
#define SUNXI_NAND_CMD_ORDER_SEQ		(1 << 25)
#define SUNXI_NAND_CMD_DATA_SWAP_METHOD		(1 << 26)
#define SUNXI_NAND_CMD_ROW_AUTO_INC		(1 << 27)
#define SUNXI_NAND_CMD_SEND_CMD3		(1 << 28)
#define SUNXI_NAND_CMD_SEND_CMD4		(1 << 29)

#define SUNXI_NAND_ECC_CTL_ECC_EN		(1 << 0)
#define SUNXI_NAND_ECC_CTL_PIPELINE		(1 << 3)
#define SUNXI_NAND_ECC_CTL_EXCEPTION		(1 << 4)
#define SUNXI_NAND_ECC_CTL_BS_512B		(1 << 5)
#define SUNXI_NAND_ECC_CTL_RND_EN		(1 << 9)
#define SUNXI_NAND_ECC_CTL_RND_DIRECTION	(1 << 10)
#define SUNXI_NAND_ECC_CTL_MODE_MASK		(0xf << 12)
#define SUNXI_NAND_ECC_CTL_MODE(a)		((a) << 12)
#define SUNXI_NAND_ECC_CTL_RND_SEED_MASK	(0xffff << 16)
#define SUNXI_NAND_ECC_CTL_RND_SEED(a)		((a) << 16)

extern void sunxi_nand_init(void);

#endif /* _SUNXI_NAND_H */
