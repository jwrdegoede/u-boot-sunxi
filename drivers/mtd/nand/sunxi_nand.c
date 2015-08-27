/*
 * Copyright (C) 2013 Boris BREZILLON <b.brezillon.dev@gmail.com>
 * Copyright (C) 2015 Roy Spliet <r.spliet@ultimaker.com>
 *
 * Derived from:
 *	https://github.com/yuq/sunxi-nfc-mtd
 *	Copyright (C) 2013 Qiang Yu <yuq825@gmail.com>
 *
 *	https://github.com/hno/Allwinner-Info
 *	Copyright (C) 2013 Henrik Nordström <Henrik Nordström>
 *
 *	Copyright (C) 2013 Dmitriy B. <rzk333@gmail.com>
 *	Copyright (C) 2013 Sergey Lapin <slapin@ossfans.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/nand.h>
#include <asm/io.h>
#include <asm/errno.h>

#include <fdtdec.h>
#include <fdt_support.h>
#include <nand.h>
#include <errno.h>
#include <malloc.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>

/* Declare global data pointer */
DECLARE_GLOBAL_DATA_PTR;

#define NAND_MAX_CLOCK (10 * 1000000)
#define DEFAULT_NAME_FORMAT	"nand@%d"
#define MAX_NAME_SIZE		(sizeof("nand@") + 2)

#define SUNXI_NAND_DEFAULT_TIMEOUT_MS	1000

/*
 * Ready/Busy detection type: describes the Ready/Busy detection modes
 *
 * @RB_NONE:	no external detection available, rely on STATUS command
 *		and software timeouts
 * @RB_NATIVE:	use sunxi NAND controller Ready/Busy support. The Ready/Busy
 *		pin of the NAND flash chip must be connected to one of the
 *		native NAND R/B pins (those which can be muxed to the NAND
 *		Controller)
 * @RB_GPIO:	use a simple GPIO to handle Ready/Busy status. The Ready/Busy
 *		pin of the NAND flash chip must be connected to a GPIO capable
 *		pin.
 */
enum sunxi_nand_rb_type {
	RB_NONE,
	RB_NATIVE,
	RB_GPIO,
};

/*
 * Ready/Busy structure: stores informations related to Ready/Busy detection
 *
 * @type:	the Ready/Busy detection mode
 * @info:	information related to the R/B detection mode. Either a gpio
 *		id or a native R/B id (those supported by the NAND controller).
 */
struct sunxi_nand_rb {
	enum sunxi_nand_rb_type type;
	union {
		int gpio;
		int nativeid;
	} info;
};

/*
 * Chip Select structure: stores informations related to NAND Chip Select
 *
 * @cs:		the NAND CS id used to communicate with a NAND Chip
 * @rb:		the Ready/Busy description
 */
struct sunxi_nand_chip_sel {
	u8 cs;
	struct sunxi_nand_rb rb;
};

/*
 * sunxi HW ECC infos: stores informations related to HW ECC support
 *
 * @mode:	the sunxi ECC mode field deduced from ECC requirements
 * @layout:	the OOB layout depending on the ECC requirements and the
 *		selected ECC mode
 */
struct sunxi_nand_hw_ecc {
	int mode;
	struct nand_ecclayout layout;
};

/*
 * sunxi NAND randomizer structure: stores NAND randomizer informations
 *
 * @page: current page
 * @column: current column
 * @nseeds: seed table size
 * @seeds: seed table
 * @subseeds: pre computed sub seeds
 * @step: step function
 * @left: number of remaining bytes in the page
 * @state: current randomizer state
 */
struct sunxi_nand_hw_rnd {
	int page;
	int column;
	int nseeds;
	u16 *seeds;
	u16 *subseeds;
	u16 (*step)(struct mtd_info *mtd, u16 state, int column, int *left);
	int left;
	u16 state;
};

/*
 * NAND chip structure: stores NAND chip device related informations
 *
 * @node:		used to store NAND chips into a list
 * @nand:		base NAND chip structure
 * @mtd:		base MTD structure
 * @default_name:	name used if no name was provided by the DT
 * @clk_rate:		clk_rate required for this NAND chip
 * @selected:		current active CS
 * @nsels:		number of CS lines required by the NAND chip
 * @sels:		array of CS lines descriptions
 */
struct sunxi_nand_chip {
	struct list_head node;
	struct nand_chip nand;
	struct mtd_info *mtd;
	char default_name[MAX_NAME_SIZE];
	void *buffer;
	unsigned long clk_rate;
	int selected;
	int nsels;
	struct sunxi_nand_chip_sel sels[0];
};

static inline struct sunxi_nand_chip *to_sunxi_nand(struct nand_chip *nand)
{
	return container_of(nand, struct sunxi_nand_chip, nand);
}

/*
 * NAND Controller structure: stores sunxi NAND controller informations
 *
 * @controller:		base controller structure
 * @regs:		NAND controller registers
 * @ahb_clk:		NAND Controller AHB clock
 * @mod_clk:		NAND Controller mod clock
 * @assigned_cs:	bitmask describing already assigned CS lines
 * @clk_rate:		NAND controller current clock rate
 * @chips:		a list containing all the NAND chips attached to
 *			this NAND controller
 * @complete:		a completion object used to wait for NAND
 *			controller events
 */
struct sunxi_nfc {
	struct nand_hw_control controller;
	const struct sunxi_nand * regs;
	struct clk *ahb_clk;
	struct clk *mod_clk;
	unsigned long assigned_cs;
	unsigned long clk_rate;
	struct list_head chips;
};

static inline struct sunxi_nfc *to_sunxi_nfc(struct nand_hw_control *ctrl)
{
	return container_of(ctrl, struct sunxi_nfc, controller);
}

static void sunxi_set_clk_rate(unsigned long hz)
{
	struct sunxi_ccm_reg *const ccm =
		(struct sunxi_ccm_reg *)SUNXI_CCM_BASE;

	uint32_t rval = readl(&ccm->pll5_cfg);
	int n = (rval >> 8) & 0x1F;
	int k = ((rval >> 4) & 0x3) + 1;
	int p = (rval >> 16) & 0x3;

	unsigned long clk_rate = 24000000 * n * k >> p;

	unsigned long edo_clk = hz *2;
	int div_n = 0, div_m;

	unsigned long nand_clk_divid_ratio = clk_rate / edo_clk;

	if (clk_rate % edo_clk)
		nand_clk_divid_ratio++;

	for (div_m = nand_clk_divid_ratio; div_m > 16 && div_n < 3; div_n++) {
		if (div_m % 2)
			div_m++;
		div_m >>= 1;
	}
	div_m--;
	if (div_m > 15)
		div_m = 15;	/* Overflow */

	/* config mod clock */
	clrsetbits_le32(&ccm->nand0_clk_cfg, 3 << 24, 2 << 24);      /* 0 = OSC24M, 1 = PLL6, 2 = PLL5 */
	clrsetbits_le32(&ccm->nand0_clk_cfg, 3 << 16, div_n << 16);
	clrsetbits_le32(&ccm->nand0_clk_cfg, 0xf << 0, div_m << 0);

	/*gate on nand clock*/
	setbits_le32(&ccm->ahb_gate0, (1 << AHB_GATE_OFFSET_NAND0));
#ifdef CONFIG_MACH_SUN9I
	setbits_le32(&ccm->ahb_gate1, (1 << AHB_GATE_OFFSET_DMA));
#else
	setbits_le32(&ccm->ahb_gate0, (1 << AHB_GATE_OFFSET_DMA));
#endif
	setbits_le32(&ccm->nand0_clk_cfg, 0x80000000);
}

static int sunxi_nfc_wait_int(struct sunxi_nfc *nfc, u32 flags,
			  unsigned int timeout_ms)
{
       u32 time_start;

	if (!timeout_ms)
		timeout_ms = (CONFIG_SYS_HZ * SUNXI_NAND_DEFAULT_TIMEOUT_MS) / 1000;

	time_start = get_timer(0);

	do {
		if ((readl(&nfc->regs->st) & flags) == flags) {
			setbits_le32(&nfc->regs->st, flags);
			return 0;
		}
	} while (get_timer(time_start) < timeout_ms);

	pr_err("Timeout waiting for interrupt\n");
	return -ETIMEDOUT;
}

static void sunxi_nfc_wait_cmd_fifo_empty(struct sunxi_nfc *nfc)
{
	u32 timeout = (CONFIG_SYS_HZ * SUNXI_NAND_DEFAULT_TIMEOUT_MS) / 1000;
	u32 time_start;

	time_start = get_timer(0);

	do {
		if (!(readl(&nfc->regs->st) & SUNXI_NAND_ST_FIFO_FULL))
			return;
	} while (get_timer(time_start) < timeout);

	pr_err("Timeout waiting for empty fifo\n");
}

static void sunxi_nfc_rst(struct sunxi_nfc *nfc)
{
	u32 timeout = (CONFIG_SYS_HZ * SUNXI_NAND_DEFAULT_TIMEOUT_MS) / 1000;
	u32 time_start;

	time_start = get_timer(0);

	writel(0, &nfc->regs->ecc_ctl);
	writel(SUNXI_NAND_CTL_RST, &nfc->regs->ctl);

	do {
		if (!(readl(&nfc->regs->ctl) & SUNXI_NAND_CTL_RST))
			return;
	} while (get_timer(time_start) < timeout);

	pr_err("Timeout waiting for reset\n");
}

static int sunxi_nfc_dev_ready(struct mtd_info *mtd)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(sunxi_nand->nand.controller);
	struct sunxi_nand_rb *rb;
	unsigned long timeo = (sunxi_nand->nand.state == FL_ERASING ? 400 : 20);
	int ret;

	if (sunxi_nand->selected < 0)
		return 0;

	rb = &sunxi_nand->sels[sunxi_nand->selected].rb;

	switch (rb->type) {
	case RB_NATIVE:
		ret = !!(readl(&nfc->regs->st) &
			 (SUNXI_NAND_ST_RB_STATE0 << rb->info.nativeid));
		if (ret)
			break;

		sunxi_nfc_wait_int(nfc, SUNXI_NAND_ST_RB_B2R, timeo);
		ret = !!(readl(&nfc->regs->st) &
			 (SUNXI_NAND_ST_RB_STATE0 << rb->info.nativeid));
		break;
	case RB_GPIO:
		/*check this*/
		ret = gpio_get_value(rb->info.gpio);
		break;
	case RB_NONE:
	default:
		ret = 0;
		pr_err("cannot check R/B NAND status!");
		break;
	}

	return ret;
}

static void sunxi_nfc_select_chip(struct mtd_info *mtd, int chip)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(sunxi_nand->nand.controller);
	struct sunxi_nand_chip_sel *sel;
	u32 ctl;

	if (chip > 0 && chip >= sunxi_nand->nsels)
		return;

	if (chip == sunxi_nand->selected)
		return;

	ctl = readl(&nfc->regs->ctl) &
	      ~(SUNXI_NAND_CTL_CE_SEL_MASK | SUNXI_NAND_CTL_RB_SEL_MASK |
		SUNXI_NAND_CTL_EN | SUNXI_NAND_CTL_PAGE_SIZE_MASK);

	if (chip >= 0) {
		sel = &sunxi_nand->sels[chip];

		ctl |= SUNXI_NAND_CTL_CE_SEL(sel->cs) | SUNXI_NAND_CTL_EN |
		       (((nand->page_shift - 10) & 0xf) << 8);
		if (sel->rb.type == RB_NONE) {
			nand->dev_ready = NULL;
		} else {
			nand->dev_ready = sunxi_nfc_dev_ready;
			if (sel->rb.type == RB_NATIVE)
				ctl |= SUNXI_NAND_CTL_RB_SEL(sel->rb.info.nativeid);
		}

		writel(mtd->writesize, &nfc->regs->spare_area);

		if (nfc->clk_rate != sunxi_nand->clk_rate) {
			sunxi_set_clk_rate(sunxi_nand->clk_rate);
			nfc->clk_rate = sunxi_nand->clk_rate;
		}
	}

	writel(ctl, &nfc->regs->ctl);

	sunxi_nand->selected = chip;
}

static void sunxi_nfc_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(sunxi_nand->nand.controller);
	int cnt;
	int offs = 0;
	u32 tmp;

	while (len > offs) {
		cnt = len - offs;
		if (cnt > 1024)
			cnt = 1024;

		sunxi_nfc_wait_cmd_fifo_empty(nfc);
		writel(cnt & 0x3ff, &nfc->regs->data_cnt);
		tmp = SUNXI_NAND_CMD_DATA_TRANS | SUNXI_NAND_CMD_DATA_SWAP_METHOD;
		writel(tmp, &nfc->regs->cmd);
		sunxi_nfc_wait_int(nfc, SUNXI_NAND_ST_CMD_INT, 0);
		if (buf)
			memcpy_fromio(buf + offs, &nfc->regs->ram0_base, cnt);
		offs += cnt;
	}
}

static void sunxi_nfc_write_buf(struct mtd_info *mtd, const uint8_t *buf,
				int len)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(sunxi_nand->nand.controller);
	int cnt;
	int offs = 0;
	u32 tmp;

	while (len > offs) {
		cnt = len - offs;
		if (cnt > 1024)
			cnt = 1024;

		sunxi_nfc_wait_cmd_fifo_empty(nfc);
		writel(cnt, &nfc->regs->data_cnt);
		memcpy_toio(&nfc->regs->ram0_base, buf + offs, cnt);
		tmp = SUNXI_NAND_CMD_DATA_TRANS | SUNXI_NAND_CMD_DATA_SWAP_METHOD |
		      SUNXI_NAND_CMD_ACCESS_WR;
		writel(tmp, &nfc->regs->cmd);
		sunxi_nfc_wait_int(nfc, SUNXI_NAND_ST_CMD_INT, 0);
		offs += cnt;
	}
}

static u16 sunxi_nfc_hwrnd_step(struct sunxi_nand_hw_rnd *rnd, u16 state, int count)
{
	state &= 0x7fff;
	count *= 8;
	while (count--)
		state = ((state >> 1) |
			 ((((state >> 0) ^ (state >> 1)) & 1) << 14)) & 0x7fff;

	return state;
}

static u16 sunxi_nfc_hwrnd_single_step(u16 state, int count)
{
	state &= 0x7fff;
	while (count--)
		state = ((state >> 1) |
			 ((((state >> 0) ^ (state >> 1)) & 1) << 14)) & 0x7fff;

	return state;
}

static int sunxi_nfc_hwrnd_config(struct mtd_info *mtd, int page, int column,
				  enum nand_rnd_action action)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nand_hw_rnd *rnd = nand->rnd.priv;
	u16 state;

	if (page < 0 && column < 0) {
		rnd->page = -1;
		rnd->column = -1;
		return 0;
	}

	if (column < 0)
		column = 0;
	if (page < 0)
		page = rnd->page;

	if (page < 0)
		return -EINVAL;

	if (page != rnd->page && action == NAND_RND_READ) {
		int status;

		status = nand_page_get_status(mtd, page);
		if (status == NAND_PAGE_STATUS_UNKNOWN) {
			nand->cmdfunc(mtd, NAND_CMD_RNDOUT, 0, -1);
			sunxi_nfc_read_buf(mtd, sunxi_nand->buffer,
					   mtd->writesize + mtd->oobsize);

			if (nand_page_is_empty(mtd, sunxi_nand->buffer,
					       sunxi_nand->buffer +
					       mtd->writesize))
				status = NAND_PAGE_EMPTY;
			else
				status = NAND_PAGE_FILLED;

			nand_page_set_status(mtd, page, status);
			nand->cmdfunc(mtd, NAND_CMD_RNDOUT, column, -1);
		}
	}

	state = rnd->seeds[page % rnd->nseeds];
	rnd->page = page;
	rnd->column = column;

	if (rnd->step) {
		rnd->state = rnd->step(mtd, state, column, &rnd->left);
	} else {
		rnd->state = sunxi_nfc_hwrnd_step(rnd, state, column % 4096);
		rnd->left = mtd->oobsize + mtd->writesize - column;
	}

	return 0;
}

static void sunxi_nfc_hwrnd_write_buf(struct mtd_info *mtd, const uint8_t *buf,
				      int len)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nfc *nfc = to_sunxi_nfc(nand->controller);
	struct sunxi_nand_hw_rnd *rnd = nand->rnd.priv;
	u32 tmp = readl(&nfc->regs->ecc_ctl);
	int cnt;
	int offs = 0;
	int rndactiv;

	tmp &= ~(SUNXI_NAND_ECC_CTL_RND_DIRECTION |
			SUNXI_NAND_ECC_CTL_RND_SEED_MASK |
			SUNXI_NAND_ECC_CTL_RND_EN);
	writel(tmp, &nfc->regs->ecc_ctl);

	if (rnd->page < 0) {
		sunxi_nfc_write_buf(mtd, buf, len);
		return;
	}

	while (len > offs) {
		cnt = len - offs;
		if (cnt > 1024)
			cnt = 1024;

		rndactiv = nand_rnd_is_activ(mtd, rnd->page, rnd->column,
					     &cnt);
		if (rndactiv > 0) {
			writel(tmp | SUNXI_NAND_ECC_CTL_RND_EN |
					SUNXI_NAND_ECC_CTL_RND_SEED(rnd->state),
			       &nfc->regs->ecc_ctl);
			if (rnd->left < cnt)
				cnt = rnd->left;
		}

		sunxi_nfc_write_buf(mtd, buf + offs, cnt);

		if (rndactiv > 0)
			writel(tmp & ~SUNXI_NAND_ECC_CTL_RND_EN,
			       &nfc->regs->ecc_ctl);

		offs += cnt;
		if (len <= offs)
			break;

		sunxi_nfc_hwrnd_config(mtd, -1, rnd->column + cnt, NAND_RND_WRITE);
	}
}

static void sunxi_nfc_hwrnd_read_buf(struct mtd_info *mtd, uint8_t *buf,
				     int len)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nfc *nfc = to_sunxi_nfc(nand->controller);
	struct sunxi_nand_hw_rnd *rnd = nand->rnd.priv;
	u32 tmp = readl(&nfc->regs->ecc_ctl);
	int cnt;
	int offs = 0;
	int rndactiv;

	tmp &= ~(SUNXI_NAND_ECC_CTL_RND_DIRECTION |
			SUNXI_NAND_ECC_CTL_RND_SEED_MASK |
			SUNXI_NAND_ECC_CTL_RND_EN);
	writel(tmp, &nfc->regs->ecc_ctl);

	if (rnd->page < 0) {
		sunxi_nfc_read_buf(mtd, buf, len);
		return;
	}

	while (len > offs) {
		cnt = len - offs;
		if (cnt > 1024)
			cnt = 1024;

		if (nand_page_get_status(mtd, rnd->page) != NAND_PAGE_EMPTY &&
		    nand_rnd_is_activ(mtd, rnd->page, rnd->column, &cnt) > 0)
			rndactiv = 1;
		else
			rndactiv = 0;

		if (rndactiv > 0) {
			writel(tmp | SUNXI_NAND_ECC_CTL_RND_EN |
					SUNXI_NAND_ECC_CTL_RND_SEED(rnd->state),
			       &nfc->regs->ecc_ctl);
			if (rnd->left < cnt)
				cnt = rnd->left;
		}

		if (buf)
			sunxi_nfc_read_buf(mtd, buf + offs, cnt);
		else
			sunxi_nfc_read_buf(mtd, NULL, cnt);

		if (rndactiv > 0)
			writel(tmp & ~SUNXI_NAND_ECC_CTL_RND_EN,
			       &nfc->regs->ecc_ctl);

		offs += cnt;
		if (len <= offs)
			break;

		sunxi_nfc_hwrnd_config(mtd, -1, rnd->column + cnt, NAND_RND_READ);
	}
}
static uint8_t sunxi_nfc_read_byte(struct mtd_info *mtd)
{
	uint8_t ret;

	sunxi_nfc_read_buf(mtd, &ret, 1);

	return ret;
}

static void sunxi_nfc_cmd_ctrl(struct mtd_info *mtd, int dat,
			       unsigned int ctrl)
{
	struct nand_chip *nand = mtd->priv;
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(nand);
	struct sunxi_nfc *nfc = to_sunxi_nfc(sunxi_nand->nand.controller);
	u32 tmp;

	sunxi_nfc_wait_cmd_fifo_empty(nfc);

	if (ctrl & NAND_CTRL_CHANGE) {
		tmp = readl(&nfc->regs->ctl);
		if (ctrl & NAND_NCE)
			tmp |= SUNXI_NAND_CTL_CE_ACT;
		else
			tmp &= ~SUNXI_NAND_CTL_CE_ACT;
		writel(tmp, &nfc->regs->ctl);
	}

	if (dat == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE) {
		writel(SUNXI_NAND_CMD_SEND_CMD1 | dat, &nfc->regs->cmd);
	} else {
		writel(dat, &nfc->regs->addr_low);
		writel(SUNXI_NAND_CMD_SEND_ADR, &nfc->regs->cmd);
	}

	sunxi_nfc_wait_int(nfc, SUNXI_NAND_ST_CMD_INT, 0);
}

static int sunxi_nfc_hw_ecc_read_page(struct mtd_info *mtd,
				      struct nand_chip *chip, uint8_t *buf,
				      int oob_required, int page)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(chip->controller);
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(chip);
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	struct nand_ecclayout *layout = ecc->layout;
	struct sunxi_nand_hw_ecc *data = ecc->priv;
	unsigned int max_bitflips = 0;
	int status;
	int offset;
	u32 tmp;
	int i;
	int cnt;

	status = nand_page_get_status(mtd, page);
	if (status == NAND_PAGE_STATUS_UNKNOWN) {
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0, -1);
		sunxi_nfc_read_buf(mtd, sunxi_nand->buffer,
				   mtd->writesize + mtd->oobsize);

		if (nand_page_is_empty(mtd, sunxi_nand->buffer,
				       sunxi_nand->buffer +
				       mtd->writesize)) {
			status = NAND_PAGE_EMPTY;
		} else {
			status = NAND_PAGE_FILLED;
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0, -1);
		}

		nand_page_set_status(mtd, page, status);
	}

	if (status == NAND_PAGE_EMPTY) {
		memset(buf, 0xff, mtd->writesize);
		if (oob_required)
			memset(chip->oob_poi, 0xff, mtd->oobsize);
		return 0;
	}

	tmp = readl(&nfc->regs->ecc_ctl);
	tmp &= ~(SUNXI_NAND_ECC_CTL_MODE_MASK | SUNXI_NAND_ECC_CTL_PIPELINE | SUNXI_NAND_ECC_CTL_BS_512B);
	tmp |= SUNXI_NAND_ECC_CTL_ECC_EN | SUNXI_NAND_ECC_CTL_MODE(data->mode) |
	       SUNXI_NAND_ECC_CTL_EXCEPTION;

	writel(tmp, &nfc->regs->ecc_ctl);

	for (i = 0; i < ecc->steps; i++) {
		bool rndactiv = false;

		if (i)
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT, i * ecc->size, -1);

		offset = mtd->writesize + layout->eccpos[i * ecc->bytes] - 4;

		nand_rnd_config(mtd, page, i * ecc->size, NAND_RND_READ);
		nand_rnd_read_buf(mtd, NULL, ecc->size);

		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, offset, -1);
		sunxi_nfc_wait_cmd_fifo_empty(nfc);

		if (i) {
			cnt = ecc->bytes + 4;
			if (nand_rnd_is_activ(mtd, page, offset, &cnt) > 0 &&
			    cnt == ecc->bytes + 4)
				rndactiv = true;
		} else {
			cnt = ecc->bytes + 2;
			if (nand_rnd_is_activ(mtd, page, offset + 2, &cnt) > 0 &&
			    cnt == ecc->bytes + 2)
				rndactiv = true;
		}

		if (rndactiv) {
			tmp = readl(&nfc->regs->ecc_ctl);
			tmp &= ~(SUNXI_NAND_ECC_CTL_RND_DIRECTION | SUNXI_NAND_ECC_CTL_EXCEPTION);
			tmp |= SUNXI_NAND_ECC_CTL_RND_EN;
			writel(tmp, &nfc->regs->ecc_ctl);
		}

		tmp = SUNXI_NAND_CMD_DATA_TRANS | SUNXI_NAND_CMD_DATA_SWAP_METHOD | (1 << 30);
		writel(tmp, &nfc->regs->cmd);

		sunxi_nfc_wait_int(nfc, SUNXI_NAND_ST_CMD_INT, 0);
		memcpy_fromio(buf + (i * ecc->size),
			      &nfc->regs->ram0_base, ecc->size);

		writel(readl(&nfc->regs->ecc_ctl) & ~SUNXI_NAND_ECC_CTL_RND_EN,
		       &nfc->regs->ecc_ctl);

		if (readl(&nfc->regs->ecc_st) & 0x1) {
			mtd->ecc_stats.failed++;
		} else {
			tmp = readl(&nfc->regs->err_cnt[0]) & 0xff;
			mtd->ecc_stats.corrected += tmp;
			max_bitflips = max_t(unsigned int, max_bitflips, tmp);
		}

		if (oob_required) {
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT, offset, -1);
			sunxi_nfc_wait_cmd_fifo_empty(nfc);
			nand_rnd_config(mtd, -1, offset, NAND_RND_READ);
			offset -= mtd->writesize;
			nand_rnd_read_buf(mtd, chip->oob_poi + offset,
					  ecc->bytes + 4);
		}
	}

	if (oob_required) {
		cnt = ecc->layout->oobfree[ecc->steps].length;
		if (cnt > 0) {
			offset = mtd->writesize +
				 ecc->layout->oobfree[ecc->steps].offset;
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT, offset, -1);
			nand_rnd_config(mtd, -1, offset, NAND_RND_READ);
			offset -= mtd->writesize;
			nand_rnd_read_buf(mtd, chip->oob_poi + offset, cnt);
		}
	}

	nand_rnd_config(mtd, -1, -1, NAND_RND_READ);

	tmp = readl(&nfc->regs->ecc_ctl);
	tmp &= ~SUNXI_NAND_ECC_CTL_ECC_EN;

	writel(tmp, &nfc->regs->ecc_ctl);

	return max_bitflips;
}

static int sunxi_nfc_hw_ecc_write_page(struct mtd_info *mtd,
				       struct nand_chip *chip,
				       const uint8_t *buf, int oob_required)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(chip->controller);
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	struct nand_ecclayout *layout = ecc->layout;
	struct sunxi_nand_hw_ecc *data = ecc->priv;
	struct sunxi_nand_hw_rnd *rnd = chip->rnd.priv;
	int offset;
	u32 tmp;
	int i;
	int cnt;

	tmp = readl(&nfc->regs->ecc_ctl);
	tmp &= ~(SUNXI_NAND_ECC_CTL_MODE_MASK | SUNXI_NAND_ECC_CTL_PIPELINE | SUNXI_NAND_ECC_CTL_BS_512B);
	tmp |= SUNXI_NAND_ECC_CTL_ECC_EN | SUNXI_NAND_ECC_CTL_MODE(data->mode) |
	       SUNXI_NAND_ECC_CTL_EXCEPTION;

	writel(tmp, &nfc->regs->ecc_ctl);

	for (i = 0; i < mtd->writesize / ecc->size; i++) {
		bool rndactiv = false;
		u32 user_data;

		if (i)
			chip->cmdfunc(mtd, NAND_CMD_RNDIN, i * ecc->size, -1);

		nand_rnd_config(mtd, -1, i * ecc->size, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, buf + (i * ecc->size), ecc->size);

		offset = layout->eccpos[i * ecc->bytes] - 4 + mtd->writesize;

		/* Fill OOB data in */
		if (!oob_required) {
			user_data = 0xffffffff;
		} else {
			memcpy(&user_data,
			       chip->oob_poi + layout->oobfree[i].offset, 4);
			user_data = le32_to_cpu(user_data);
		}

		if (i) {
			cnt = ecc->bytes + 4;
			if (rnd &&
			    nand_rnd_is_activ(mtd, -1, offset, &cnt) > 0 &&
			    cnt == ecc->bytes + 4)
				rndactiv = true;
		} else {
			cnt = ecc->bytes + 2;
			if (rnd &&
			    nand_rnd_is_activ(mtd, -1, offset + 2, &cnt) > 0 &&
			    cnt == ecc->bytes + 2)
				rndactiv = true;
		}

		if (rndactiv) {
			/* pre randomize to generate FF patterns on the NAND */
			if (!i) {
				u8 oob_tmp[2];
				u16 state = rnd->subseeds[rnd->page % rnd->nseeds];
				oob_tmp[0] = user_data;
				oob_tmp[1] = user_data >> 8;
				state = sunxi_nfc_hwrnd_single_step(state, 15);
				oob_tmp[0] ^= state;
				state = sunxi_nfc_hwrnd_step(rnd, state, 1);
				oob_tmp[1] ^= state;
				user_data &= ~0xffff;
				user_data |= oob_tmp[0] | (oob_tmp[1] << 8);
			}
			tmp = readl(&nfc->regs->ecc_ctl);
			tmp &= ~(SUNXI_NAND_ECC_CTL_RND_DIRECTION | SUNXI_NAND_ECC_CTL_EXCEPTION);
			tmp |= SUNXI_NAND_ECC_CTL_RND_EN;
			writel(tmp, &nfc->regs->ecc_ctl);
		}

		writel(user_data, &nfc->regs->user_data[0]);

		chip->cmdfunc(mtd, NAND_CMD_RNDIN, offset, -1);
		sunxi_nfc_wait_cmd_fifo_empty(nfc);

		tmp = SUNXI_NAND_CMD_DATA_TRANS | SUNXI_NAND_CMD_DATA_SWAP_METHOD | SUNXI_NAND_CMD_ACCESS_WR |
		      (1 << 30);
		writel(tmp, &nfc->regs->cmd);
		sunxi_nfc_wait_int(nfc, SUNXI_NAND_ST_CMD_INT, 0);

		writel(readl(&nfc->regs->ecc_ctl) & ~SUNXI_NAND_ECC_CTL_RND_EN,
		       &nfc->regs->ecc_ctl);
	}

	if (oob_required) {
		cnt = ecc->layout->oobfree[i].length;
		if (cnt > 0) {
			offset = mtd->writesize +
				 ecc->layout->oobfree[i].offset;
			chip->cmdfunc(mtd, NAND_CMD_RNDIN, offset, -1);
			nand_rnd_config(mtd, -1, offset, NAND_RND_WRITE);
			offset -= mtd->writesize;
			nand_rnd_write_buf(mtd, chip->oob_poi + offset, cnt);
		}
	}

	nand_rnd_config(mtd, -1, -1, NAND_RND_WRITE);

	tmp = readl(&nfc->regs->ecc_ctl);
	tmp &= ~SUNXI_NAND_ECC_CTL_ECC_EN;

	writel(tmp, &nfc->regs->ecc_ctl);

	return 0;
}

static u16 sunxi_nfc_hw_ecc_rnd_steps(struct mtd_info *mtd, u16 state,
				      int column, int *left)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	struct sunxi_nand_hw_rnd *rnd = chip->rnd.priv;
	int nblks = mtd->writesize / ecc->size;
	int modsize = ecc->size;
	int steps;

	if (column < mtd->writesize) {
		steps = column % modsize;
		*left = modsize - steps;
	} else if (column < mtd->writesize +
			    (nblks * (ecc->bytes + 4))) {
		column -= mtd->writesize;
		steps = column % (ecc->bytes + 4);
		*left = ecc->bytes + 4 - steps;
		state = rnd->subseeds[rnd->page % rnd->nseeds];
	} else {
		steps = column % 4096;
		*left = mtd->writesize + mtd->oobsize - column;
	}

	return sunxi_nfc_hwrnd_step(rnd, state, steps);
}

static int sunxi_nfc_hw_syndrome_ecc_read_page(struct mtd_info *mtd,
					       struct nand_chip *chip,
					       uint8_t *buf, int oob_required,
					       int page)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(chip->controller);
	struct sunxi_nand_chip *sunxi_nand = to_sunxi_nand(chip);
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	struct sunxi_nand_hw_ecc *data = ecc->priv;
	int steps = mtd->writesize / ecc->size;
	unsigned int max_bitflips = 0;
	uint8_t *oob = chip->oob_poi;
	int offset = 0;
	int status;
	int cnt;
	u32 tmp;
	int i;

	status = nand_page_get_status(mtd, page);
	if (status == NAND_PAGE_STATUS_UNKNOWN) {
		chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0, -1);
		sunxi_nfc_read_buf(mtd, sunxi_nand->buffer,
				   mtd->writesize + mtd->oobsize);

		if (nand_page_is_empty(mtd, sunxi_nand->buffer,
				       sunxi_nand->buffer +
				       mtd->writesize)) {
			status = NAND_PAGE_EMPTY;
		} else {
			status = NAND_PAGE_FILLED;
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT, 0, -1);
		}

		nand_page_set_status(mtd, page, status);
	}

	if (status == NAND_PAGE_EMPTY) {
		memset(buf, 0xff, mtd->writesize);
		if (oob_required)
			memset(chip->oob_poi, 0xff, mtd->oobsize);
		return 0;
	}

	tmp = readl(&nfc->regs->ecc_ctl);
	tmp &= ~(SUNXI_NAND_ECC_CTL_MODE_MASK | SUNXI_NAND_ECC_CTL_PIPELINE | SUNXI_NAND_ECC_CTL_BS_512B);
	tmp |= SUNXI_NAND_ECC_CTL_ECC_EN | SUNXI_NAND_ECC_CTL_MODE(data->mode) |
	       SUNXI_NAND_ECC_CTL_EXCEPTION;

	writel(tmp, &nfc->regs->ecc_ctl);

	for (i = 0; i < steps; i++) {
		nand_rnd_config(mtd, page, offset, NAND_RND_READ);
		nand_rnd_read_buf(mtd, NULL, ecc->size);

		cnt = ecc->bytes + 4;
		if (nand_rnd_is_activ(mtd, page, offset, &cnt) > 0 &&
		    cnt == ecc->bytes + 4) {
			tmp = readl(&nfc->regs->ecc_ctl);
			tmp &= ~(SUNXI_NAND_ECC_CTL_RND_DIRECTION | SUNXI_NAND_ECC_CTL_EXCEPTION);
			tmp |= SUNXI_NAND_ECC_CTL_RND_EN;
			writel(tmp, &nfc->regs->ecc_ctl);
		}

		tmp = SUNXI_NAND_CMD_DATA_TRANS | SUNXI_NAND_CMD_DATA_SWAP_METHOD | (1 << 30);
		writel(tmp, &nfc->regs->cmd);
		sunxi_nfc_wait_int(nfc, SUNXI_NAND_ST_CMD_INT, 0);
		memcpy_fromio(buf, &nfc->regs->ram0_base, ecc->size);
		buf += ecc->size;
		offset += ecc->size;

		writel(readl(&nfc->regs->ecc_ctl) & ~SUNXI_NAND_ECC_CTL_RND_EN,
		       &nfc->regs->ecc_ctl);

		if (readl(&nfc->regs->ecc_st) & 0x1) {
			mtd->ecc_stats.failed++;
		} else {
			tmp = readl(&nfc->regs->err_cnt[0]) & 0xff;
			mtd->ecc_stats.corrected += tmp;
			max_bitflips = max_t(unsigned int, max_bitflips, tmp);
		}

		if (oob_required) {
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT, offset, -1);
			nand_rnd_config(mtd, -1, offset, NAND_RND_READ);
			nand_rnd_read_buf(mtd, oob, ecc->bytes + ecc->prepad);
			oob += ecc->bytes + ecc->prepad;
		}

		offset += ecc->bytes + ecc->prepad;
	}

	if (oob_required) {
		cnt = mtd->oobsize - (oob - chip->oob_poi);
		if (cnt > 0) {
			chip->cmdfunc(mtd, NAND_CMD_RNDOUT, offset, -1);
			nand_rnd_config(mtd, page, offset, NAND_RND_READ);
			nand_rnd_read_buf(mtd, oob, cnt);
		}
	}

	nand_rnd_config(mtd, -1, -1, NAND_RND_READ);

	writel(readl(&nfc->regs->ecc_ctl) & ~SUNXI_NAND_ECC_CTL_ECC_EN,
	       &nfc->regs->ecc_ctl);

	return max_bitflips;
}

static int sunxi_nfc_hw_syndrome_ecc_write_page(struct mtd_info *mtd,
						struct nand_chip *chip,
						const uint8_t *buf,
						int oob_required)
{
	struct sunxi_nfc *nfc = to_sunxi_nfc(chip->controller);
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	struct sunxi_nand_hw_ecc *data = ecc->priv;
	struct sunxi_nand_hw_rnd *rnd = chip->rnd.priv;
	int steps = mtd->writesize / ecc->size;
	uint8_t *oob = chip->oob_poi;
	int offset = 0;
	int cnt;
	u32 tmp;
	int i;

	tmp = readl(&nfc->regs->ecc_ctl);
	tmp &= ~(SUNXI_NAND_ECC_CTL_MODE_MASK | SUNXI_NAND_ECC_CTL_PIPELINE | SUNXI_NAND_ECC_CTL_BS_512B);
	tmp |= SUNXI_NAND_ECC_CTL_ECC_EN | SUNXI_NAND_ECC_CTL_MODE(data->mode) |
	       SUNXI_NAND_ECC_CTL_EXCEPTION;

	writel(tmp, &nfc->regs->ecc_ctl);

	for (i = 0; i < steps; i++) {
		nand_rnd_config(mtd, -1, offset, NAND_RND_WRITE);
		nand_rnd_write_buf(mtd, buf + (i * ecc->size), ecc->size);
		offset += ecc->size;

		/* Fill OOB data in */
		if (oob_required) {
			tmp = 0xffffffff;
		} else {
			memcpy(&tmp, oob, sizeof(tmp));
			tmp = le32_to_cpu(tmp);
		}

		writel(tmp, &nfc->regs->user_data[0]);

		cnt = ecc->bytes + 4;
		if (rnd &&
		    nand_rnd_is_activ(mtd, rnd->page, offset, &cnt) > 0 &&
		    cnt == ecc->bytes + 4) {
			tmp = readl(&nfc->regs->ecc_ctl);
			tmp &= ~(SUNXI_NAND_ECC_CTL_RND_DIRECTION | SUNXI_NAND_ECC_CTL_EXCEPTION);
			tmp |= SUNXI_NAND_ECC_CTL_RND_EN;
			writel(tmp, &nfc->regs->ecc_ctl);
		}

		tmp = SUNXI_NAND_CMD_DATA_TRANS | SUNXI_NAND_CMD_DATA_SWAP_METHOD | SUNXI_NAND_CMD_ACCESS_WR |
		      (1 << 30);
		writel(tmp, &nfc->regs->cmd);
		sunxi_nfc_wait_int(nfc, SUNXI_NAND_ST_CMD_INT, 0);

		writel(readl(&nfc->regs->ecc_ctl) & ~SUNXI_NAND_ECC_CTL_RND_EN,
		       &nfc->regs->ecc_ctl);

		offset += ecc->bytes + ecc->prepad;
		oob += ecc->bytes + ecc->prepad;
	}

	if (oob_required) {
		cnt = mtd->oobsize - (oob - chip->oob_poi);
		if (cnt > 0) {
			chip->cmdfunc(mtd, NAND_CMD_RNDIN, offset, -1);
			nand_rnd_config(mtd, -1, offset, NAND_RND_WRITE);
			nand_rnd_write_buf(mtd, oob, cnt);
		}
	}
	nand_rnd_config(mtd, -1, -1, NAND_RND_WRITE);

	tmp = readl(&nfc->regs->ecc_ctl);
	tmp &= ~SUNXI_NAND_ECC_CTL_ECC_EN;

	writel(tmp, &nfc->regs->ecc_ctl);

	return 0;
}

static u16 sunxi_nfc_hw_syndrome_ecc_rnd_steps(struct mtd_info *mtd, u16 state,
					       int column, int *left)
{
	struct nand_chip *chip = mtd->priv;
	struct nand_ecc_ctrl *ecc = &chip->ecc;
	struct sunxi_nand_hw_rnd *rnd = chip->rnd.priv;
	int eccsteps = mtd->writesize / ecc->size;
	int modsize = ecc->size + ecc->prepad + ecc->bytes;
	int steps;

	if (column < (eccsteps * modsize)) {
		steps = column % modsize;
		*left = modsize - steps;
		if (steps >= ecc->size) {
			steps -= ecc->size;
			state = rnd->subseeds[rnd->page % rnd->nseeds];
		}
	} else {
		steps = column % 4096;
		*left = mtd->writesize + mtd->oobsize - column;
	}

	return sunxi_nfc_hwrnd_step(rnd, state, steps);
}

static u16 default_seeds[] = {0x4a80};
#ifndef __UBOOT__
static void sunxi_nand_rnd_ctrl_cleanup(struct nand_rnd_ctrl *rnd)
{
	struct sunxi_nand_hw_rnd *hwrnd = rnd->priv;

	if (hwrnd->seeds != default_seeds)
		kfree(hwrnd->seeds);
	kfree(hwrnd->subseeds);
	kfree(rnd->layout);
	kfree(hwrnd);
}
#endif

static int sunxi_nand_rnd_ctrl_init(int node, struct mtd_info *mtd,
				    struct nand_rnd_ctrl *rnd,
				    struct nand_ecc_ctrl *ecc)
{
	struct sunxi_nand_hw_rnd *hwrnd;
	struct nand_rnd_layout *layout = NULL;
	int ret;

	hwrnd = kzalloc(sizeof(*hwrnd), GFP_KERNEL);
	if (!hwrnd)
		return -ENOMEM;

	hwrnd->seeds = default_seeds;
	hwrnd->nseeds = ARRAY_SIZE(default_seeds);

	if(fdt_getprop(gd->fdt_blob, node, "nand-randomizer-seeds", &ret)){
		hwrnd->nseeds = ret / sizeof(*hwrnd->seeds);
		hwrnd->seeds = kzalloc(hwrnd->nseeds * sizeof(*hwrnd->seeds),
				       GFP_KERNEL);
		if (!hwrnd->seeds) {
			ret = -ENOMEM;
			goto err;
		}

		ret = fdtdec_get_u16_array(gd->fdt_blob, node, "nand-randomizer-seeds",
						 hwrnd->seeds, hwrnd->nseeds);
		if (ret)
			goto err;
	}

	switch (ecc->mode) {
	case NAND_ECC_HW_SYNDROME:
		hwrnd->step = sunxi_nfc_hw_syndrome_ecc_rnd_steps;
		break;

	case NAND_ECC_HW:
		hwrnd->step = sunxi_nfc_hw_ecc_rnd_steps;

	default:
		layout = kzalloc(sizeof(*layout) + sizeof(struct nand_rndfree),
				 GFP_KERNEL);
		if (!layout) {
			ret = -ENOMEM;
			goto err;
		}
		layout->nranges = 1;
		layout->ranges[0].offset = mtd->writesize;
		layout->ranges[0].length = 2;
		rnd->layout = layout;
		break;
	}

	if (ecc->mode == NAND_ECC_HW_SYNDROME || ecc->mode == NAND_ECC_HW) {
		int i;

		hwrnd->subseeds = kzalloc(hwrnd->nseeds *
					  sizeof(*hwrnd->subseeds),
					  GFP_KERNEL);
		if (!hwrnd->subseeds) {
			ret = -ENOMEM;
			goto err;
		}

		for (i = 0; i < hwrnd->nseeds; i++)
			hwrnd->subseeds[i] = sunxi_nfc_hwrnd_step(hwrnd,
							hwrnd->seeds[i],
							ecc->size);
	}

	rnd->config = sunxi_nfc_hwrnd_config;
	rnd->read_buf = sunxi_nfc_hwrnd_read_buf;
	rnd->write_buf = sunxi_nfc_hwrnd_write_buf;
	rnd->priv = hwrnd;

	return 0;

err:
	kfree(hwrnd);
	kfree(layout);

	return ret;
}

static int sunxi_nand_chip_set_timings(struct sunxi_nand_chip *chip,
				       const struct nand_sdr_timings *timings)
{
	u32 min_clk_period = 0;

	/* T1 <=> tCLS */
	if (timings->tCLS_min > min_clk_period)
		min_clk_period = timings->tCLS_min;

	/* T2 <=> tCLH */
	if (timings->tCLH_min > min_clk_period)
		min_clk_period = timings->tCLH_min;

	/* T3 <=> tCS */
	if (timings->tCS_min > min_clk_period)
		min_clk_period = timings->tCS_min;

	/* T4 <=> tCH */
	if (timings->tCH_min > min_clk_period)
		min_clk_period = timings->tCH_min;

	/* T5 <=> tWP */
	if (timings->tWP_min > min_clk_period)
		min_clk_period = timings->tWP_min;

	/* T6 <=> tWH */
	if (timings->tWH_min > min_clk_period)
		min_clk_period = timings->tWH_min;

	/* T7 <=> tALS */
	if (timings->tALS_min > min_clk_period)
		min_clk_period = timings->tALS_min;

	/* T8 <=> tDS */
	if (timings->tDS_min > min_clk_period)
		min_clk_period = timings->tDS_min;

	/* T9 <=> tDH */
	if (timings->tDH_min > min_clk_period)
		min_clk_period = timings->tDH_min;

	/* T10 <=> tRR */
	if (timings->tRR_min > (min_clk_period * 3))
		min_clk_period = (timings->tRR_min + 2) / 3;

	/* T11 <=> tALH */
	if (timings->tALH_min > min_clk_period)
		min_clk_period = timings->tALH_min;

	/* T12 <=> tRP */
	if (timings->tRP_min > min_clk_period)
		min_clk_period = timings->tRP_min;

	/* T13 <=> tREH */
	if (timings->tREH_min > min_clk_period)
		min_clk_period = timings->tREH_min;

	/* T14 <=> tRC */
	if (timings->tRC_min > (min_clk_period * 2))
		min_clk_period = (timings->tRC_min + 1) / 2;

	/* T15 <=> tWC */
	if (timings->tWC_min > (min_clk_period * 2))
		min_clk_period = (timings->tWC_min + 1) / 2;


	/* min_clk_period = (NAND-clk-period * 2) */
	if (min_clk_period < 1000)
		min_clk_period = 1000;

	min_clk_period /= 1000;
	chip->clk_rate = (2 * 1000000000) / min_clk_period;

	/* TODO: configure T16-T19 */

	return 0;
}

static int sunxi_nand_chip_init_timings(struct sunxi_nand_chip *chip)
{
	const struct nand_sdr_timings *timings;
	int ret;
	int mode;

	mode = onfi_get_async_timing_mode(&chip->nand);
	if (mode == ONFI_TIMING_MODE_UNKNOWN) {
		mode = chip->nand.onfi_timing_mode_default;
	} else {
		uint8_t feature[ONFI_SUBFEATURE_PARAM_LEN] = {};

		mode = fls(mode) - 1;
		if (mode < 0)
			mode = 0;

		feature[0] = mode;
		ret = chip->nand.onfi_set_features(chip->mtd, &chip->nand,
						ONFI_FEATURE_ADDR_TIMING_MODE,
						feature);
		if (ret)
			return ret;
	}

	timings = onfi_async_timing_mode_to_sdr_timings(mode);
	if (IS_ERR(timings))
		return PTR_ERR(timings);

	return sunxi_nand_chip_set_timings(chip, timings);
}

static int sunxi_nand_hw_common_ecc_ctrl_init(struct mtd_info *mtd,
					      struct nand_ecc_ctrl *ecc)
{
	struct sunxi_nand_hw_ecc *data;
	struct nand_ecclayout *layout;
	int nsectors;
	int ret;

	if (!ecc->strength || !ecc->size)
		return -EINVAL;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* Add ECC info retrieval from DT */
	if (ecc->strength <= 16) {
		ecc->strength = 16;
		data->mode = 0;
	} else if (ecc->strength <= 24) {
		ecc->strength = 24;
		data->mode = 1;
	} else if (ecc->strength <= 28) {
		ecc->strength = 28;
		data->mode = 2;
	} else if (ecc->strength <= 32) {
		ecc->strength = 32;
		data->mode = 3;
	} else if (ecc->strength <= 40) {
		ecc->strength = 40;
		data->mode = 4;
	} else if (ecc->strength <= 48) {
		ecc->strength = 48;
		data->mode = 5;
	} else if (ecc->strength <= 56) {
		ecc->strength = 56;
		data->mode = 6;
	} else if (ecc->strength <= 60) {
		ecc->strength = 60;
		data->mode = 7;
	} else if (ecc->strength <= 64) {
		ecc->strength = 64;
		data->mode = 8;
	} else {
		pr_err("unsupported strength\n");
		ret = -ENOTSUPP;
		goto err;
	}

	/* HW ECC always request ECC bytes for 1024 bytes blocks */
	ecc->bytes = ((ecc->strength * fls(8 * 1024)) + 7) / 8;

	/* HW ECC always work with even numbers of ECC bytes */
	if (ecc->bytes % 2)
		ecc->bytes++;

	layout = &data->layout;
	nsectors = mtd->writesize / ecc->size;

	if (mtd->oobsize < ((ecc->bytes + 4) * nsectors)) {
		ret = -EINVAL;
		goto err;
	}

	layout->eccbytes = (ecc->bytes * nsectors);

	ecc->layout = layout;
	ecc->priv = data;

	return 0;

err:
	kfree(data);

	return ret;
}

#ifndef __UBOOT__
static void sunxi_nand_hw_common_ecc_ctrl_cleanup(struct nand_ecc_ctrl *ecc)
{
	kfree(ecc->priv);
}
#endif

static int sunxi_nand_hw_ecc_ctrl_init(struct mtd_info *mtd,
				       struct nand_ecc_ctrl *ecc)
{
	struct nand_ecclayout *layout;
	int nsectors;
	int i, j;
	int ret;

	ret = sunxi_nand_hw_common_ecc_ctrl_init(mtd, ecc);
	if (ret)
		return ret;

	ecc->read_page = sunxi_nfc_hw_ecc_read_page;
	ecc->write_page = sunxi_nfc_hw_ecc_write_page;
	layout = ecc->layout;
	nsectors = mtd->writesize / ecc->size;

	for (i = 0; i < nsectors; i++) {
		if (i) {
			layout->oobfree[i].offset =
				layout->oobfree[i - 1].offset +
				layout->oobfree[i - 1].length +
				ecc->bytes;
			layout->oobfree[i].length = 4;
		} else {
			/*
			 * The first 2 bytes are used for BB markers, hence we
			 * only have 2 bytes available in the first user data
			 * section.
			 */
			layout->oobfree[i].length = 2;
			layout->oobfree[i].offset = 2;
		}

		for (j = 0; j < ecc->bytes; j++)
			layout->eccpos[(ecc->bytes * i) + j] =
					layout->oobfree[i].offset +
					layout->oobfree[i].length + j;
	}

	if (mtd->oobsize > (ecc->bytes + 4) * nsectors) {
		layout->oobfree[nsectors].offset =
				layout->oobfree[nsectors - 1].offset +
				layout->oobfree[nsectors - 1].length +
				ecc->bytes;
		layout->oobfree[nsectors].length = mtd->oobsize -
				((ecc->bytes + 4) * nsectors);
	}

	return 0;
}

static int sunxi_nand_hw_syndrome_ecc_ctrl_init(struct mtd_info *mtd,
						struct nand_ecc_ctrl *ecc)
{
	struct nand_ecclayout *layout;
	int nsectors;
	int i;
	int ret;

	ret = sunxi_nand_hw_common_ecc_ctrl_init(mtd, ecc);
	if (ret)
		return ret;

	ecc->prepad = 4;
	ecc->read_page = sunxi_nfc_hw_syndrome_ecc_read_page;
	ecc->write_page = sunxi_nfc_hw_syndrome_ecc_write_page;

	layout = ecc->layout;
	nsectors = mtd->writesize / ecc->size;

	for (i = 0; i < (ecc->bytes * nsectors); i++)
		layout->eccpos[i] = i;

	layout->oobfree[0].length = mtd->oobsize - i;
	layout->oobfree[0].offset = i;

	return 0;
}

/**
 * It maps 'enum nand_ecc_modes_t' found in include/linux/mtd/nand.h
 * into the device tree binding of 'nand-ecc', so that MTD
 * device driver can get nand ecc from device tree.
 */
static const char *nand_ecc_modes[] = {
	[NAND_ECC_NONE]		= "none",
	[NAND_ECC_SOFT]		= "soft",
	[NAND_ECC_HW]			= "hw",
	[NAND_ECC_HW_SYNDROME]	= "hw_syndrome",
	[NAND_ECC_HW_OOB_FIRST]	= "hw_oob_first",
	[NAND_ECC_SOFT_BCH]		= "soft_bch",
};

/**
 * of_get_nand_ecc_mode - Get nand ecc mode for given device_node
 * @np:	Pointer to the given device_node
 *
 * The function gets ecc mode string from property 'nand-ecc-mode',
 * and return its index in nand_ecc_modes table, or errno in error case.
 */
static inline int of_get_nand_ecc_mode(int node)
{
	const char *pm;
	int len, i;

	pm = fdt_getprop(gd->fdt_blob, node,  "nand-ecc-mode", &len);
	if (!pm)
		return -1;
	for (i = 0; i < ARRAY_SIZE(nand_ecc_modes); i++)
		if (!strcasecmp(pm, nand_ecc_modes[i]))
			return i;
	return -1;
}

 /**
 * It maps 'enum nand_rnd_modes_t' found in include/linux/mtd/nand.h
 * into the device tree binding of 'nand-rnd', so that MTD
 * device driver can get nand rnd from device tree.
 */
static const char *nand_rnd_modes[] = {
	[NAND_RND_NONE]	= "none",
	[NAND_RND_SOFT]	= "soft",
	[NAND_RND_HW]		= "hw",
};

/**
 * of_get_nand_rnd_mode - Get nand randomizer mode for given device_node
 * @np:	Pointer to the given device_node
 *
 * The function gets randomizer mode string from property 'nand-rnd-mode',
 * and return its index in nand_rnd_modes table, or errno in error case.
*/
static inline int of_get_nand_rnd_mode(int node)
{
	const char *pm;
	int len, i;

	pm = fdt_getprop(gd->fdt_blob, node, "nand-rnd-mode", &len);
	if (!pm)
		return -1;
	for (i = 0; i < ARRAY_SIZE(nand_rnd_modes); i++)
		if (!strcasecmp(pm, nand_rnd_modes[i]))
			return i;
	return -1;
}

#ifndef __UBOOT__
static void sunxi_nand_rnd_cleanup(struct nand_rnd_ctrl *rnd)
{
	switch (rnd->mode) {
	case NAND_RND_HW:
		sunxi_nand_rnd_ctrl_cleanup(rnd);
		break;
	default:
		break;
	}
}
#endif

static int sunxi_nand_rnd_init(int node, struct mtd_info *mtd,
			       struct nand_rnd_ctrl *rnd,
			       struct nand_ecc_ctrl *ecc)
{
	int ret;

	rnd->mode = NAND_RND_NONE;

	ret = of_get_nand_rnd_mode(node);
	if (ret >= 0)
		rnd->mode = ret;

	switch (rnd->mode) {
	case NAND_RND_HW:
		return sunxi_nand_rnd_ctrl_init(node, mtd, rnd, ecc);
	default:
		break;
	}

	return 0;
}

#ifndef __UBOOT__
static void sunxi_nand_ecc_cleanup(struct nand_ecc_ctrl *ecc)
{
	switch (ecc->mode) {
	case NAND_ECC_HW:
	case NAND_ECC_HW_SYNDROME:
		sunxi_nand_hw_common_ecc_ctrl_cleanup(ecc);
		break;
	case NAND_ECC_NONE:
		kfree(ecc->layout);
	default:
		break;
	}
}
#endif

static int sunxi_nand_ecc_init(int node, struct mtd_info *mtd,
				struct nand_ecc_ctrl *ecc)
{
	struct nand_chip *nand = mtd->priv;
	s32 strength;
	s32 blk_size;
	int ret;

	blk_size = fdtdec_get_int(gd->fdt_blob, node, "nand-ecc-step-size", -1);
	strength = fdtdec_get_int(gd->fdt_blob, node, "nand-ecc-strength", -1);
	if ((blk_size | strength) > -1){
		ecc->size = blk_size;
		ecc->strength = strength;
	} else {
		ecc->size = nand->ecc_step_ds;
		ecc->strength = nand->ecc_strength_ds;
	}

	printf("strength = %d/%d\n", ecc->strength,ecc->size);
	ecc->mode = NAND_ECC_HW;

	ret = of_get_nand_ecc_mode(node);
	if (ret >= 0)
		ecc->mode = ret;

	switch (ecc->mode) {
	case NAND_ECC_SOFT_BCH:
		if (!ecc->size || !ecc->strength)
			return -EINVAL;
		ecc->bytes = ((ecc->strength * fls(8 * ecc->size)) + 7) / 8;
		break;
	case NAND_ECC_HW:
		ret = sunxi_nand_hw_ecc_ctrl_init(mtd, ecc);
		if (ret)
			return ret;
		break;
	case NAND_ECC_HW_SYNDROME:
		ret = sunxi_nand_hw_syndrome_ecc_ctrl_init(mtd, ecc);
		if (ret)
			return ret;
		break;
	case NAND_ECC_NONE:
		ecc->layout = kzalloc(sizeof(*ecc->layout), GFP_KERNEL);
		if (!ecc->layout)
			return -ENOMEM;
		ecc->layout->oobfree[0].length = mtd->oobsize;
	case NAND_ECC_SOFT:
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int sunxi_nand_chip_init(int node, struct sunxi_nfc *nfc, int devnum)
{
	const struct nand_sdr_timings *timings;
	struct sunxi_nand_chip *chip;
	struct mtd_info *mtd;
	struct nand_chip *nand;
	int nsels;
	int ret;
	int i;
	u32 tmp[8];

	if(!fdt_getprop(gd->fdt_blob, node, "reg", &nsels))
		return -EINVAL;

	nsels /= sizeof(u32);
	if (!nsels)
		return -EINVAL;

	chip = kzalloc(sizeof(*chip) +
			(nsels * sizeof(struct sunxi_nand_chip_sel)),
			GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->nsels = nsels;
	chip->selected = -1;

	for (i = 0; i < nsels; i++) {
		ret = fdtdec_get_int_array(gd->fdt_blob, node, "reg", tmp,
				   nsels);
		if(ret)
			return ret;

		if (tmp[i] > 7)
			return -EINVAL;

		if (test_and_set_bit(tmp[i], &nfc->assigned_cs))
			return -EINVAL;

		chip->sels[i].cs = tmp[i];

		if(fdtdec_get_int_array(gd->fdt_blob, node, "allwinner,rb", tmp,
			nsels) && tmp[i] < 2){
			chip->sels[i].rb.type = RB_NATIVE;
			chip->sels[i].rb.info.nativeid = tmp[i];
		} else {
			chip->sels[i].rb.type = RB_NONE;
		}
	}

	timings = onfi_async_timing_mode_to_sdr_timings(0);
	if (IS_ERR(timings))
		return PTR_ERR(timings);

	nand = &chip->nand;

	nand->chip_delay = 200;
	nand->controller = &nfc->controller;
	nand->select_chip = sunxi_nfc_select_chip;
	nand->cmd_ctrl = sunxi_nfc_cmd_ctrl;
	nand->read_buf = sunxi_nfc_read_buf;
	nand->write_buf = sunxi_nfc_write_buf;
	nand->read_byte = sunxi_nfc_read_byte;

	if(fdtdec_get_bool(gd->fdt_blob, node, "nand-on-flash-bbt"))
		nand->bbt_options |= NAND_BBT_USE_FLASH | NAND_BBT_NO_OOB;

	mtd = &nand_info[devnum];
	chip->mtd = mtd;
	mtd->priv = nand;

	ret = sunxi_nand_chip_set_timings(chip, timings);
	if (ret) {
		pr_err("Error in sunxi_nand_chip_set_timings: %d\n", ret);
		return ret;
	}

	ret = nand_scan_ident(mtd, nsels, NULL);
	if (ret){
		return ret;
	}

	chip->buffer = kzalloc(mtd->writesize + mtd->oobsize, GFP_KERNEL);
	if (!chip->buffer)
		return -ENOMEM;

	ret = sunxi_nand_chip_init_timings(chip);
	if (ret)
		return ret;

	ret = nand_pst_create(mtd);
	if (ret)
		return ret;

	ret = sunxi_nand_ecc_init(node, mtd, &nand->ecc);
	printf("%d\n", ret);
	if (ret)
		return ret;

	ret = sunxi_nand_rnd_init(node, mtd, &nand->rnd, &nand->ecc);
	if (ret)
		return ret;

	ret = nand_scan_tail(mtd);
	if (ret)
		return ret;

	/*if(fdt_getprop(gd->fdt_blob, node, "nand-name", NULL)){
		snprintf(chip->default_name, MAX_NAME_SIZE,
			 DEFAULT_NAME_FORMAT, chip->sels[i].cs);
		mtd->name = chip->default_name;
	}*/

	ret = nand_register(devnum);

	if (ret){
		pr_err("%s:failed to register\n", __func__);
		return ret;
	}

	list_add_tail(&chip->node, &nfc->chips);

	return 0;
}

#ifndef __UBOOT__
static void sunxi_nand_chips_cleanup(struct sunxi_nfc *nfc)
{
	struct sunxi_nand_chip *chip;

	while (!list_empty(&nfc->chips)) {
		chip = list_first_entry(&nfc->chips, struct sunxi_nand_chip,
					node);
		nand_release(&chip->mtd);
		sunxi_nand_ecc_cleanup(&chip->nand.ecc);
		sunxi_nand_rnd_cleanup(&chip->nand.rnd);
		kfree(chip->buffer);
	}
}
#endif /* !__UBOOT__ */

static int sunxi_nand_chips_init(int node, struct sunxi_nfc *nfc)
{
	int ret, offset;
	/*TODO  check maximum chips*/
	for (offset = fdt_first_subnode(gd->fdt_blob, node);
					offset >= 0;
					offset = fdt_next_subnode(gd->fdt_blob, offset)) {
		ret = sunxi_nand_chip_init(offset, nfc, 0 );
		if (ret)
			return ret;
	}

	return 0;
}

static int sunxi_nfc_init(struct sunxi_nfc *nfc)
{
	int node;
	int ret;

	spin_lock_init(&nfc->controller.lock);
	init_waitqueue_head(&nfc->controller.wq);
	INIT_LIST_HEAD(&nfc->chips);

	node = fdtdec_next_compatible(gd->fdt_blob, 0,
				      COMPAT_SUNXI_NAND);
	if (node < 0){
		pr_err("%s:unable to find nfc in device tree\n", __func__);
		goto err;
	}

	if(!fdtdec_get_is_enabled(gd->fdt_blob, node)){
		pr_err("%s:nfc disabled in device tree\n", __func__);
		goto err;
	}

	nfc->regs = (struct sunxi_nand * const)fdtdec_get_addr(gd->fdt_blob,
							node, "reg");
	if ((fdt_addr_t)nfc->regs == FDT_ADDR_T_NONE) {
		pr_err("%s:unable to find nfc address in device tree\n", __func__);
		goto err;
	}

	/* clock enable*/
	sunxi_set_clk_rate(NAND_MAX_CLOCK);
	sunxi_nfc_rst(nfc);

	writel(0, &nfc->regs->intr);

	/*
	 * TODO: replace these magic values with proper flags as soon as we
	 * know what they are encoding.
	 */
	writel(0x100, &nfc->regs->timing_ctl);
	writel(0x7ff, &nfc->regs->timing_cfg);

	ret = sunxi_nand_chips_init(node, nfc);
	if (ret) {
		pr_err("%s:failed to init nand chips\n", __func__);
		goto err;
	}

	return 0;

err:
	kfree(nfc);
	return ret;
}

void sunxi_nand_init(void)
{
	struct sunxi_nfc *nfc;

	nfc = kzalloc(sizeof(*nfc), GFP_KERNEL);
	if (!nfc)
		return;

	sunxi_nfc_init(nfc);
}

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Boris BREZILLON");
MODULE_DESCRIPTION("Allwinner NAND Flash Controller driver");
