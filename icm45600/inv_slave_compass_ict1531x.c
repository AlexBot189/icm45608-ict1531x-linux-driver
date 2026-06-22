/*
 * Copyright (C) 2024 InvenSense, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#define pr_fmt(fmt) "inv_mpu: " fmt

#include "inv_slave_compass_ict1531x.h"

/*
 * inv_setup_compass_ict1531x() - Configure ICT1531x magnetometer
 *
 * This function:
 *   1. Configures I2CM hardware (restart_en=1, speed=1)
 *   2. Soft-resets ICT1531x via I2CM
 *   3. Validates WHOAMI (0x45)
 *   4. Reads STATUS_REG to clear I2CM_RDATA state
 *   5. Configures eDMP auto-acquisition:
 *      - DEV_PROFILE0/1 = [STATUS_REG=0x06, device I2C addr=0x1E]
 *      - WR_DATA0..3 = [MODE_CTRL=0x04, SINGLE_SHOT=0x02, MODE_CTRL=0x04, STANDBY=0x00]
 *      - COMMAND_1 = 0x82 (endflag=1, ch=0, rw=write, burst=2)
 */
static int inv_setup_compass_ict1531x(struct inv_mpu_state *st)
{
	int res;
	u8 addr;
	u8 i2cm_ch, i2cm_lenW, i2cm_lenR;
	u8 i2cm_command_x;
	u8 data;
	u8 rd_data;
	int retry;

	addr = st->plat_data.secondary_i2c_addr;
	if (addr == 0)
		addr = ICT1531X_I2C_ADDR;
	pr_info("setup ICT1531x compass, i2c_addr=0x%02x\n", addr);

	/* ---- 1. Configure I2CM hardware ---- */
	res = inv_ireg_read(st, REG_I2CM_CONTROL_IPREG_TOP1, 1, &data);
	if (res)
		return res;
	/* restart_en=1, speed=1 (400kHz) */
	data |= REG_I2CM_CONTROL_I2CM_RESTART_EN_MASK;
	data |= REG_I2CM_CONTROL_I2CM_SPEED_MASK;
	res = inv_ireg_single_write(st, REG_I2CM_CONTROL_IPREG_TOP1, data);
	if (res)
		return res;

	/* ---- 2. Soft-reset ICT1531x through I2CM ---- */
	/* Write 0x01 to MODE_CTRL_REG to trigger reset */
	res = inv_ireg_single_write(st, REG_I2CM_WR_DATA0_IPREG_TOP1,
				    ICT1531X_MODE_CTRL_REG);
	if (res)
		return res;
	res = inv_ireg_single_write(st, REG_I2CM_WR_DATA1_IPREG_TOP1,
				    0x01); /* reset command */
	if (res)
		return res;

	/* Command: endflag=1, ch=0, rw=write(0), burst=2 */
	i2cm_command_x = (1 << REG_I2CM_COMMAND_0_ENDFLAG_0_POS) |
			 (0 << REG_I2CM_COMMAND_0_CH_SEL_0_POS) |
			 (0 << REG_I2CM_COMMAND_0_R_W_0_POS) |
			 (2 << REG_I2CM_COMMAND_0_BURSTLEN_0_POS);
	res = inv_ireg_single_write(st, REG_I2CM_COMMAND_0_IPREG_TOP1,
				    i2cm_command_x);
	if (res)
		return res;

	/* Trigger I2CM transfer by setting GO bit */
	res = inv_ireg_read(st, REG_I2CM_CONTROL_IPREG_TOP1, 1, &data);
	if (res)
		return res;
	data |= REG_I2CM_CONTROL_I2CM_GO_MASK;
	res = inv_ireg_single_write(st, REG_I2CM_CONTROL_IPREG_TOP1, data);
	if (res)
		return res;

	/* Wait for I2CM to finish */
	retry = 0;
	msleep(5);
	do {
		res = inv_ireg_read(st, REG_I2CM_STATUS_IPREG_TOP1, 1, &data);
		if (res)
			return res;
		if (!(data & REG_I2CM_STATUS_I2CM_BUSY_MASK))
			break;
		usleep_range(1000, 2000);
		retry++;
	} while (retry < 100);
	if (retry >= 100) {
		pr_err("ICT1531x: I2CM busy timeout during soft reset\n");
		return -ETIMEDOUT;
	}

	/* Clear GO bit */
	res = inv_ireg_read(st, REG_I2CM_CONTROL_IPREG_TOP1, 1, &data);
	if (res)
		return res;
	data &= ~REG_I2CM_CONTROL_I2CM_GO_MASK;
	res = inv_ireg_single_write(st, REG_I2CM_CONTROL_IPREG_TOP1, data);
	if (res)
		return res;

	msleep(10); /* Wait for ICT1531x reset to complete */

	/* ---- 3. WHOAMI verification ---- */
	/* Read WHOAMI register from ICT1531x */
	i2cm_command_x = (1 << REG_I2CM_COMMAND_0_ENDFLAG_0_POS) |
			 (0 << REG_I2CM_COMMAND_0_CH_SEL_0_POS) |
			 (1 << REG_I2CM_COMMAND_0_R_W_0_POS) | /* read */
			 (1 << REG_I2CM_COMMAND_0_BURSTLEN_0_POS); /* 1 byte */

	/* Write the register address to read (WHOAMI = 0x45... wait, let's use
	 * a generic approach: first write the target register address, then read)
	 *
	 * I2CM auto mode:
	 * DEV_PROFILE0 = target register address to read from
	 * DEV_PROFILE1 = device I2C address
	 * But for single-shot read, we use WR_DATA to set read address, then read from RD_DATA
	 */
	res = inv_ireg_single_write(st, REG_I2CM_WR_DATA0_IPREG_TOP1, 0x00); /* WHOAMI reg on ICT1531x is 0x00 */
	if (res)
		return res;
	res = inv_ireg_single_write(st, REG_I2CM_WR_DATA1_IPREG_TOP1, addr); /* target device addr */
	if (res)
		return res;

	/* Write: burst=2 (reg_addr + device_addr for the write phase) */
	i2cm_command_x = (1 << REG_I2CM_COMMAND_0_ENDFLAG_0_POS) |
			 (0 << REG_I2CM_COMMAND_0_CH_SEL_0_POS) |
			 (0 << REG_I2CM_COMMAND_0_R_W_0_POS) |
			 (2 << REG_I2CM_COMMAND_0_BURSTLEN_0_POS);
	res = inv_ireg_single_write(st, REG_I2CM_COMMAND_0_IPREG_TOP1,
				    i2cm_command_x);
	if (res)
		return res;

	/* Trigger transfer */
	res = inv_ireg_read(st, REG_I2CM_CONTROL_IPREG_TOP1, 1, &data);
	if (res)
		return res;
	data |= REG_I2CM_CONTROL_I2CM_GO_MASK;
	res = inv_ireg_single_write(st, REG_I2CM_CONTROL_IPREG_TOP1, data);
	if (res)
		return res;

	retry = 0;
	do {
		res = inv_ireg_read(st, REG_I2CM_STATUS_IPREG_TOP1, 1, &data);
		if (res)
			return res;
		if (!(data & REG_I2CM_STATUS_I2CM_BUSY_MASK))
			break;
		usleep_range(1000, 2000);
		retry++;
	} while (retry < 100);

	/* Now do a read from RD_DATA0 */
	/* Use command to read 1 byte from I2CM_RD_DATA0 */
	i2cm_command_x = (1 << REG_I2CM_COMMAND_0_ENDFLAG_0_POS) |
			 (0 << REG_I2CM_COMMAND_0_CH_SEL_0_POS) |
			 (1 << REG_I2CM_COMMAND_0_R_W_0_POS) | /* read */
			 (1 << REG_I2CM_COMMAND_0_BURSTLEN_0_POS);
	res = inv_ireg_single_write(st, REG_I2CM_COMMAND_0_IPREG_TOP1,
				    i2cm_command_x);
	if (res)
		return res;

	res = inv_ireg_read(st, REG_I2CM_CONTROL_IPREG_TOP1, 1, &data);
	if (res)
		return res;
	data |= REG_I2CM_CONTROL_I2CM_GO_MASK;
	res = inv_ireg_single_write(st, REG_I2CM_CONTROL_IPREG_TOP1, data);
	if (res)
		return res;

	retry = 0;
	do {
		res = inv_ireg_read(st, REG_I2CM_STATUS_IPREG_TOP1, 1, &data);
		if (res)
			return res;
		if (!(data & REG_I2CM_STATUS_I2CM_BUSY_MASK))
			break;
		usleep_range(1000, 2000);
		retry++;
	} while (retry < 100);

	res = inv_ireg_read(st, REG_I2CM_RD_DATA0_IPREG_TOP1, 1, &rd_data);
	if (res)
		return res;

	pr_info("ICT1531x WHOAMI: read 0x%02x, expected 0x%02x\n",
		rd_data, ICT1531X_WHOAMI);
	if (rd_data != ICT1531X_WHOAMI) {
		pr_err("ICT1531x WHOAMI mismatch: 0x%02x != 0x%02x\n",
		       rd_data, ICT1531X_WHOAMI);
		return -ENODEV;
	}

	/* Clear GO bit */
	res = inv_ireg_read(st, REG_I2CM_CONTROL_IPREG_TOP1, 1, &data);
	if (res)
		return res;
	data &= ~REG_I2CM_CONTROL_I2CM_GO_MASK;
	res = inv_ireg_single_write(st, REG_I2CM_CONTROL_IPREG_TOP1, data);
	if (res)
		return res;

	/* ---- 4. Read STATUS_REG to clear I2CM_RDATA state ---- */
	/* This ensures the next auto-acquisition cycle starts fresh */
	res = inv_ireg_single_write(st, REG_I2CM_WR_DATA0_IPREG_TOP1, ICT1531X_STATUS_REG);
	if (res)
		return res;
	res = inv_ireg_single_write(st, REG_I2CM_WR_DATA1_IPREG_TOP1, addr);
	if (res)
		return res;

	i2cm_command_x = (1 << REG_I2CM_COMMAND_0_ENDFLAG_0_POS) |
			 (0 << REG_I2CM_COMMAND_0_CH_SEL_0_POS) |
			 (0 << REG_I2CM_COMMAND_0_R_W_0_POS) |
			 (2 << REG_I2CM_COMMAND_0_BURSTLEN_0_POS);
	res = inv_ireg_single_write(st, REG_I2CM_COMMAND_0_IPREG_TOP1,
				    i2cm_command_x);
	if (res)
		return res;

	res = inv_ireg_read(st, REG_I2CM_CONTROL_IPREG_TOP1, 1, &data);
	if (res)
		return res;
	data |= REG_I2CM_CONTROL_I2CM_GO_MASK;
	res = inv_ireg_single_write(st, REG_I2CM_CONTROL_IPREG_TOP1, data);
	if (res)
		return res;

	retry = 0;
	do {
		res = inv_ireg_read(st, REG_I2CM_STATUS_IPREG_TOP1, 1, &data);
		if (res)
			return res;
		if (!(data & REG_I2CM_STATUS_I2CM_BUSY_MASK))
			break;
		usleep_range(1000, 2000);
		retry++;
	} while (retry < 100);

	/* Read back to clear */
	i2cm_command_x = (1 << REG_I2CM_COMMAND_0_ENDFLAG_0_POS) |
			 (0 << REG_I2CM_COMMAND_0_CH_SEL_0_POS) |
			 (1 << REG_I2CM_COMMAND_0_R_W_0_POS) | /* read */
			 (1 << REG_I2CM_COMMAND_0_BURSTLEN_0_POS);
	res = inv_ireg_single_write(st, REG_I2CM_COMMAND_0_IPREG_TOP1,
				    i2cm_command_x);
	if (res)
		return res;

	res = inv_ireg_read(st, REG_I2CM_CONTROL_IPREG_TOP1, 1, &data);
	if (res)
		return res;
	data |= REG_I2CM_CONTROL_I2CM_GO_MASK;
	res = inv_ireg_single_write(st, REG_I2CM_CONTROL_IPREG_TOP1, data);
	if (res)
		return res;

	retry = 0;
	do {
		res = inv_ireg_read(st, REG_I2CM_STATUS_IPREG_TOP1, 1, &data);
		if (res)
			return res;
		if (!(data & REG_I2CM_STATUS_I2CM_BUSY_MASK))
			break;
		usleep_range(1000, 2000);
		retry++;
	} while (retry < 100);

	/* Clear GO */
	res = inv_ireg_read(st, REG_I2CM_CONTROL_IPREG_TOP1, 1, &data);
	if (res)
		return res;
	data &= ~REG_I2CM_CONTROL_I2CM_GO_MASK;
	res = inv_ireg_single_write(st, REG_I2CM_CONTROL_IPREG_TOP1, data);
	if (res)
		return res;

	/* ---- 5. Configure eDMP auto-acquisition for ES0 ---- */
	i2cm_ch = 0;
	i2cm_lenW = 2;
	i2cm_lenR = 9; /* 9 bytes: STATUS(1) + X(2) + Y(2) + Z(2) + RESERVED(2) */

	/* Setup profile device ID and read register addresses */
	/* DEV_PROFILE0 = STATUS_REG (0x06): where eDMP starts reading from */
	res = inv_ireg_single_write(st, REG_I2CM_DEV_PROFILE0_IPREG_TOP1,
				    ICT1531X_STATUS_REG);
	if (res)
		return res;
	/* DEV_PROFILE1 = magnetometer I2C address */
	res = inv_ireg_single_write(st, REG_I2CM_DEV_PROFILE1_IPREG_TOP1, addr);
	if (res)
		return res;

	/* Setup to read registers, Command 0 using channel 0 */
	/* Read 9 bytes from STATUS_REG onwards */
	i2cm_command_x = (0 << REG_I2CM_COMMAND_0_ENDFLAG_0_POS) |
			 (i2cm_ch << REG_I2CM_COMMAND_0_CH_SEL_0_POS) |
			 (1 << REG_I2CM_COMMAND_0_R_W_0_POS) |  /* read */
			 (i2cm_lenR << REG_I2CM_COMMAND_0_BURSTLEN_0_POS);
	res = inv_ireg_single_write(st, REG_I2CM_COMMAND_0_IPREG_TOP1,
				    i2cm_command_x);
	if (res)
		return res;

	/* Setup to write registers, Command 1 using channel 0
	 * Write sequence: MODE_CTRL_REG=0x04, SINGLE_SHOT=0x02 (2 bytes)
	 * Then: MODE_CTRL_REG=0x04, STANDBY=0x00 (2 bytes)
	 *
	 * eDMP auto-acquisition: after reading STATUS_REG+data (CMD0),
	 * it executes CMD1 (write SINGLE_SHOT) to trigger next measurement.
	 *
	 * WR_DATA0 = MODE_CTRL_REG (0x04)
	 * WR_DATA1 = SINGLE_SHOT (0x02)
	 *
	 * Per MCU SDK pattern: endflag=1, ch=0, rw=write, burst=2
	 */
	res = inv_ireg_single_write(st, REG_I2CM_WR_DATA0_IPREG_TOP1,
				    ICT1531X_MODE_CTRL_REG);
	if (res)
		return res;
	res = inv_ireg_single_write(st, REG_I2CM_WR_DATA1_IPREG_TOP1,
				    ICT1531X_MODE_CTRL_REG_MODE_SINGLE_SHOT);
	if (res)
		return res;

	/* WR_DATA2/3 for returning to STANDBY after measurement */
	res = inv_ireg_single_write(st, REG_I2CM_WR_DATA2_IPREG_TOP1,
				    ICT1531X_MODE_CTRL_REG);
	if (res)
		return res;
	res = inv_ireg_single_write(st, REG_I2CM_WR_DATA3_IPREG_TOP1,
				    ICT1531X_MODE_CTRL_REG_MODE_STANDBY);
	if (res)
		return res;

	/* Command 1: endflag=1, ch=0, rw=write, burst=2
	 * 0x82 = (1<<7) | (0<<6) | (0<<4) | (2<<0)
	 *       = endflag=1 | ch=0 | rw=write(00) | burst=2
	 */
	i2cm_command_x = (1 << REG_I2CM_COMMAND_1_ENDFLAG_1_POS) |
			 (i2cm_ch << REG_I2CM_COMMAND_1_CH_SEL_1_POS) |
			 (0 << REG_I2CM_COMMAND_1_R_W_1_POS) |
			 (i2cm_lenW << REG_I2CM_COMMAND_1_BURSTLEN_1_POS);
	res = inv_ireg_single_write(st, REG_I2CM_COMMAND_1_IPREG_TOP1,
				    i2cm_command_x);
	if (res)
		return res;

	pr_info("ICT1531x eDMP auto-acquisition configured successfully\n");
	return 0;
}

static int inv_read_ict1531x_scale(struct inv_mpu_state *st, int *scale)
{
	if (COMPASS_ID_ICT1531X == st->plat_data.sec_slave_id) {
		*scale = ICT1531X_SCALE_Q30;
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int inv_suspend_ict1531x(struct inv_mpu_state *st)
{
	return 0;
}

static int inv_resume_ict1531x(struct inv_mpu_state *st)
{
	return 0;
}

static struct inv_mpu_slave slave_ict1531x = {
	.suspend = inv_suspend_ict1531x,
	.resume = inv_resume_ict1531x,
	.get_scale = inv_read_ict1531x_scale,
	.setup = inv_setup_compass_ict1531x,
	.rate_scale = ICT1531X_RATE_SCALE,
	.min_read_time = (9 * NSEC_PER_MSEC),
};
