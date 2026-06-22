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
 *
 * GAF (Gyroscope-Accelerometer-Fusion) eDMP module for ICM45600.
 *
 * Implements 9-axis sensor fusion using ICM45608's eDMP hardware GAF engine.
 * The ICT1531x magnetometer is read via IMU's internal I2CM and fused on-chip.
 * Output: quaternions in FIFO via ES0/ES1 extended sensor channels.
 */
#define pr_fmt(fmt) "inv_mpu_gaf: " fmt

#include "inv_mpu_edmp_gaf.h"
#include "../inv_mpu_iio.h"
#include <linux/delay.h>

/* --------------------------------------------------------------------------
 * Helper: write u32 to IREG SRAM (little-endian byte order, as eDMP expects)
 * -------------------------------------------------------------------------- */
static int gaf_write_u32(struct inv_mpu_state *st, int addr, u32 val)
{
	u8 buf[4];

	buf[0] = (u8)(val & 0xFF);
	buf[1] = (u8)((val >> 8) & 0xFF);
	buf[2] = (u8)((val >> 16) & 0xFF);
	buf[3] = (u8)((val >> 24) & 0xFF);

	return inv_ireg_write(st, addr, 4, buf);
}

/* --------------------------------------------------------------------------
 * Helper: write s32 array to IREG SRAM (3 elements × 4 bytes = 12 bytes)
 * -------------------------------------------------------------------------- */
static int gaf_write_s32x3(struct inv_mpu_state *st, int addr,
			   const int vals[3])
{
	u8 buf[12];
	int i;

	for (i = 0; i < 3; i++) {
		u32 v = (u32)vals[i];
		buf[i * 4 + 0] = (u8)(v & 0xFF);
		buf[i * 4 + 1] = (u8)((v >> 8) & 0xFF);
		buf[i * 4 + 2] = (u8)((v >> 16) & 0xFF);
		buf[i * 4 + 3] = (u8)((v >> 24) & 0xFF);
	}

	return inv_ireg_write(st, addr, 12, buf);
}

/* --------------------------------------------------------------------------
 * Helper: write 3x3 matrix (9 × s16 in 18 bytes, LE)
 * -------------------------------------------------------------------------- */
static int gaf_write_matrix3x3_s16(struct inv_mpu_state *st, int addr,
				   const int matrix[9])
{
	u8 buf[18];
	int i;

	for (i = 0; i < 9; i++) {
		s16 v = (s16)matrix[i];
		buf[i * 2 + 0] = (u8)(v & 0xFF);
		buf[i * 2 + 1] = (u8)((v >> 8) & 0xFF);
	}

	return inv_ireg_write(st, addr, 18, buf);
}

/* --------------------------------------------------------------------------
 * Helper: write 3x3 matrix (9 × s32 in 36 bytes, LE, Q16 format)
 * -------------------------------------------------------------------------- */
static int gaf_write_matrix3x3_q30(struct inv_mpu_state *st, int addr,
				   const int matrix[9])
{
	u8 buf[36];
	int i;

	for (i = 0; i < 9; i++) {
		u32 v = (u32)matrix[i];
		buf[i * 4 + 0] = (u8)(v & 0xFF);
		buf[i * 4 + 1] = (u8)((v >> 8) & 0xFF);
		buf[i * 4 + 2] = (u8)((v >> 16) & 0xFF);
		buf[i * 4 + 3] = (u8)((v >> 24) & 0xFF);
	}

	return inv_ireg_write(st, addr, 36, buf);
}

/* --------------------------------------------------------------------------
 * inv_mpu_edmp_gaf_set_params - Write GAF operating parameters to SRAM
 * -------------------------------------------------------------------------- */
int inv_mpu_edmp_gaf_set_params(struct inv_mpu_state *st,
				const struct inv_gaf_params *params)
{
	int res;

	if (!params)
		return -EINVAL;

	res = gaf_write_u32(st, EDMP_GAF_CONFIG_ACC_ODR_US, params->acc_odr_us);
	if (res)
		return res;
	res = gaf_write_u32(st, EDMP_GAF_CONFIG_GYR_ODR_US, params->gyr_odr_us);
	if (res)
		return res;
	res = gaf_write_u32(st, EDMP_GAF_CONFIG_ACC_PDR_US, params->acc_pdr_us);
	if (res)
		return res;
	res = gaf_write_u32(st, EDMP_GAF_CONFIG_GYR_PDR_US, params->gyr_pdr_us);
	if (res)
		return res;
	res = gaf_write_u32(st, EDMP_GAF_CONFIG_MAG_DT_US, params->mag_dt_us);
	if (res)
		return res;
	res = gaf_write_u32(st, EDMP_GAF_CONFIG_MAGCAL_DT_US,
			    params->magcal_dt_us);
	if (res)
		return res;
	res = inv_ireg_single_write(st, EDMP_GAF_CONFIG_RUN_SPHERICAL,
				    params->run_spherical ? 1 : 0);
	if (res)
		return res;
	res = inv_ireg_single_write(st, EDMP_GAF_CONFIG_PLL_CLOCK_VARIATION,
				    params->pll_clock_variation);
	if (res)
		return res;

	pr_debug("GAF params set: acc_odr=%u, gyr_odr=%u, "
		 "mag_dt=%u, magcal_dt=%u\n",
		 params->acc_odr_us, params->gyr_odr_us,
		 params->mag_dt_us, params->magcal_dt_us);

	return 0;
}

/* --------------------------------------------------------------------------
 * inv_mpu_edmp_gaf_set_mode - Set GAF mode (gyro_on, mag_on)
 * -------------------------------------------------------------------------- */
int inv_mpu_edmp_gaf_set_mode(struct inv_mpu_state *st,
			      int gyro_on, int mag_on)
{
	u8 mode = 0;

	if (gyro_on)
		mode |= GAF_MODE_GYRO;
	if (mag_on)
		mode |= GAF_MODE_MAG;
	/* FIFO push is enabled by default (0 = push enabled) */
	mode |= GAF_MODE_FIFO_PUSH;

	return inv_ireg_single_write(st, EDMP_GAF_MODE, mode);
}

/* --------------------------------------------------------------------------
 * inv_mpu_edmp_gaf_set_bias - Write saved gyro/accel/mag biases
 * -------------------------------------------------------------------------- */
int inv_mpu_edmp_gaf_set_bias(struct inv_mpu_state *st,
			      const int gyr_bias_q12[3],
			      int gyr_accuracy,
			      int gyr_temp_q16)
{
	int res;

	res = gaf_write_s32x3(st, EDMP_GAF_GYR_BIAS_DPS_Q12, gyr_bias_q12);
	if (res)
		return res;

	/* Write zero accel bias */
	{
		int zero[3] = {0, 0, 0};
		res = gaf_write_s32x3(st, EDMP_GAF_SAVED_ACC_BIAS_1G_Q16, zero);
		if (res)
			return res;
	}

	/* Write zero mag bias */
	{
		int zero[3] = {0, 0, 0};
		res = gaf_write_s32x3(st, EDMP_GAF_SAVED_MAG_BIAS_UT_Q16, zero);
		if (res)
			return res;
	}

	res = gaf_write_u32(st, EDMP_GAF_SAVED_GYR_ACCURACY,
			    (u32)gyr_accuracy);
	if (res)
		return res;
	res = gaf_write_u32(st, EDMP_GAF_SAVED_MAG_ACCURACY, 0);
	if (res)
		return res;
	res = gaf_write_u32(st, EDMP_GAF_GYR_BIAS_TEMPERATURE_DEG_Q16,
			    (u32)gyr_temp_q16);

	return res;
}

/* --------------------------------------------------------------------------
 * inv_mpu_edmp_gaf_set_mounting_matrix - Set global mounting matrix
 * -------------------------------------------------------------------------- */
int inv_mpu_edmp_gaf_set_mounting_matrix(struct inv_mpu_state *st,
					 const int matrix[9])
{
	return gaf_write_matrix3x3_s16(st, EDMP_GLOBAL_MOUNTING_MATRIX, matrix);
}

/* --------------------------------------------------------------------------
 * inv_mpu_edmp_gaf_set_soft_iron_matrix - Set mag soft-iron calibration matrix
 * -------------------------------------------------------------------------- */
int inv_mpu_edmp_gaf_set_soft_iron_matrix(struct inv_mpu_state *st,
					  const int matrix[9])
{
	return gaf_write_matrix3x3_q30(st, EDMP_GAF_CONFIG_MAG_SOFTIRON_MATRIX,
				       matrix);
}

/* --------------------------------------------------------------------------
 * inv_mpu_edmp_gaf_start_fifo_push - Enable GAF FIFO push in GAF_MODE
 *
 * Writes mode with FIFO_PUSH=0 (push enabled), GYRO|MAG enabled.
 * -------------------------------------------------------------------------- */
int inv_mpu_edmp_gaf_start_fifo_push(struct inv_mpu_state *st)
{
	int res;
	u8 mode;

	/* Read current mode, set FIFO_PUSH=0 (enabled), keep gyro/mag */
	res = inv_ireg_read(st, EDMP_GAF_MODE, 1, &mode);
	if (res) {
		/* If GAF_MODE hasn't been set yet, start from defaults */
		mode = GAF_MODE_GYRO | GAF_MODE_MAG;
	}
	/* Ensure FIFO push is enabled (bit3 = 0) */
	mode &= ~0x08;
	res = inv_ireg_single_write(st, EDMP_GAF_MODE, mode);

	pr_debug("GAF FIFO push started, mode=0x%02x\n", mode);
	return res;
}

/* --------------------------------------------------------------------------
 * inv_mpu_edmp_gaf_enable - Enable GAF fusion in EDMP_APEX_EN1
 *
 * Sets GAF_EN (bit7) in REG_EDMP_APEX_EN1_DREG_BANK1 (0x2A).
 * -------------------------------------------------------------------------- */
int inv_mpu_edmp_gaf_enable(struct inv_mpu_state *st)
{
	int res;
	u8 val;

	res = inv_plat_read(st, REG_EDMP_APEX_EN1_DREG_BANK1, 1, &val);
	if (res)
		return res;

	val |= REG_EDMP_APEX_EN1_GAF_EN_MASK;

	res = inv_plat_single_write(st, REG_EDMP_APEX_EN1_DREG_BANK1, val);

	pr_debug("GAF enabled, EDMP_APEX_EN1=0x%02x\n", val);
	return res;
}

/* --------------------------------------------------------------------------
 * inv_mpu_edmp_gaf_disable - Disable GAF fusion
 * -------------------------------------------------------------------------- */
int inv_mpu_edmp_gaf_disable(struct inv_mpu_state *st)
{
	int res;
	u8 val;

	res = inv_plat_read(st, REG_EDMP_APEX_EN1_DREG_BANK1, 1, &val);
	if (res)
		return res;

	val &= ~REG_EDMP_APEX_EN1_GAF_EN_MASK;

	res = inv_plat_single_write(st, REG_EDMP_APEX_EN1_DREG_BANK1, val);

	pr_debug("GAF disabled, EDMP_APEX_EN1=0x%02x\n", val);
	return res;
}

/* --------------------------------------------------------------------------
 * inv_mpu_edmp_mrm_init - Initialize MRM (Magnetic Rate Model)
 *
 * Loads MRM firmware image into eDMP SRAM at RAM_MRM_IMG_DATA_BASE.
 * The firmware image is expected in edmp_ram_mrm_image.h.
 * -------------------------------------------------------------------------- */
int inv_mpu_edmp_mrm_init(struct inv_mpu_state *st)
{
#ifdef CONFIG_INV_MPU_EDMP_MRM_FW
	#include "edmp/edmp_ram_mrm_image.h"

	if (ram_mrm_image_size > 0 && ram_mrm_image != NULL) {
		pr_info("Loading MRM image: %u bytes to 0x%04X\n",
			ram_mrm_image_size, RAM_MRM_IMG_DATA_BASE);
		return inv_ireg_write(st, RAM_MRM_IMG_DATA_BASE,
				      ram_mrm_image_size, ram_mrm_image);
	}
#endif
	/* MRM is optional - GAF can run without it */
	pr_info("MRM firmware not loaded (optional)\n");
	return 0;
}

/* --------------------------------------------------------------------------
 * inv_mpu_edmp_mrm_enable_auto - Enable MRM in auto mode
 *
 * Sets MRM_ENABLE bit in GAF_MODE.
 * -------------------------------------------------------------------------- */
int inv_mpu_edmp_mrm_enable_auto(struct inv_mpu_state *st)
{
	int res;
	u8 mode;

	res = inv_ireg_read(st, EDMP_GAF_MODE, 1, &mode);
	if (res)
		return res;

	mode |= GAF_MODE_MRM_ENABLE;

	res = inv_ireg_single_write(st, EDMP_GAF_MODE, mode);

	pr_debug("MRM auto enabled, GAF_MODE=0x%02x\n", mode);
	return res;
}

/* --------------------------------------------------------------------------
 * inv_mpu_edmp_gaf_decode_fifo - Decode GAF ES0/ES1 FIFO frames
 *
 * ES0 (9 bytes): q1, q2, q3 as s16 LE (quaternion x, y, z in Q14)
 * ES1 (6 bytes): q0, heading_x, heading_y as s16 LE
 *
 * The full quaternion is (q0, q1, q2, q3) in Q14 format.
 * Heading is in Q11 format.
 * -------------------------------------------------------------------------- */
int inv_mpu_edmp_gaf_decode_fifo(const u8 es0[9], const u8 es1[6],
				 struct gaf_fifo_output *output)
{
	if (!es0 || !es1 || !output)
		return -EINVAL;

	/* ES0 contains quaternion x,y,z as s16 LE */
	output->quat_q14[1] = (s16)le16_to_cpup((const __le16 *)&es0[0]); /* qx */
	output->quat_q14[2] = (s16)le16_to_cpup((const __le16 *)&es0[2]); /* qy */
	output->quat_q14[3] = (s16)le16_to_cpup((const __le16 *)&es0[4]); /* qz */

	/* ES1 contains quaternion w, heading x,y as s16 LE */
	output->quat_q14[0] = (s16)le16_to_cpup((const __le16 *)&es1[0]); /* qw */
	output->heading_q11[0] = (s16)le16_to_cpup((const __le16 *)&es1[2]); /* hx */
	output->heading_q11[1] = (s16)le16_to_cpup((const __le16 *)&es1[4]); /* hy */

	/* Check validity: non-zero quaternion indicates valid data */
	if (output->quat_q14[0] != 0 || output->quat_q14[1] != 0 ||
	    output->quat_q14[2] != 0 || output->quat_q14[3] != 0) {
		output->valid = 1;
	} else {
		output->valid = 0;
	}

	/* Initialize other fields to 0 */
	output->gyr_bias_q12[0] = 0;
	output->gyr_bias_q12[1] = 0;
	output->gyr_bias_q12[2] = 0;
	output->raw_mag[0] = 0;
	output->raw_mag[1] = 0;
	output->raw_mag[2] = 0;
	output->accuracy = 0;
	output->stationary = 0;

	pr_debug("GAF quat: qw=%d qx=%d qy=%d qz=%d, heading=%d,%d valid=%d\n",
		 output->quat_q14[0], output->quat_q14[1],
		 output->quat_q14[2], output->quat_q14[3],
		 output->heading_q11[0], output->heading_q11[1],
		 output->valid);

	return 0;
}
