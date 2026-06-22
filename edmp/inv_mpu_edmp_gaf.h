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

#ifndef _INV_MPU_EDMP_GAF_H_
#define _INV_MPU_EDMP_GAF_H_

#include <linux/types.h>

/* Forward declaration */
struct inv_mpu_state;

/* ------------------------------------------------------------------------- */
/* GAF Mode bitmasks                                                         */
/* ------------------------------------------------------------------------- */
#define GAF_MODE_MAG            0x01
#define GAF_MODE_GYRO           0x02
#define GAF_MODE_MRM_ENABLE     0x04
#define GAF_MODE_FIFO_PUSH      0x00  /* 0 = push enabled */

/* On-demand dynamic service request register and enable bits */
#define EDMP_ONDEMAND_DYNAMIC_SERVICE_REQUEST      0x004
#define EDMP_ONDEMAND_ENABLE_GAF_PARAM             0x02
#define EDMP_ONDEMAND_ENABLE_GAF_BIAS              0x04

/* ------------------------------------------------------------------------- */
/* GAF SRAM addresses (from MCU SDK edmp_memmap.h)                           */
/* ------------------------------------------------------------------------- */
#define EDMP_GAF_MODE                           0x1BE
#define EDMP_GAF_MODE_SIZE                      1

#define EDMP_GAF_CONFIG_RUN_SPHERICAL           0x2AE
#define EDMP_GAF_CONFIG_RUN_SPHERICAL_SIZE      1

#define EDMP_GAF_CONFIG_ACC_ODR_US              0x288
#define EDMP_GAF_CONFIG_ACC_ODR_US_SIZE         4

#define EDMP_GAF_CONFIG_GYR_ODR_US              0x28C
#define EDMP_GAF_CONFIG_GYR_ODR_US_SIZE         4

#define EDMP_GAF_CONFIG_ACC_PDR_US              0x290
#define EDMP_GAF_CONFIG_ACC_PDR_US_SIZE         4

#define EDMP_GAF_CONFIG_GYR_PDR_US              0x294
#define EDMP_GAF_CONFIG_GYR_PDR_US_SIZE         4

#define EDMP_GAF_CONFIG_MAG_DT_US               0x4A8
#define EDMP_GAF_CONFIG_MAG_DT_US_SIZE          4

#define EDMP_GAF_CONFIG_MAGCAL_DT_US            0x818
#define EDMP_GAF_CONFIG_MAGCAL_DT_US_SIZE       4

#define EDMP_GAF_CONFIG_PLL_CLOCK_VARIATION     0x2AC
#define EDMP_GAF_CONFIG_PLL_CLOCK_VARIATION_SIZE    1

#define EDMP_GAF_CONFIG_MAG_SOFTIRON_MATRIX     0x264
#define EDMP_GAF_CONFIG_MAG_SOFTIRON_MATRIX_SIZE    36

#define EDMP_GLOBAL_MOUNTING_MATRIX             0x1AC
#define EDMP_GLOBAL_MOUNTING_MATRIX_SIZE         18

#define EDMP_GAF_INIT_STATUS                    0x924
#define EDMP_GAF_INIT_STATUS_SIZE                1

#define EDMP_GAF_GYR_BIAS_DPS_Q12               0x6CC
#define EDMP_GAF_GYR_BIAS_DPS_Q12_SIZE           12

#define EDMP_GAF_SAVED_ACC_BIAS_1G_Q16          0x8C8
#define EDMP_GAF_SAVED_ACC_BIAS_1G_Q16_SIZE      12

#define EDMP_GAF_SAVED_MAG_BIAS_UT_Q16          0x774
#define EDMP_GAF_SAVED_MAG_BIAS_UT_Q16_SIZE      12

#define EDMP_GAF_SAVED_GYR_ACCURACY             0x5BC
#define EDMP_GAF_SAVED_GYR_ACCURACY_SIZE         4

#define EDMP_GAF_SAVED_MAG_ACCURACY             0x770
#define EDMP_GAF_SAVED_MAG_ACCURACY_SIZE         4

#define EDMP_GAF_GYR_BIAS_TEMPERATURE_DEG_Q16   0x6A0
#define EDMP_GAF_GYR_BIAS_TEMPERATURE_DEG_Q16_SIZE   4

/* MRM firmware injection address */
#define RAM_MRM_IMG_DATA_BASE                   0x9DC

/* ------------------------------------------------------------------------- */
/* REG_EDMP_APEX_EN1 bitmasks (extensions)                                   */
/* ------------------------------------------------------------------------- */
/* bit7: GAF enable (not in upstream header) */
#define REG_EDMP_APEX_EN1_GAF_EN_POS             0x07
#define REG_EDMP_APEX_EN1_GAF_EN_MASK            (0x01 << REG_EDMP_APEX_EN1_GAF_EN_POS)

/* ------------------------------------------------------------------------- */
/* GAF FIFO output structures                                                */
/* ------------------------------------------------------------------------- */

/**
 * struct gaf_es0_data - 9-byte ES0 output from eDMP FIFO
 * @q0: Quaternion component 0 (s16)
 * @q1: Quaternion component 1 (s16)
 * @q2: Quaternion component 2 (s16)
 * @reserved: Reserved byte
 */
struct gaf_es0_data {
	s16 q0;
	s16 q1;
	s16 q2;
	u8 reserved[3];
} __packed;

/**
 * struct gaf_es1_data - 6-byte ES1 output from eDMP FIFO
 * @qw:   Quaternion component w (s16)
 * @hx:   Heading X component (s16)
 * @hy:   Heading Y component (s16)
 */
struct gaf_es1_data {
	s16 qw;
	s16 hx;
	s16 hy;
} __packed;

/**
 * struct gaf_fifo_output - Decoded GAF FIFO output
 * @quat_q14:     Quaternion in Q14 format (w, x, y, z)
 * @heading_q11:  Heading in Q11 format (x, y)
 * @gyr_bias_q12: Gyro bias in Q12 dps (x, y, z)
 * @raw_mag:      Raw magnetometer data (x, y, z)
 * @accuracy:     Gyro accuracy (0-3)
 * @stationary:   1 if stationary detected
 * @valid:        1 if data is valid
 */
struct gaf_fifo_output {
	s16 quat_q14[4];      /* w, x, y, z */
	s16 heading_q11[2];   /* x, y */
	s16 gyr_bias_q12[3];  /* x, y, z */
	s16 raw_mag[3];       /* x, y, z */
	u8  accuracy;
	u8  stationary;
	u8  valid;
};

/* ------------------------------------------------------------------------- */
/* GAF parameter structure                                                   */
/* ------------------------------------------------------------------------- */
struct inv_gaf_params {
	u32 acc_odr_us;         /* Accelerometer ODR in microseconds */
	u32 gyr_odr_us;         /* Gyroscope ODR in microseconds */
	u32 acc_pdr_us;         /* Accelerometer PDR in microseconds */
	u32 gyr_pdr_us;         /* Gyroscope PDR in microseconds */
	u32 mag_dt_us;          /* Magnetometer delta time in microseconds */
	u32 magcal_dt_us;       /* Mag calibration delta time in microseconds */
	u8  run_spherical;      /* 1 = run spherical correction */
	u8  pll_clock_variation; /* PLL clock variation */
};

/* ------------------------------------------------------------------------- */
/* API Declarations                                                          */
/* ------------------------------------------------------------------------- */

/* Base configuration */
int inv_mpu_edmp_gaf_set_params(struct inv_mpu_state *st,
				const struct inv_gaf_params *params);
int inv_mpu_edmp_gaf_set_mode(struct inv_mpu_state *st,
			      int gyro_on, int mag_on);
int inv_mpu_edmp_gaf_set_bias(struct inv_mpu_state *st,
			      const int gyr_bias_q12[3],
			      int gyr_accuracy,
			      int gyr_temp_q16);
int inv_mpu_edmp_gaf_set_mounting_matrix(struct inv_mpu_state *st,
					 const int matrix[9]);
int inv_mpu_edmp_gaf_set_soft_iron_matrix(struct inv_mpu_state *st,
					  const int matrix[9]);

/* Default GAF parameters */
#define GAF_DEFAULT_ACC_ODR_US      5000
#define GAF_DEFAULT_GYR_ODR_US      5000
#define GAF_DEFAULT_ACC_PDR_US      5000
#define GAF_DEFAULT_GYR_PDR_US      5000
#define GAF_DEFAULT_MAG_DT_US      50000
#define GAF_DEFAULT_MAGCAL_DT_US  100000
#define GAF_DEFAULT_PLL_CLK_VAR     0x22

/* Enable / Disable */
int inv_mpu_edmp_gaf_start_fifo_push(struct inv_mpu_state *st);
int inv_mpu_edmp_gaf_enable(struct inv_mpu_state *st);
int inv_mpu_edmp_gaf_disable(struct inv_mpu_state *st);

/* MRM */
int inv_mpu_edmp_mrm_init(struct inv_mpu_state *st);
int inv_mpu_edmp_mrm_enable_auto(struct inv_mpu_state *st);

/* FIFO decoding */
int inv_mpu_edmp_gaf_decode_fifo(const u8 es0[9], const u8 es1[6],
				 struct gaf_fifo_output *output);

#endif /* _INV_MPU_EDMP_GAF_H_ */
