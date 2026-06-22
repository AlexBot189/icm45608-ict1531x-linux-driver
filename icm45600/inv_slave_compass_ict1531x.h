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

#ifndef _INV_SLAVE_COMPASS_ICT1531X_H_
#define _INV_SLAVE_COMPASS_ICT1531X_H_

#include "../inv_mpu_iio.h"

/* ICT1531x magnetometer register definitions */
#define ICT1531X_STATUS_REG                 0x06
#define ICT1531X_MODE_CTRL_REG              0x04
#define ICT1531X_MODE_CTRL_REG_MODE_SINGLE_SHOT    2
#define ICT1531X_MODE_CTRL_REG_MODE_STANDBY        0
#define ICT1531X_WHOAMI                     0x45

/* I2C address of ICT1531x */
#define ICT1531X_I2C_ADDR                   0x1E

/* Scale: 0.075 uT/LSB * (1 << 30) */
#define ICT1531X_SCALE_Q30                  80530636

/* milliseconds between each access */
#define ICT1531X_RATE_SCALE                 10

#endif /* _INV_SLAVE_COMPASS_ICT1531X_H_ */
