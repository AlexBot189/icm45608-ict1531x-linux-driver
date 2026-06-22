/*
 * edmp_ram_mrm_image.h - MRM (Magnetic Rate Model) Firmware Image
 *
 * This file contains the pre-compiled firmware binary for the eDMP
 * Magnetic Rate Model (MRM) engine running on ICM45608.
 *
 * The firmware image is injected into eDMP SRAM at address
 * RAM_MRM_IMG_DATA_BASE (0x9DC) during initialization.
 *
 * This is a stub file. Replace with the actual firmware binary
 * from the MCU SDK at:
 *   sources/drivers/icm45608/imu/edmp_ram_mrm_image.h
 *
 * To enable, define CONFIG_INV_MPU_EDMP_MRM_FW in your build config
 * and provide the actual firmware.
 *
 * Copyright (C) 2024
 * Licensed under GPL-2.0
 */

#ifndef _EDMP_RAM_MRM_IMAGE_H_
#define _EDMP_RAM_MRM_IMAGE_H_

#ifdef CONFIG_INV_MPU_EDMP_MRM_FW

/*
 * Replace the following with actual MRM firmware data from MCU SDK.
 * Format: array of uint8_t bytes.
 *
 * static const uint8_t ram_mrm_image[] = {
 *     // MRM firmware binary data
 * };
 * static const unsigned int ram_mrm_image_size = 0;
 */

#else /* !CONFIG_INV_MPU_EDMP_MRM_FW */

/* MRM firmware not configured - using empty stub */
static const uint8_t ram_mrm_image[] = { 0 };
static const unsigned int ram_mrm_image_size = 0;

#endif /* CONFIG_INV_MPU_EDMP_MRM_FW */

#endif /* _EDMP_RAM_MRM_IMAGE_H_ */
