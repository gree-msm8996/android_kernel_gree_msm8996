/* Copyright (c) 2011-2017, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License version 2 for more details.
 */

/*
 * G0245D: CM13 (Android 6.0) sensor ABI compatibility shim.
 *
 * The stock mm-qcamera-daemon shipped with the G0245D CM13 image is a
 * 32-bit ARM binary that was compiled against a camera SDK whose
 * enum sensor_sub_module_t had only 12 entries.  This tree's
 * include/uapi/media/msm_cam_sensor.h has since grown that enum to
 * 14 entries (SUB_MODULE_LASER_LED was appended).
 *
 * Because struct msm_sensor_info_t embeds two int32_t[12] arrays, the
 * extra entries change the size of every structure that embeds it:
 *
 *     sizeof(struct msm_sensor_info_t)      12 -> 14 entries: 148 -> 164
 *     sizeof(struct sensorb_cfg_data32)                152 -> 168
 *
 * V4L2 ioctl numbers encode the size of their argument structure in
 * bits 16..29, so the stock daemon and this kernel compute *different*
 * ioctl numbers for the very same command:
 *
 *     stock  : _IOWR('V', 192+1, 152) = 0xc09856c1
 *     kernel : _IOWR('V', 192+1, 168) = 0xc0a856c1
 *
 * The stock daemon therefore issues 0xc09856c1, which matches neither
 * VIDIOC_MSM_SENSOR_CFG nor VIDIOC_MSM_SENSOR_CFG32 here, and the
 * kernel answers -ENOIOCTLCMD.  The stock LOG_IOCTL() wrapper collapses
 * every negative ioctl result into -5, which the daemon reports as
 *
 *     sensor_set_stop_stream_settings: 686: failed
 *     sensor_init: 765: failed
 *     module_sensor_offload_init_config: 134: failed: SENSOR_INIT rc:-5
 *
 * i.e. the failure looks like a sensor-init problem even though the
 * ioctl never reached the sensor driver at all.
 *
 * The fix is to accept the legacy encoding explicitly and translate the
 * legacy payload into the current layout, mirroring what is already done
 * for VIDIOC_MSM_SENSOR_INIT_CFG32 (see msm_sensor_init.c).
 */

#ifndef __MSM_CAM_COMPAT_LEGACY12_H__
#define __MSM_CAM_COMPAT_LEGACY12_H__

#include <linux/types.h>
#include <linux/videodev2.h>

/* CM13-era enum sensor_sub_module_t had 12 entries. */
#define MSM_CAM_SUB_MODULE_MAX_LEGACY12 12

struct msm_sensor_info_t_legacy12 {
	char     sensor_name[MAX_SENSOR_NAME];
	uint32_t session_id;
	int32_t  subdev_id[MSM_CAM_SUB_MODULE_MAX_LEGACY12];
	int32_t  subdev_intf[MSM_CAM_SUB_MODULE_MAX_LEGACY12];
	uint8_t  is_mount_angle_valid;
	uint32_t sensor_mount_angle;
	int      modes_supported;
	enum camb_position_t position;
};

/*
 * CM13-era struct sensorb_cfg_data as seen by the 32-bit daemon.
 *
 * Note the union member order and that "setting" is a plain 32-bit
 * pointer for the compat build; the union is sized by the largest
 * member, which is the legacy msm_sensor_info_t (148 bytes).
 */
struct sensorb_cfg_data_legacy12 {
	int cfgtype;
	union {
		struct msm_sensor_info_t_legacy12 sensor_info;
		struct msm_sensor_init_params    sensor_init_params;
		compat_uptr_t                    setting;
		struct msm_sensor_i2c_sync_params sensor_i2c_sync_params;
	} cfg;
};

/*
 * The stock daemon's encoding of VIDIOC_MSM_SENSOR_CFG.  Verified
 * against the literal pool of the shipped libmmcamera2_sensor_modules.so:
 * the word at 0x22dbc is 0xc09856c1, which is _IOWR('V', 193, 152).
 */
#define VIDIOC_MSM_SENSOR_CFG32_LEGACY12 \
	_IOWR('V', BASE_VIDIOC_PRIVATE + 1, \
	      struct sensorb_cfg_data_legacy12)

#endif /* __MSM_CAM_COMPAT_LEGACY12_H__ */
