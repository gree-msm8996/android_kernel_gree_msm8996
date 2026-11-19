/* Copyright (c) 2026 */
/*
 * G0245D: CM13 (Android 6.0) ISPIF ABI compatibility shim.
 *
 * The stock mm-qcamera-daemon shipped with the G0245D CM13 image was
 * compiled before the Xiaomi camera stack import added a trailing
 * reserved_param word to
 *
 *     struct msm_ispif_param_data
 *
 * which is the largest member of the union embedded in
 *
 *     struct ispif_cfg_data.
 *
 * The resulting layouts are
 *
 *     stock  : cfg_type(4) + { num(4) + entries[10]*36 }        = 368
 *     kernel : cfg_type(4) + { num(4) + entries[10]*36 + rsvd } = 372
 *
 * V4L2 ioctl numbers encode the argument size in bits 16..29, so the
 * two layouts produce different command numbers for the same request:
 *
 *     stock  : _IOWR('V', BASE_VIDIOC_PRIVATE, 368) = 0xc17056c0
 *     kernel : _IOWR('V', BASE_VIDIOC_PRIVATE, 372) = 0xc17456c0
 *
 * The stock daemon therefore issues 0xc17056c0, which does not match
 * VIDIOC_MSM_ISPIF_CFG here, and msm_ispif_subdev_ioctl_unlocked()
 * answers
 *
 *     msm_ispif_subdev_ioctl_unlocked: invalid cmd 0xc17056c0 received
 *
 * The daemon then reports
 *
 *     iface_ispif_hw_reset: 84: iface_ispif_hw_reset: ISPIF_SET_VFE_INFO
 *         kernel return error = -1
 *     iface_ispif_proc_streamon: 369: ispif_hw_reset failed
 *     mct_pipeline_process_set: 2829: STREAM-ON failed ...
 *
 * and the HAL turns that into "Cannot connect to camera".
 *
 * The two layouts agree byte-for-byte through the last entry, and
 * reserved_param is only used internally by the ISPIF driver, so
 * accepting the legacy encoding only requires translating the 368-byte
 * payload into a zeroed 372-byte structure.
 */

#ifndef __MSM_ISPIF_COMPAT_LEGACY368_H__
#define __MSM_ISPIF_COMPAT_LEGACY368_H__

#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/msmb_ispif.h>

/* CM13-era msm_ispif_param_data (no reserved_param tail). */
struct msm_ispif_param_data_legacy368 {
	uint32_t num;
	struct msm_ispif_params_entry entries[MAX_PARAM_ENTRIES];
};

struct ispif_cfg_data_legacy368 {
	enum ispif_cfg_type_t cfg_type;
	union {
		int reg_dump;
		uint32_t csid_version;
		struct msm_ispif_vfe_info vfe_info;
		struct msm_ispif_param_data_legacy368 params;
	};
};

#define VIDIOC_MSM_ISPIF_CFG_LEGACY368 \
	_IOWR('V', BASE_VIDIOC_PRIVATE, struct ispif_cfg_data_legacy368)

#endif /* __MSM_ISPIF_COMPAT_LEGACY368_H__ */
