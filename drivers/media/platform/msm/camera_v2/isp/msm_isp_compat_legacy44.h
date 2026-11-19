/* Copyright (c) 2026 */
/*
 * G0245D: CM13 (Android 6.0) ISP ABI compatibility shim.
 *
 * The stock mm-qcamera-daemon shipped with the G0245D CM13 image was
 * compiled against a camera SDK in which
 *
 *     struct msm_vfe_axi_stream_cfg_cmd
 *
 * ended at sync_frame_id_src (44 bytes after alignment).  This tree
 * carries upstream commit e5fe32061e1b ("msm: camera_v2: fix for older
 * blobs"), which appended a trailing int32_t dummy to reproduce the
 * layout used by a *different* generation of blobs:
 *
 *     stock  : 1 + pad + 8*4 + 4 + 1 + pad        = 44 bytes
 *     kernel : 1 + pad + 8*4 + 4 + 1 + pad + dmy  = 48 bytes
 *
 * V4L2 ioctl numbers encode the argument size in bits 16..29, so the
 * two layouts produce different command numbers for the same request:
 *
 *     stock  : _IOWR('V', MSM_ISP_CFG_STREAM, 44) = 0xc02c56c5
 *     kernel : _IOWR('V', MSM_ISP_CFG_STREAM, 48) = 0xc03056c5
 *
 * The stock daemon therefore issues 0xc02c56c5, which does not match
 * VIDIOC_MSM_ISP_CFG_STREAM here, and msm_isp_ioctl_unlocked() answers
 *
 *     msm_isp_ioctl_unlocked: Invalid ISP command -1070836027
 *
 * The daemon's ISP module then reports
 *
 *     isp_resource_update_module_cfg: 284: failed: ...
 *     port_sensor_handle_stream_on: ... failed evaluated expression is FALSE
 *     mct_pipeline_process_set: 2829: STREAM-ON failed ...
 *
 * and the HAL turns that into "Cannot connect to camera".
 *
 * The two layouts agree byte-for-byte through sync_frame_id_src, and the
 * dummy field is never read by the kernel, so accepting the legacy
 * encoding only requires translating the 44-byte payload into a zeroed
 * 48-byte structure before calling msm_isp_cfg_axi_stream().
 */

#ifndef __MSM_ISP_COMPAT_LEGACY44_H__
#define __MSM_ISP_COMPAT_LEGACY44_H__

#include <linux/types.h>
#include <linux/videodev2.h>
#include <media/msmb_isp.h>

/* CM13-era msm_vfe_axi_stream_cfg_cmd (44 bytes, no dummy tail). */
struct msm_vfe_axi_stream_cfg_cmd_legacy44 {
	uint8_t num_streams;
	uint32_t stream_handle[VFE_AXI_SRC_MAX];
	enum msm_vfe_axi_stream_cmd cmd;
	uint8_t sync_frame_id_src;
};

#define VIDIOC_MSM_ISP_CFG_STREAM_LEGACY44 \
	_IOWR('V', MSM_ISP_CFG_STREAM, \
	      struct msm_vfe_axi_stream_cfg_cmd_legacy44)

#endif /* __MSM_ISP_COMPAT_LEGACY44_H__ */
