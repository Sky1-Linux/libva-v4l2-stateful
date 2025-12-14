/*
 * H.264 codec support for VA-API to V4L2 stateful backend
 *
 * For V4L2 stateful decoders, we need to provide Annex-B bitstream
 * including SPS/PPS NAL units. VA-API provides parsed parameters,
 * so we reconstruct the NAL units from VAPictureParameterBufferH264.
 */

#include "vabackend.h"
#include <va/va.h>
#include <string.h>
#include <stdbool.h>

static const uint8_t NAL_START_CODE[] = { 0x00, 0x00, 0x01 };

/*
 * Simple bit writer for generating NAL units
 */
typedef struct {
    uint8_t *data;
    size_t size;
    size_t capacity;
    int bit_pos;
    uint8_t current_byte;
} BitWriter;

static void bw_init(BitWriter *bw, uint8_t *buf, size_t capacity) {
    bw->data = buf;
    bw->size = 0;
    bw->capacity = capacity;
    bw->bit_pos = 0;
    bw->current_byte = 0;
}

static void bw_put_bits(BitWriter *bw, uint32_t val, int bits) {
    for (int i = bits - 1; i >= 0; i--) {
        bw->current_byte = (bw->current_byte << 1) | ((val >> i) & 1);
        bw->bit_pos++;
        if (bw->bit_pos == 8) {
            if (bw->size < bw->capacity)
                bw->data[bw->size++] = bw->current_byte;
            bw->bit_pos = 0;
            bw->current_byte = 0;
        }
    }
}

static void bw_put_ue(BitWriter *bw, uint32_t val) {
    /* Exp-Golomb unsigned encoding */
    val++;
    int bits = 0;
    uint32_t tmp = val;
    while (tmp) { bits++; tmp >>= 1; }
    bw_put_bits(bw, 0, bits - 1);  /* leading zeros */
    bw_put_bits(bw, val, bits);     /* value */
}

static void bw_put_se(BitWriter *bw, int32_t val) {
    /* Exp-Golomb signed encoding */
    if (val <= 0)
        bw_put_ue(bw, (uint32_t)(-val * 2));
    else
        bw_put_ue(bw, (uint32_t)(val * 2 - 1));
}

static size_t bw_finish(BitWriter *bw) {
    if (bw->bit_pos > 0) {
        /* RBSP trailing bits: 1 followed by zeros */
        bw->current_byte = (bw->current_byte << 1) | 1;
        bw->bit_pos++;
        bw->current_byte <<= (8 - bw->bit_pos);
        if (bw->size < bw->capacity)
            bw->data[bw->size++] = bw->current_byte;
    }
    return bw->size;
}

/*
 * Detect H.264 profile from VA-API parameters
 */
static int h264_detect_profile(VAPictureParameterBufferH264 *pic) {
    /* High 10 / High 4:2:2 / High 4:4:4 */
    if (pic->bit_depth_luma_minus8 > 0 || pic->bit_depth_chroma_minus8 > 0) {
        if (pic->seq_fields.bits.chroma_format_idc == 3)
            return 244;  /* High 4:4:4 Predictive */
        if (pic->seq_fields.bits.chroma_format_idc == 2)
            return 122;  /* High 4:2:2 */
        return 110;      /* High 10 */
    }

    /* High profile: 8x8 transform */
    if (pic->pic_fields.bits.transform_8x8_mode_flag)
        return 100;  /* High */

    /* Main profile: CABAC */
    if (pic->pic_fields.bits.entropy_coding_mode_flag)
        return 77;   /* Main */

    /* Baseline/Constrained Baseline */
    return 66;       /* Baseline */
}

/*
 * Calculate H.264 level from resolution and reference frames
 * Based on ITU-T H.264 Table A-1
 */
static int h264_calc_level(VAPictureParameterBufferH264 *pic) {
    int width_mbs = pic->picture_width_in_mbs_minus1 + 1;
    int height_mbs = pic->picture_height_in_mbs_minus1 + 1;
    int total_mbs = width_mbs * height_mbs;
    int max_dpb_mbs = total_mbs * (pic->num_ref_frames + 1);

    /* Level determination based on MaxMBPS and MaxDpbMbs */
    if (max_dpb_mbs <= 396)        return 10;   /* 176x144 */
    if (max_dpb_mbs <= 900)        return 11;   /* 352x288 */
    if (max_dpb_mbs <= 2376)       return 12;   /* 352x288 */
    if (max_dpb_mbs <= 4752)       return 20;   /* 352x288 */
    if (max_dpb_mbs <= 8100)       return 21;   /* 720x480 */
    if (max_dpb_mbs <= 18000)      return 22;   /* 720x480 */
    if (max_dpb_mbs <= 20480)      return 30;   /* 1280x720 */
    if (max_dpb_mbs <= 36864)      return 31;   /* 1280x720 */
    if (max_dpb_mbs <= 32768)      return 32;   /* 1280x1024 */
    if (max_dpb_mbs <= 110400)     return 40;   /* 1920x1080 */
    if (max_dpb_mbs <= 184320)     return 41;   /* 1920x1080 */
    if (max_dpb_mbs <= 184320)     return 42;   /* 1920x1080 */
    if (max_dpb_mbs <= 696320)     return 50;   /* 2560x1920 */
    if (max_dpb_mbs <= 696320)     return 51;   /* 4096x2160 */
    return 52;                                   /* 4096x2160 @ 60fps */
}

/*
 * Generate SPS NAL unit from VAPictureParameterBufferH264
 */
static size_t h264_generate_sps(VAPictureParameterBufferH264 *pic, uint8_t *buf, size_t bufsize) {
    BitWriter bw;
    bw_init(&bw, buf, bufsize);

    int profile_idc = h264_detect_profile(pic);
    int level_idc = h264_calc_level(pic);

    int width_mbs = pic->picture_width_in_mbs_minus1 + 1;
    int height_mbs = pic->picture_height_in_mbs_minus1 + 1;
    int width_pixels = width_mbs * 16;
    int height_pixels = height_mbs * 16;

    /* Check if frame cropping is needed */
    bool need_crop = false;
    int crop_right = 0, crop_bottom = 0;

    /* Common resolutions that need cropping */
    if (width_pixels == 1920 && height_pixels == 1088) {
        need_crop = true;
        crop_bottom = 4;  /* 1088 - 1080 = 8 pixels = 4 in chroma units */
    } else if (width_pixels == 1280 && height_pixels == 736) {
        need_crop = true;
        crop_bottom = 4;  /* 736 - 720 = 16 pixels = 8, /2 = 4 */
    } else if (width_pixels == 640 && height_pixels == 368) {
        need_crop = true;
        crop_bottom = 4;  /* 368 - 360 = 8 pixels = 4 */
    }

    /* NAL header: nal_ref_idc=3, nal_unit_type=7 (SPS) */
    bw_put_bits(&bw, 0x67, 8);

    /* Profile/level */
    bw_put_bits(&bw, profile_idc, 8);

    /* Constraint set flags - be permissive */
    bw_put_bits(&bw, (profile_idc == 66) ? 1 : 0, 1);  /* constraint_set0 (Baseline) */
    bw_put_bits(&bw, (profile_idc <= 77) ? 1 : 0, 1);  /* constraint_set1 (Main compatible) */
    bw_put_bits(&bw, 0, 1);                             /* constraint_set2 */
    bw_put_bits(&bw, 0, 1);                             /* constraint_set3 */
    bw_put_bits(&bw, 0, 1);                             /* constraint_set4 */
    bw_put_bits(&bw, 0, 1);                             /* constraint_set5 */
    bw_put_bits(&bw, 0, 2);                             /* reserved_zero_2bits */
    bw_put_bits(&bw, level_idc, 8);

    bw_put_ue(&bw, 0);  /* seq_parameter_set_id */

    /* High profile extensions */
    if (profile_idc >= 100) {
        bw_put_ue(&bw, pic->seq_fields.bits.chroma_format_idc);
        if (pic->seq_fields.bits.chroma_format_idc == 3)
            bw_put_bits(&bw, 0, 1);  /* separate_colour_plane_flag */
        bw_put_ue(&bw, pic->bit_depth_luma_minus8);
        bw_put_ue(&bw, pic->bit_depth_chroma_minus8);
        bw_put_bits(&bw, 0, 1);      /* qpprime_y_zero_transform_bypass_flag */
        bw_put_bits(&bw, 0, 1);      /* seq_scaling_matrix_present_flag */
    }

    bw_put_ue(&bw, pic->seq_fields.bits.log2_max_frame_num_minus4);
    bw_put_ue(&bw, pic->seq_fields.bits.pic_order_cnt_type);

    if (pic->seq_fields.bits.pic_order_cnt_type == 0) {
        bw_put_ue(&bw, pic->seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4);
    } else if (pic->seq_fields.bits.pic_order_cnt_type == 1) {
        /* POC type 1 - use minimal valid values */
        bw_put_bits(&bw, pic->seq_fields.bits.delta_pic_order_always_zero_flag, 1);
        bw_put_se(&bw, 0);  /* offset_for_non_ref_pic */
        bw_put_se(&bw, 0);  /* offset_for_top_to_bottom_field */
        bw_put_ue(&bw, 0);  /* num_ref_frames_in_pic_order_cnt_cycle */
    }
    /* POC type 2 needs no additional fields */

    bw_put_ue(&bw, pic->num_ref_frames);
    bw_put_bits(&bw, pic->seq_fields.bits.gaps_in_frame_num_value_allowed_flag, 1);
    bw_put_ue(&bw, pic->picture_width_in_mbs_minus1);
    bw_put_ue(&bw, pic->picture_height_in_mbs_minus1);
    bw_put_bits(&bw, pic->seq_fields.bits.frame_mbs_only_flag, 1);

    if (!pic->seq_fields.bits.frame_mbs_only_flag)
        bw_put_bits(&bw, pic->seq_fields.bits.mb_adaptive_frame_field_flag, 1);

    bw_put_bits(&bw, pic->seq_fields.bits.direct_8x8_inference_flag, 1);

    /* Frame cropping */
    bw_put_bits(&bw, need_crop ? 1 : 0, 1);
    if (need_crop) {
        bw_put_ue(&bw, 0);           /* frame_crop_left_offset */
        bw_put_ue(&bw, crop_right);  /* frame_crop_right_offset */
        bw_put_ue(&bw, 0);           /* frame_crop_top_offset */
        bw_put_ue(&bw, crop_bottom); /* frame_crop_bottom_offset */
    }

    bw_put_bits(&bw, 0, 1);  /* vui_parameters_present_flag */

    return bw_finish(&bw);
}

/*
 * Generate PPS NAL unit from VAPictureParameterBufferH264
 */
static size_t h264_generate_pps(VAPictureParameterBufferH264 *pic, uint8_t *buf, size_t bufsize) {
    BitWriter bw;
    bw_init(&bw, buf, bufsize);

    int profile_idc = h264_detect_profile(pic);

    /* NAL header: nal_ref_idc=3, nal_unit_type=8 (PPS) */
    bw_put_bits(&bw, 0x68, 8);

    bw_put_ue(&bw, 0);  /* pic_parameter_set_id */
    bw_put_ue(&bw, 0);  /* seq_parameter_set_id */
    bw_put_bits(&bw, pic->pic_fields.bits.entropy_coding_mode_flag, 1);
    bw_put_bits(&bw, pic->pic_fields.bits.pic_order_present_flag, 1);
    bw_put_ue(&bw, 0);  /* num_slice_groups_minus1 (FMO not supported) */

    /* Use reasonable defaults for ref_idx */
    bw_put_ue(&bw, 0);  /* num_ref_idx_l0_default_active_minus1 */
    bw_put_ue(&bw, 0);  /* num_ref_idx_l1_default_active_minus1 */

    bw_put_bits(&bw, pic->pic_fields.bits.weighted_pred_flag, 1);
    bw_put_bits(&bw, pic->pic_fields.bits.weighted_bipred_idc, 2);
    bw_put_se(&bw, pic->pic_init_qp_minus26);
    bw_put_se(&bw, pic->pic_init_qs_minus26);
    bw_put_se(&bw, pic->chroma_qp_index_offset);
    bw_put_bits(&bw, pic->pic_fields.bits.deblocking_filter_control_present_flag, 1);
    bw_put_bits(&bw, pic->pic_fields.bits.constrained_intra_pred_flag, 1);
    bw_put_bits(&bw, pic->pic_fields.bits.redundant_pic_cnt_present_flag, 1);

    /* High profile extensions */
    if (profile_idc >= 100 && pic->pic_fields.bits.transform_8x8_mode_flag) {
        bw_put_bits(&bw, 1, 1);  /* transform_8x8_mode_flag */
        bw_put_bits(&bw, 0, 1);  /* pic_scaling_matrix_present_flag */
        bw_put_se(&bw, pic->second_chroma_qp_index_offset);
    }

    return bw_finish(&bw);
}

/*
 * Handle H.264 picture parameters - store for SPS/PPS generation
 */
static void h264_handle_picture_params(V4L2Context *ctx, V4L2Buffer *buf)
{
    VAPictureParameterBufferH264 *pic = (VAPictureParameterBufferH264 *)buf->data;

    /* Generate and cache SPS/PPS */
    ctx->h264.last_sps_size = h264_generate_sps(pic, ctx->h264.last_sps, sizeof(ctx->h264.last_sps));
    ctx->h264.last_pps_size = h264_generate_pps(pic, ctx->h264.last_pps, sizeof(ctx->h264.last_pps));

    LOG("H.264: Got picture params: %dx%d MBs, profile=%d, level=%d, refs=%d",
        pic->picture_width_in_mbs_minus1 + 1,
        pic->picture_height_in_mbs_minus1 + 1,
        h264_detect_profile(pic),
        h264_calc_level(pic),
        pic->num_ref_frames);
}

/*
 * Handle H.264 slice data (the actual compressed bitstream)
 */
static void h264_handle_slice_data(V4L2Context *ctx, V4L2Buffer *buf)
{
    VASliceParameterBufferH264 *slice_params = ctx->last_slice_params;

    if (!slice_params) {
        LOG("H.264: No slice params available!");
        return;
    }

    for (unsigned int i = 0; i < ctx->last_slice_count; i++) {
        VASliceParameterBufferH264 *sp = &slice_params[i];
        uint8_t *slice_data = (uint8_t *)buf->data + sp->slice_data_offset;

        /* Check NAL unit type from first byte */
        uint8_t nal_type = slice_data[0] & 0x1f;

        /* For IDR slices (type 5), prepend SPS/PPS */
        if (nal_type == 5 && !ctx->h264.sps_pps_sent) {
            /* Append SPS */
            if (ctx->h264.last_sps_size > 0) {
                bitstream_append(&ctx->bitstream, NAL_START_CODE, sizeof(NAL_START_CODE));
                bitstream_append(&ctx->bitstream, ctx->h264.last_sps, ctx->h264.last_sps_size);
                LOG("H.264: Prepended SPS (%zu bytes)", ctx->h264.last_sps_size);
            }

            /* Append PPS */
            if (ctx->h264.last_pps_size > 0) {
                bitstream_append(&ctx->bitstream, NAL_START_CODE, sizeof(NAL_START_CODE));
                bitstream_append(&ctx->bitstream, ctx->h264.last_pps, ctx->h264.last_pps_size);
                LOG("H.264: Prepended PPS (%zu bytes)", ctx->h264.last_pps_size);
            }

            ctx->h264.sps_pps_sent = true;
        }

        /* Prepend NAL start code and append slice data */
        bitstream_append(&ctx->bitstream, NAL_START_CODE, sizeof(NAL_START_CODE));
        bitstream_append(&ctx->bitstream, slice_data, sp->slice_data_size);
    }
}

/*
 * Prepare bitstream for submission
 */
static void h264_prepare_bitstream(V4L2Context *ctx)
{
    (void)ctx;
}

/* Supported H.264 profiles */
static VAProfile h264_profiles[] = {
    VAProfileH264ConstrainedBaseline,
    VAProfileH264Main,
    VAProfileH264High,
};

/* H.264 codec definition */
const V4L2Codec h264_codec = {
    .name = "H.264",
    .v4l2_pixfmt = V4L2_PIX_FMT_H264,
    .profiles = h264_profiles,
    .num_profiles = sizeof(h264_profiles) / sizeof(h264_profiles[0]),
    .handle_picture_params = h264_handle_picture_params,
    .handle_slice_data = h264_handle_slice_data,
    .prepare_bitstream = h264_prepare_bitstream,
};
