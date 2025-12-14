/*
 * HEVC (H.265) codec support for VA-API to V4L2 stateful backend
 *
 * For V4L2 stateful decoders, we need to provide Annex-B bitstream
 * including VPS/SPS/PPS NAL units. VA-API provides parsed parameters,
 * so we reconstruct the NAL units from VAPictureParameterBufferHEVC.
 */

#include "vabackend.h"
#include <va/va.h>
#include <va/va_dec_hevc.h>
#include <string.h>
#include <stdbool.h>

static const uint8_t NAL_START_CODE[] = { 0x00, 0x00, 0x01 };

/* HEVC NAL unit types */
#define HEVC_NAL_IDR_W_RADL     19
#define HEVC_NAL_IDR_N_LP       20
#define HEVC_NAL_CRA_NUT        21
#define HEVC_NAL_VPS            32
#define HEVC_NAL_SPS            33
#define HEVC_NAL_PPS            34

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
 * NAL unit scanner for extracting original VPS/SPS/PPS from bitstream
 *
 * When decoding MPEG-TS or raw Annex-B streams, the original parameter sets
 * are present in the slice data. We extract and cache them to use instead
 * of reconstructed ones, which may have bit-level differences causing decode
 * failures (especially for B-frame streams).
 */

typedef struct {
    const uint8_t *data;    /* Pointer to NAL data (after start code) */
    size_t size;            /* Size of NAL unit */
    uint8_t nal_type;       /* HEVC NAL unit type (0-63) */
} HevcNalUnit;

typedef struct {
    HevcNalUnit vps;
    HevcNalUnit sps;
    HevcNalUnit pps;
    bool has_vps;
    bool has_sps;
    bool has_pps;
} HevcParameterSets;

/*
 * Find next NAL start code in buffer
 * Returns pointer to first byte after start code, or NULL if not found
 */
static const uint8_t *find_start_code(const uint8_t *data, size_t size,
                                       const uint8_t **start_code_pos) {
    for (size_t i = 0; i + 2 < size; i++) {
        if (data[i] == 0 && data[i+1] == 0) {
            if (data[i+2] == 1) {
                /* 3-byte start code: 00 00 01 */
                if (start_code_pos)
                    *start_code_pos = &data[i];
                return &data[i+3];
            }
            if (i + 3 < size && data[i+2] == 0 && data[i+3] == 1) {
                /* 4-byte start code: 00 00 00 01 */
                if (start_code_pos)
                    *start_code_pos = &data[i];
                return &data[i+4];
            }
        }
    }
    return NULL;
}

/*
 * Scan buffer for HEVC NAL units and extract VPS/SPS/PPS if found
 * Returns number of parameter sets found (0-3)
 */
static int hevc_scan_for_parameter_sets(const uint8_t *data, size_t size,
                                         HevcParameterSets *params) {
    memset(params, 0, sizeof(*params));

    const uint8_t *p = data;
    const uint8_t *end = data + size;
    const uint8_t *start_code_pos = NULL;
    int found = 0;

    /* Find first start code */
    p = find_start_code(p, end - p, &start_code_pos);
    if (!p)
        return 0;

    while (p && p + 2 <= end) {
        /* Get NAL type from header (byte0: forbidden(1) | type(6) | layer_high(1)) */
        uint8_t nal_type = (p[0] >> 1) & 0x3f;

        /* Find next start code to determine NAL unit size */
        const uint8_t *next_start = NULL;
        const uint8_t *next_p = find_start_code(p, end - p, &next_start);

        size_t nal_size;
        if (next_start) {
            nal_size = next_start - p;
        } else {
            nal_size = end - p;
        }

        /* Check for parameter sets */
        switch (nal_type) {
        case HEVC_NAL_VPS:
            if (!params->has_vps && nal_size <= 64) {
                params->vps.data = p;
                params->vps.size = nal_size;
                params->vps.nal_type = nal_type;
                params->has_vps = true;
                found++;
            }
            break;
        case HEVC_NAL_SPS:
            if (!params->has_sps && nal_size <= 256) {
                params->sps.data = p;
                params->sps.size = nal_size;
                params->sps.nal_type = nal_type;
                params->has_sps = true;
                found++;
            }
            break;
        case HEVC_NAL_PPS:
            if (!params->has_pps && nal_size <= 128) {
                params->pps.data = p;
                params->pps.size = nal_size;
                params->pps.nal_type = nal_type;
                params->has_pps = true;
                found++;
            }
            break;
        default:
            /* Not a parameter set NAL */
            break;
        }

        /* Move to next NAL */
        p = next_p;
    }

    return found;
}

/*
 * Write HEVC NAL unit header (2 bytes)
 * Format: forbidden_zero_bit (1) | nal_unit_type (6) | nuh_layer_id (6) | nuh_temporal_id_plus1 (3)
 */
static void hevc_write_nal_header(BitWriter *bw, int nal_type) {
    bw_put_bits(bw, 0, 1);         /* forbidden_zero_bit */
    bw_put_bits(bw, nal_type, 6);  /* nal_unit_type */
    bw_put_bits(bw, 0, 6);         /* nuh_layer_id */
    bw_put_bits(bw, 1, 3);         /* nuh_temporal_id_plus1 */
}

/* Color primaries constants (ITU-T H.273) */
#define COLOR_PRIMARIES_BT709       1
#define COLOR_PRIMARIES_BT2020      9

/* Transfer characteristics constants */
#define TRANSFER_BT709              1
#define TRANSFER_PQ                 16  /* SMPTE ST 2084 (HDR10) */
#define TRANSFER_HLG                18  /* ARIB STD-B67 (HLG) */

/* Matrix coefficients constants */
#define MATRIX_BT709                1
#define MATRIX_BT2020_NCL           9   /* BT.2020 non-constant luminance */

/*
 * Generate VUI parameters for SPS
 * For Main10 profile, assume BT.2020/PQ (HDR10) color space
 * For Main profile, assume BT.709 (SDR)
 */
static void hevc_write_vui(BitWriter *bw, VAPictureParameterBufferHEVC *pic) {
    bool is_hdr = (pic->bit_depth_luma_minus8 > 0);  /* Main10 likely HDR */

    /* aspect_ratio_info_present_flag */
    bw_put_bits(bw, 0, 1);

    /* overscan_info_present_flag */
    bw_put_bits(bw, 0, 1);

    /* video_signal_type_present_flag */
    bw_put_bits(bw, 1, 1);
    {
        /* video_format (5 = unspecified) */
        bw_put_bits(bw, 5, 3);
        /* video_full_range_flag (0 = limited range) */
        bw_put_bits(bw, 0, 1);
        /* colour_description_present_flag */
        bw_put_bits(bw, 1, 1);
        {
            if (is_hdr) {
                /* BT.2020 color space with PQ transfer for HDR10 */
                bw_put_bits(bw, COLOR_PRIMARIES_BT2020, 8);  /* colour_primaries */
                bw_put_bits(bw, TRANSFER_PQ, 8);            /* transfer_characteristics */
                bw_put_bits(bw, MATRIX_BT2020_NCL, 8);      /* matrix_coeffs */
            } else {
                /* BT.709 color space for SDR */
                bw_put_bits(bw, COLOR_PRIMARIES_BT709, 8);  /* colour_primaries */
                bw_put_bits(bw, TRANSFER_BT709, 8);         /* transfer_characteristics */
                bw_put_bits(bw, MATRIX_BT709, 8);           /* matrix_coeffs */
            }
        }
    }

    /* chroma_loc_info_present_flag */
    bw_put_bits(bw, 0, 1);

    /* neutral_chroma_indication_flag */
    bw_put_bits(bw, 0, 1);

    /* field_seq_flag */
    bw_put_bits(bw, 0, 1);

    /* frame_field_info_present_flag */
    bw_put_bits(bw, 0, 1);

    /* default_display_window_flag */
    bw_put_bits(bw, 0, 1);

    /* vui_timing_info_present_flag */
    bw_put_bits(bw, 0, 1);

    /* bitstream_restriction_flag */
    bw_put_bits(bw, 0, 1);
}

/*
 * Detect HEVC tier/level from resolution
 * Based on ITU-T H.265 Table A.6
 */
static int hevc_calc_level(VAPictureParameterBufferHEVC *pic) {
    int width = pic->pic_width_in_luma_samples;
    int height = pic->pic_height_in_luma_samples;
    int pixels = width * height;

    /* Level * 30 (so 3.0 = 90, 4.0 = 120, etc.) */
    if (pixels <= 36864)        return 30;   /* Level 1 */
    if (pixels <= 122880)       return 60;   /* Level 2 */
    if (pixels <= 245760)       return 63;   /* Level 2.1 */
    if (pixels <= 552960)       return 90;   /* Level 3 */
    if (pixels <= 983040)       return 93;   /* Level 3.1 */
    if (pixels <= 2228224)      return 120;  /* Level 4 */
    if (pixels <= 2228224)      return 123;  /* Level 4.1 */
    if (pixels <= 8912896)      return 150;  /* Level 5 */
    if (pixels <= 8912896)      return 153;  /* Level 5.1 */
    if (pixels <= 8912896)      return 156;  /* Level 5.2 */
    if (pixels <= 35651584)     return 180;  /* Level 6 */
    if (pixels <= 35651584)     return 183;  /* Level 6.1 */
    return 186;                              /* Level 6.2 */
}

/*
 * Detect HEVC tier (0=Main, 1=High) based on resolution and level
 * High tier allows higher bitrates at the same level
 */
static int hevc_calc_tier(VAPictureParameterBufferHEVC *pic, int level_idc) {
    int pixels = pic->pic_width_in_luma_samples * pic->pic_height_in_luma_samples;

    /* Use High tier for 4K+ content at Level 5.0 or higher
     * This allows higher bitrates which are common in 4K HDR content
     */
    if (level_idc >= 150 && pixels >= 8294400) {  /* Level 5.0+, ~4K */
        return 1;  /* High tier */
    }
    return 0;  /* Main tier */
}

/*
 * Generate VPS NAL unit (minimal valid VPS)
 */
static size_t hevc_generate_vps(VAPictureParameterBufferHEVC *pic, uint8_t *buf, size_t bufsize) {
    BitWriter bw;
    bw_init(&bw, buf, bufsize);

    int level_idc = hevc_calc_level(pic);
    int tier = hevc_calc_tier(pic, level_idc);

    /* NAL header */
    hevc_write_nal_header(&bw, HEVC_NAL_VPS);

    /* vps_video_parameter_set_id */
    bw_put_bits(&bw, 0, 4);
    /* vps_base_layer_internal_flag */
    bw_put_bits(&bw, 1, 1);
    /* vps_base_layer_available_flag */
    bw_put_bits(&bw, 1, 1);
    /* vps_max_layers_minus1 */
    bw_put_bits(&bw, 0, 6);
    /* vps_max_sub_layers_minus1 */
    bw_put_bits(&bw, 0, 3);
    /* vps_temporal_id_nesting_flag */
    bw_put_bits(&bw, 1, 1);
    /* vps_reserved_0xffff_16bits */
    bw_put_bits(&bw, 0xffff, 16);

    /* profile_tier_level (general) */
    /* general_profile_space */
    bw_put_bits(&bw, 0, 2);
    /* general_tier_flag (0=Main, 1=High) */
    bw_put_bits(&bw, tier, 1);
    /* general_profile_idc (1=Main, 2=Main10) */
    bw_put_bits(&bw, pic->bit_depth_luma_minus8 > 0 ? 2 : 1, 5);
    /* general_profile_compatibility_flag[32]
     * flag[j] is written MSB first, so flag[1] = bit 30, flag[2] = bit 29
     * Main (profile 1): set flag[1] and flag[2] for Main/Main10 compat
     * Main10 (profile 2): set flag[2] for Main10 compat
     */
    uint32_t compat;
    if (pic->bit_depth_luma_minus8 > 0) {
        /* Main10 profile */
        compat = (1U << 29);  /* flag[2] for Main10 */
    } else {
        /* Main profile - compatible with Main and Main10 */
        compat = (1U << 30) | (1U << 29);  /* flag[1] and flag[2] */
    }
    bw_put_bits(&bw, compat, 32);
    /* general_progressive_source_flag */
    bw_put_bits(&bw, 1, 1);
    /* general_interlaced_source_flag */
    bw_put_bits(&bw, 0, 1);
    /* general_non_packed_constraint_flag */
    bw_put_bits(&bw, 0, 1);
    /* general_frame_only_constraint_flag */
    bw_put_bits(&bw, 1, 1);
    /* general_reserved_zero_44bits */
    bw_put_bits(&bw, 0, 32);
    bw_put_bits(&bw, 0, 12);
    /* general_level_idc */
    bw_put_bits(&bw, level_idc, 8);

    /* vps_sub_layer_ordering_info_present_flag */
    bw_put_bits(&bw, 1, 1);
    /* vps_max_dec_pic_buffering_minus1[0] */
    bw_put_ue(&bw, pic->sps_max_dec_pic_buffering_minus1);
    /* vps_max_num_reorder_pics[0] - ALWAYS 0 for V4L2 stateful decoders
     * V4L2 stateful decoders output in decode order. Setting max_reorder > 0
     * causes the decoder to buffer frames internally, which deadlocks with
     * our synchronous decode model. The app can reorder using pic_order_cnt.
     */
    bw_put_ue(&bw, 0);
    /* vps_max_latency_increase_plus1[0] */
    bw_put_ue(&bw, 0);

    /* vps_max_layer_id */
    bw_put_bits(&bw, 0, 6);
    /* vps_num_layer_sets_minus1 */
    bw_put_ue(&bw, 0);
    /* vps_timing_info_present_flag */
    bw_put_bits(&bw, 0, 1);
    /* vps_extension_flag */
    bw_put_bits(&bw, 0, 1);

    return bw_finish(&bw);
}

/*
 * Generate SPS NAL unit from VAPictureParameterBufferHEVC
 */
static size_t hevc_generate_sps(VAPictureParameterBufferHEVC *pic, uint8_t *buf, size_t bufsize) {
    BitWriter bw;
    bw_init(&bw, buf, bufsize);

    int level_idc = hevc_calc_level(pic);
    int tier = hevc_calc_tier(pic, level_idc);
    int profile_idc = pic->bit_depth_luma_minus8 > 0 ? 2 : 1;  /* Main10 or Main */

    /* NAL header */
    hevc_write_nal_header(&bw, HEVC_NAL_SPS);

    /* sps_video_parameter_set_id */
    bw_put_bits(&bw, 0, 4);
    /* sps_max_sub_layers_minus1 */
    bw_put_bits(&bw, 0, 3);
    /* sps_temporal_id_nesting_flag */
    bw_put_bits(&bw, 1, 1);

    /* profile_tier_level (same as VPS) */
    bw_put_bits(&bw, 0, 2);  /* general_profile_space */
    bw_put_bits(&bw, tier, 1);  /* general_tier_flag (0=Main, 1=High) */
    bw_put_bits(&bw, profile_idc, 5);  /* general_profile_idc */
    /* general_profile_compatibility_flag[32] - flag[j] at bit (31-j) */
    uint32_t compat = (profile_idc == 2) ? (1U << 29) : ((1U << 30) | (1U << 29));
    bw_put_bits(&bw, compat, 32);  /* general_profile_compatibility_flag */
    bw_put_bits(&bw, 1, 1);  /* general_progressive_source_flag */
    bw_put_bits(&bw, 0, 1);  /* general_interlaced_source_flag */
    bw_put_bits(&bw, 0, 1);  /* general_non_packed_constraint_flag */
    bw_put_bits(&bw, 1, 1);  /* general_frame_only_constraint_flag */
    bw_put_bits(&bw, 0, 32); /* general_reserved_zero_44bits part 1 */
    bw_put_bits(&bw, 0, 12); /* general_reserved_zero_44bits part 2 */
    bw_put_bits(&bw, level_idc, 8);  /* general_level_idc */

    /* sps_seq_parameter_set_id */
    bw_put_ue(&bw, 0);
    /* chroma_format_idc */
    bw_put_ue(&bw, pic->pic_fields.bits.chroma_format_idc);
    if (pic->pic_fields.bits.chroma_format_idc == 3) {
        bw_put_bits(&bw, pic->pic_fields.bits.separate_colour_plane_flag, 1);
    }

    /* pic_width_in_luma_samples */
    bw_put_ue(&bw, pic->pic_width_in_luma_samples);
    /* pic_height_in_luma_samples */
    bw_put_ue(&bw, pic->pic_height_in_luma_samples);

    /* conformance_window_flag - check if cropping needed */
    int ctb_size = 1 << (pic->log2_min_luma_coding_block_size_minus3 + 3 +
                         pic->log2_diff_max_min_luma_coding_block_size);
    int aligned_width = ((pic->pic_width_in_luma_samples + ctb_size - 1) / ctb_size) * ctb_size;
    int aligned_height = ((pic->pic_height_in_luma_samples + ctb_size - 1) / ctb_size) * ctb_size;
    bool need_crop = (aligned_width != pic->pic_width_in_luma_samples ||
                      aligned_height != pic->pic_height_in_luma_samples);

    if (need_crop) {
        bw_put_bits(&bw, 1, 1);  /* conformance_window_flag */
        int sub_width_c = (pic->pic_fields.bits.chroma_format_idc == 1 ||
                          pic->pic_fields.bits.chroma_format_idc == 2) ? 2 : 1;
        int sub_height_c = (pic->pic_fields.bits.chroma_format_idc == 1) ? 2 : 1;
        bw_put_ue(&bw, 0);  /* conf_win_left_offset */
        bw_put_ue(&bw, (aligned_width - pic->pic_width_in_luma_samples) / sub_width_c);
        bw_put_ue(&bw, 0);  /* conf_win_top_offset */
        bw_put_ue(&bw, (aligned_height - pic->pic_height_in_luma_samples) / sub_height_c);
    } else {
        bw_put_bits(&bw, 0, 1);  /* conformance_window_flag */
    }

    /* bit_depth_luma_minus8 */
    bw_put_ue(&bw, pic->bit_depth_luma_minus8);
    /* bit_depth_chroma_minus8 */
    bw_put_ue(&bw, pic->bit_depth_chroma_minus8);

    /* log2_max_pic_order_cnt_lsb_minus4 */
    bw_put_ue(&bw, pic->log2_max_pic_order_cnt_lsb_minus4);

    /* sps_sub_layer_ordering_info_present_flag */
    bw_put_bits(&bw, 1, 1);
    /* sps_max_dec_pic_buffering_minus1[0] */
    bw_put_ue(&bw, pic->sps_max_dec_pic_buffering_minus1);
    /* sps_max_num_reorder_pics[0] - ALWAYS 0 (see VPS comment) */
    bw_put_ue(&bw, 0);
    /* sps_max_latency_increase_plus1[0] */
    bw_put_ue(&bw, 0);

    /* log2_min_luma_coding_block_size_minus3 */
    bw_put_ue(&bw, pic->log2_min_luma_coding_block_size_minus3);
    /* log2_diff_max_min_luma_coding_block_size */
    bw_put_ue(&bw, pic->log2_diff_max_min_luma_coding_block_size);
    /* log2_min_luma_transform_block_size_minus2 */
    bw_put_ue(&bw, pic->log2_min_transform_block_size_minus2);
    /* log2_diff_max_min_luma_transform_block_size */
    bw_put_ue(&bw, pic->log2_diff_max_min_transform_block_size);
    /* max_transform_hierarchy_depth_inter */
    bw_put_ue(&bw, pic->max_transform_hierarchy_depth_inter);
    /* max_transform_hierarchy_depth_intra */
    bw_put_ue(&bw, pic->max_transform_hierarchy_depth_intra);

    /* scaling_list_enabled_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.scaling_list_enabled_flag, 1);
    if (pic->pic_fields.bits.scaling_list_enabled_flag) {
        bw_put_bits(&bw, 0, 1);  /* sps_scaling_list_data_present_flag */
    }

    /* amp_enabled_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.amp_enabled_flag, 1);
    /* sample_adaptive_offset_enabled_flag */
    bw_put_bits(&bw, pic->slice_parsing_fields.bits.sample_adaptive_offset_enabled_flag, 1);

    /* pcm_enabled_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.pcm_enabled_flag, 1);
    if (pic->pic_fields.bits.pcm_enabled_flag) {
        bw_put_bits(&bw, pic->pcm_sample_bit_depth_luma_minus1, 4);
        bw_put_bits(&bw, pic->pcm_sample_bit_depth_chroma_minus1, 4);
        bw_put_ue(&bw, pic->log2_min_pcm_luma_coding_block_size_minus3);
        bw_put_ue(&bw, pic->log2_diff_max_min_pcm_luma_coding_block_size);
        bw_put_bits(&bw, pic->pic_fields.bits.pcm_loop_filter_disabled_flag, 1);
    }

    /* num_short_term_ref_pic_sets */
    bw_put_ue(&bw, 0);
    /* long_term_ref_pics_present_flag */
    bw_put_bits(&bw, pic->slice_parsing_fields.bits.long_term_ref_pics_present_flag, 1);
    if (pic->slice_parsing_fields.bits.long_term_ref_pics_present_flag) {
        bw_put_ue(&bw, 0);  /* num_long_term_ref_pics_sps */
    }

    /* sps_temporal_mvp_enabled_flag */
    bw_put_bits(&bw, pic->slice_parsing_fields.bits.sps_temporal_mvp_enabled_flag, 1);
    /* strong_intra_smoothing_enabled_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.strong_intra_smoothing_enabled_flag, 1);

    /* vui_parameters_present_flag - enable for color signaling */
    bw_put_bits(&bw, 1, 1);
    hevc_write_vui(&bw, pic);

    /* sps_extension_present_flag */
    bw_put_bits(&bw, 0, 1);

    return bw_finish(&bw);
}

/*
 * Generate PPS NAL unit from VAPictureParameterBufferHEVC
 */
static size_t hevc_generate_pps(VAPictureParameterBufferHEVC *pic, uint8_t *buf, size_t bufsize) {
    BitWriter bw;
    bw_init(&bw, buf, bufsize);

    /* NAL header */
    hevc_write_nal_header(&bw, HEVC_NAL_PPS);

    /* pps_pic_parameter_set_id */
    bw_put_ue(&bw, 0);
    /* pps_seq_parameter_set_id */
    bw_put_ue(&bw, 0);

    /* dependent_slice_segments_enabled_flag */
    bw_put_bits(&bw, pic->slice_parsing_fields.bits.dependent_slice_segments_enabled_flag, 1);
    /* output_flag_present_flag */
    bw_put_bits(&bw, pic->slice_parsing_fields.bits.output_flag_present_flag, 1);
    /* num_extra_slice_header_bits */
    bw_put_bits(&bw, pic->num_extra_slice_header_bits, 3);
    /* sign_data_hiding_enabled_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.sign_data_hiding_enabled_flag, 1);
    /* cabac_init_present_flag */
    bw_put_bits(&bw, pic->slice_parsing_fields.bits.cabac_init_present_flag, 1);

    /* num_ref_idx_l0_default_active_minus1 */
    bw_put_ue(&bw, pic->num_ref_idx_l0_default_active_minus1);
    /* num_ref_idx_l1_default_active_minus1 */
    bw_put_ue(&bw, pic->num_ref_idx_l1_default_active_minus1);

    /* init_qp_minus26 */
    bw_put_se(&bw, pic->init_qp_minus26);
    /* constrained_intra_pred_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.constrained_intra_pred_flag, 1);
    /* transform_skip_enabled_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.transform_skip_enabled_flag, 1);

    /* cu_qp_delta_enabled_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.cu_qp_delta_enabled_flag, 1);
    if (pic->pic_fields.bits.cu_qp_delta_enabled_flag) {
        bw_put_ue(&bw, pic->diff_cu_qp_delta_depth);
    }

    /* pps_cb_qp_offset */
    bw_put_se(&bw, pic->pps_cb_qp_offset);
    /* pps_cr_qp_offset */
    bw_put_se(&bw, pic->pps_cr_qp_offset);
    /* pps_slice_chroma_qp_offsets_present_flag */
    bw_put_bits(&bw, pic->slice_parsing_fields.bits.pps_slice_chroma_qp_offsets_present_flag, 1);

    /* weighted_pred_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.weighted_pred_flag, 1);
    /* weighted_bipred_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.weighted_bipred_flag, 1);
    /* transquant_bypass_enabled_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.transquant_bypass_enabled_flag, 1);

    /* tiles_enabled_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.tiles_enabled_flag, 1);
    /* entropy_coding_sync_enabled_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.entropy_coding_sync_enabled_flag, 1);

    if (pic->pic_fields.bits.tiles_enabled_flag) {
        bw_put_ue(&bw, pic->num_tile_columns_minus1);
        bw_put_ue(&bw, pic->num_tile_rows_minus1);
        /* uniform_spacing_flag - assume uniform for simplicity */
        bw_put_bits(&bw, 1, 1);
        /* loop_filter_across_tiles_enabled_flag */
        bw_put_bits(&bw, pic->pic_fields.bits.loop_filter_across_tiles_enabled_flag, 1);
    }

    /* pps_loop_filter_across_slices_enabled_flag */
    bw_put_bits(&bw, pic->pic_fields.bits.pps_loop_filter_across_slices_enabled_flag, 1);

    /* deblocking_filter_control_present_flag */
    bw_put_bits(&bw, pic->slice_parsing_fields.bits.deblocking_filter_override_enabled_flag ||
                     pic->slice_parsing_fields.bits.pps_disable_deblocking_filter_flag, 1);
    if (pic->slice_parsing_fields.bits.deblocking_filter_override_enabled_flag ||
        pic->slice_parsing_fields.bits.pps_disable_deblocking_filter_flag) {
        bw_put_bits(&bw, pic->slice_parsing_fields.bits.deblocking_filter_override_enabled_flag, 1);
        bw_put_bits(&bw, pic->slice_parsing_fields.bits.pps_disable_deblocking_filter_flag, 1);
        if (!pic->slice_parsing_fields.bits.pps_disable_deblocking_filter_flag) {
            bw_put_se(&bw, pic->pps_beta_offset_div2);
            bw_put_se(&bw, pic->pps_tc_offset_div2);
        }
    }

    /* pps_scaling_list_data_present_flag */
    bw_put_bits(&bw, 0, 1);
    /* lists_modification_present_flag */
    bw_put_bits(&bw, pic->slice_parsing_fields.bits.lists_modification_present_flag, 1);
    /* log2_parallel_merge_level_minus2 */
    bw_put_ue(&bw, pic->log2_parallel_merge_level_minus2);
    /* slice_segment_header_extension_present_flag */
    bw_put_bits(&bw, pic->slice_parsing_fields.bits.slice_segment_header_extension_present_flag, 1);
    /* pps_extension_present_flag */
    bw_put_bits(&bw, 0, 1);

    return bw_finish(&bw);
}

/*
 * Prepend VPS/SPS/PPS to bitstream
 * Uses reconstructed parameter sets from VA-API params.
 */
static void hevc_prepend_parameter_sets(V4L2Context *ctx)
{
    /* VPS */
    if (ctx->hevc.last_vps_size > 0) {
        bitstream_append(&ctx->bitstream, NAL_START_CODE, sizeof(NAL_START_CODE));
        bitstream_append(&ctx->bitstream, ctx->hevc.last_vps, ctx->hevc.last_vps_size);
    }

    /* SPS */
    if (ctx->hevc.last_sps_size > 0) {
        bitstream_append(&ctx->bitstream, NAL_START_CODE, sizeof(NAL_START_CODE));
        bitstream_append(&ctx->bitstream, ctx->hevc.last_sps, ctx->hevc.last_sps_size);
    }

    /* PPS */
    if (ctx->hevc.last_pps_size > 0) {
        bitstream_append(&ctx->bitstream, NAL_START_CODE, sizeof(NAL_START_CODE));
        bitstream_append(&ctx->bitstream, ctx->hevc.last_pps, ctx->hevc.last_pps_size);
    }
}

/* Cache key for detecting parameter changes */
static uint32_t hevc_last_width = 0;
static uint32_t hevc_last_height = 0;
static uint8_t hevc_last_bit_depth = 0;

/*
 * Handle HEVC picture parameters - generate and cache VPS/SPS/PPS
 * Only regenerate when parameters actually change to avoid redundant work.
 */
static void hevc_handle_picture_params(V4L2Context *ctx, V4L2Buffer *buf)
{
    VAPictureParameterBufferHEVC *pic = (VAPictureParameterBufferHEVC *)buf->data;

    /* Check if parameters changed - only regenerate if needed */
    bool params_changed = (pic->pic_width_in_luma_samples != hevc_last_width ||
                          pic->pic_height_in_luma_samples != hevc_last_height ||
                          pic->bit_depth_luma_minus8 != hevc_last_bit_depth ||
                          ctx->hevc.last_vps_size == 0);

    if (params_changed) {
        /* Generate and cache reconstructed VPS/SPS/PPS */
        ctx->hevc.last_vps_size = hevc_generate_vps(pic, ctx->hevc.last_vps, sizeof(ctx->hevc.last_vps));
        ctx->hevc.last_sps_size = hevc_generate_sps(pic, ctx->hevc.last_sps, sizeof(ctx->hevc.last_sps));
        ctx->hevc.last_pps_size = hevc_generate_pps(pic, ctx->hevc.last_pps, sizeof(ctx->hevc.last_pps));

        /* Update cache key */
        hevc_last_width = pic->pic_width_in_luma_samples;
        hevc_last_height = pic->pic_height_in_luma_samples;
        hevc_last_bit_depth = pic->bit_depth_luma_minus8;

        /* Log only on parameter changes */
        int profile = pic->bit_depth_luma_minus8 > 0 ? 2 : 1;
        int level = hevc_calc_level(pic);
        int tier = hevc_calc_tier(pic, level);

        LOG("HEVC: Picture params: %dx%d, Main%s, L%d.%d %s tier",
            pic->pic_width_in_luma_samples,
            pic->pic_height_in_luma_samples,
            profile == 2 ? "10" : "",
            level / 30, (level % 30) / 3,
            tier ? "High" : "Main");
    }
}

/*
 * Handle HEVC slice data (the actual compressed bitstream)
 */
static void hevc_handle_slice_data(V4L2Context *ctx, V4L2Buffer *buf)
{
    VASliceParameterBufferHEVC *slice_params = ctx->last_slice_params;

    if (!slice_params) {
        LOG("HEVC: No slice params available!");
        return;
    }

    /*
     * NOTE: NAL scanning for original VPS/SPS/PPS is disabled for performance.
     * VA-API parsed streams from MP4/MKV containers never contain parameter sets
     * in the slice data - they've been extracted by the demuxer. Only raw Annex-B
     * or MPEG-TS sources would have them, and those rarely use VA-API.
     * We always use reconstructed parameter sets instead.
     */
    (void)hevc_scan_for_parameter_sets;  /* Suppress unused warning */

    for (unsigned int i = 0; i < ctx->last_slice_count; i++) {
        VASliceParameterBufferHEVC *sp = &slice_params[i];
        uint8_t *slice_data = (uint8_t *)buf->data + sp->slice_data_offset;

        /* Get NAL type from slice data (first 2 bytes are NAL header) */
        uint8_t nal_type = (slice_data[0] >> 1) & 0x3f;

        /* Skip VPS/SPS/PPS NAL units in slice data - we handle them separately */
        if (nal_type == HEVC_NAL_VPS || nal_type == HEVC_NAL_SPS || nal_type == HEVC_NAL_PPS) {
            continue;
        }

        /* For IDR/CRA slices, prepend VPS/SPS/PPS */
        if ((nal_type >= HEVC_NAL_IDR_W_RADL && nal_type <= HEVC_NAL_CRA_NUT) &&
            !ctx->hevc.params_sent) {
            hevc_prepend_parameter_sets(ctx);
            ctx->hevc.params_sent = true;
        }

        /* Prepend NAL start code and append slice data */
        bitstream_append(&ctx->bitstream, NAL_START_CODE, sizeof(NAL_START_CODE));
        bitstream_append(&ctx->bitstream, slice_data, sp->slice_data_size);
    }
}

/*
 * Prepare bitstream for submission
 */
static void hevc_prepare_bitstream(V4L2Context *ctx)
{
    (void)ctx;
}

/* Supported HEVC profiles */
static VAProfile hevc_profiles[] = {
    VAProfileHEVCMain,
    VAProfileHEVCMain10,
};

/* HEVC codec definition */
const V4L2Codec hevc_codec = {
    .name = "HEVC",
    .v4l2_pixfmt = V4L2_PIX_FMT_HEVC,
    .profiles = hevc_profiles,
    .num_profiles = sizeof(hevc_profiles) / sizeof(hevc_profiles[0]),
    .handle_picture_params = hevc_handle_picture_params,
    .handle_slice_data = hevc_handle_slice_data,
    .prepare_bitstream = hevc_prepare_bitstream,
};
