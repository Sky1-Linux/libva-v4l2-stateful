/*
 * HEVC (H.265) codec support for VA-API to V4L2 stateful backend
 *
 * Similar to H.264, VA-API provides raw NAL unit data.
 * For V4L2 stateful decoder:
 * 1. Prepend NAL start codes (0x00 0x00 0x01)
 * 2. Concatenate all slice data
 * 3. Submit to V4L2 OUTPUT queue
 */

#include "vabackend.h"
#include <va/va.h>
#include <string.h>

static const uint8_t NAL_START_CODE[] = { 0x00, 0x00, 0x01 };

/*
 * Handle HEVC slice data
 * VA-API provides slice data without start codes.
 */
static void hevc_handle_slice_data(V4L2Context *ctx, V4L2Buffer *buf)
{
    VASliceParameterBufferHEVC *slice_params = ctx->last_slice_params;

    if (!slice_params) {
        LOG("HEVC: No slice params available!");
        return;
    }

    for (unsigned int i = 0; i < ctx->last_slice_count; i++) {
        VASliceParameterBufferHEVC *sp = &slice_params[i];

        /* Prepend NAL start code */
        bitstream_append(&ctx->bitstream, NAL_START_CODE, sizeof(NAL_START_CODE));

        /* Append the slice data (raw NAL unit content) */
        uint8_t *slice_data = (uint8_t *)buf->data + sp->slice_data_offset;
        bitstream_append(&ctx->bitstream, slice_data, sp->slice_data_size);
    }
}

/*
 * Prepare bitstream for submission
 */
static void hevc_prepare_bitstream(V4L2Context *ctx)
{
    /* HEVC bitstream is ready as-is */
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
    .handle_slice_data = hevc_handle_slice_data,
    .prepare_bitstream = hevc_prepare_bitstream,
};
