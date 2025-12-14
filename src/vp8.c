/*
 * VP8 codec support for VA-API to V4L2 stateful backend
 *
 * VP8 bitstream format is different from H.264/HEVC:
 * - No NAL units, just raw VP8 frames
 * - VA-API provides the frame data directly
 */

#include "vabackend.h"
#include <va/va.h>
#include <string.h>

/*
 * Handle VP8 slice data
 * VA-API provides the raw VP8 frame data directly
 */
static void vp8_handle_slice_data(V4L2Context *ctx, V4L2Buffer *buf)
{
    VASliceParameterBufferVP8 *slice_params = ctx->last_slice_params;

    if (!slice_params) {
        LOG("VP8: No slice params available!");
        return;
    }

    /* VP8 typically has one slice per frame */
    for (unsigned int i = 0; i < ctx->last_slice_count; i++) {
        VASliceParameterBufferVP8 *sp = &slice_params[i];

        /* For VP8, slice_data_offset points to the frame data */
        uint8_t *frame_data = (uint8_t *)buf->data + sp->slice_data_offset;
        bitstream_append(&ctx->bitstream, frame_data, sp->slice_data_size);
    }
}

/*
 * Prepare bitstream for submission
 */
static void vp8_prepare_bitstream(V4L2Context *ctx)
{
    /* VP8 bitstream is ready as-is */
    (void)ctx;
}

/* Supported VP8 profiles */
static VAProfile vp8_profiles[] = {
    VAProfileVP8Version0_3,
};

/* VP8 codec definition */
const V4L2Codec vp8_codec = {
    .name = "VP8",
    .v4l2_pixfmt = V4L2_PIX_FMT_VP8,
    .profiles = vp8_profiles,
    .num_profiles = sizeof(vp8_profiles) / sizeof(vp8_profiles[0]),
    .handle_slice_data = vp8_handle_slice_data,
    .prepare_bitstream = vp8_prepare_bitstream,
};
