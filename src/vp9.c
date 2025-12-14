/*
 * VP9 codec support for VA-API to V4L2 stateful backend
 *
 * VP9 bitstream format:
 * - No NAL units, raw VP9 superframes
 * - VA-API provides the frame data directly
 * - Superframe format may contain multiple frames
 */

#include "vabackend.h"
#include <va/va.h>
#include <string.h>

/*
 * Handle VP9 slice data
 * VA-API provides the raw VP9 frame data
 */
static void vp9_handle_slice_data(V4L2Context *ctx, V4L2Buffer *buf)
{
    VASliceParameterBufferVP9 *slice_params = ctx->last_slice_params;

    if (!slice_params) {
        LOG("VP9: No slice params available!");
        return;
    }

    for (unsigned int i = 0; i < ctx->last_slice_count; i++) {
        VASliceParameterBufferVP9 *sp = &slice_params[i];

        /* VP9 frame data */
        uint8_t *frame_data = (uint8_t *)buf->data + sp->slice_data_offset;
        bitstream_append(&ctx->bitstream, frame_data, sp->slice_data_size);
    }
}

/*
 * Prepare bitstream for submission
 */
static void vp9_prepare_bitstream(V4L2Context *ctx)
{
    /* VP9 bitstream is ready as-is */
    (void)ctx;
}

/* Supported VP9 profiles */
static VAProfile vp9_profiles[] = {
    VAProfileVP9Profile0,
    VAProfileVP9Profile2,
};

/* VP9 codec definition */
const V4L2Codec vp9_codec = {
    .name = "VP9",
    .v4l2_pixfmt = V4L2_PIX_FMT_VP9,
    .profiles = vp9_profiles,
    .num_profiles = sizeof(vp9_profiles) / sizeof(vp9_profiles[0]),
    .handle_slice_data = vp9_handle_slice_data,
    .prepare_bitstream = vp9_prepare_bitstream,
};
