/*
 * H.264 codec support for VA-API to V4L2 stateful backend
 *
 * Key insight: VA-API provides raw NAL unit data in slice data buffers.
 * For V4L2 stateful decoder, we just need to:
 * 1. Prepend NAL start codes (0x00 0x00 0x01)
 * 2. Concatenate all slice data
 * 3. Submit to V4L2 OUTPUT queue
 *
 * The hardware handles all parsing internally.
 */

#include "vabackend.h"
#include <va/va.h>
#include <string.h>

static const uint8_t NAL_START_CODE[] = { 0x00, 0x00, 0x01 };

/*
 * Handle H.264 picture parameters
 * For stateful V4L2, we don't need to parse these - hardware does it.
 * But we store for potential debugging/validation.
 */
static void h264_handle_picture_params(V4L2Context *ctx, V4L2Buffer *buf)
{
    /* VAPictureParameterBufferH264 *pic = (VAPictureParameterBufferH264 *)buf->data; */
    /* For stateful decoder, picture params are in the bitstream - nothing to do */
    (void)ctx;
    (void)buf;
}

/*
 * Handle H.264 slice parameters
 * These tell us offset/size within the slice data buffer.
 */
static void h264_handle_slice_params(V4L2Context *ctx, V4L2Buffer *buf)
{
    ctx->last_slice_params = buf->data;
    ctx->last_slice_count = buf->num_elements;
}

/*
 * Handle H.264 slice data (the actual compressed bitstream)
 *
 * VA-API provides slice data without start codes. We need to:
 * 1. For each slice, prepend start code
 * 2. Copy the raw NAL unit data
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

        /* Prepend NAL start code */
        bitstream_append(&ctx->bitstream, NAL_START_CODE, sizeof(NAL_START_CODE));

        /* Append the slice data (raw NAL unit content) */
        uint8_t *slice_data = (uint8_t *)buf->data + sp->slice_data_offset;
        bitstream_append(&ctx->bitstream, slice_data, sp->slice_data_size);
    }
}

/*
 * Handle H.264 IQ matrix
 * Not needed for stateful V4L2 - hardware parses from bitstream
 */
static void h264_handle_iq_matrix(V4L2Context *ctx, V4L2Buffer *buf)
{
    (void)ctx;
    (void)buf;
}

/*
 * Prepare bitstream for submission
 * For H.264, the bitstream is already assembled in handle_slice_data.
 * This is a hook for codecs that need post-processing.
 */
static void h264_prepare_bitstream(V4L2Context *ctx)
{
    /* H.264 bitstream is ready as-is */
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
    .handle_slice_data = h264_handle_slice_data,
    .prepare_bitstream = h264_prepare_bitstream,
};
