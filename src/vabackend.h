/*
 * VA-API to V4L2 Stateful Decoder Backend
 *
 * This driver provides a VA-API interface over V4L2 stateful (memory-to-memory)
 * video decoders. Unlike stateless V4L2 which requires parsed slice data,
 * stateful V4L2 accepts raw bitstreams and handles parsing internally.
 *
 * Architecture:
 *   VA-API Application (Firefox, etc.)
 *         |
 *         v
 *   [libva-v4l2-stateful] (this driver)
 *         |
 *   - Collect slice data buffers (raw NAL units)
 *   - Prepend start codes
 *   - Submit to V4L2 OUTPUT queue
 *         |
 *         v
 *   V4L2 Stateful Decoder (/dev/video0)
 *         |
 *         v
 *   V4L2 CAPTURE queue (decoded frames)
 *         |
 *   DMABuf export back to VA-API surface
 */

#ifndef VABACKEND_H
#define VABACKEND_H

#include <va/va_backend.h>
#include <va/va_drmcommon.h>
#include <stdbool.h>
#include <stdint.h>
#include <pthread.h>
#include <linux/videodev2.h>

#define MAX_SURFACES 32
#define MAX_BUFFERS 1024
#define MAX_FRAME_BUFFERS 1024
#define MAX_PROFILES 16
#define MAX_OUTPUT_BUFFERS 8
#define MAX_CAPTURE_BUFFERS 16
#define BITSTREAM_BUFFER_SIZE (4 * 1024 * 1024)  /* 4MB */

/* Forward declarations */
struct V4L2Context;
struct V4L2Surface;
struct V4L2Codec;

/* Growable buffer for accumulating bitstream data */
typedef struct {
    void        *data;
    size_t      size;
    size_t      allocated;
} BitstreamBuffer;

/* VA-API buffer wrapper */
typedef struct {
    VABufferType    type;
    unsigned int    num_elements;
    unsigned int    element_size;
    void            *data;
    VASurfaceID     surface_id;     /* For DeriveImage buffers */
    uint32_t        width;          /* For image buffers */
    uint32_t        height;         /* For image buffers */
    int             capture_idx;    /* For image buffers mapped from CAPTURE */
    bool            in_use;         /* For image buffers held by app */
} V4L2Buffer;

/* V4L2 memory-mapped buffer */
typedef struct {
    void            *start;
    size_t          length;
    int             fd;         /* DMABuf fd for CAPTURE buffers */
    bool            queued;
    uint32_t        index;
    /* For CAPTURE buffers: cached mmap pointers */
    void            *plane0_ptr;
    void            *plane1_ptr;
    size_t          plane0_len;
    size_t          plane1_len;
} V4L2MmapBuffer;

/* Decoded surface (maps to CAPTURE buffer) */
typedef struct V4L2Surface {
    uint32_t        width;
    uint32_t        height;
    uint32_t        fourcc;         /* V4L2 pixel format */
    int             capture_idx;    /* Index in CAPTURE queue, -1 if not assigned */
    int             dmabuf_fd;      /* DMABuf fd for zero-copy */
    bool            decoded;        /* Has valid decoded content */
    bool            no_output;      /* Frame decoded but no CAPTURE output (show_frame=0) */
    VAImageID       cached_image;   /* Cached image buffer for this surface */
    pthread_mutex_t mutex;
    pthread_cond_t  cond;
    struct V4L2Context *context;
} V4L2Surface;

/* Codec-specific handler */
typedef struct V4L2Codec {
    const char      *name;
    uint32_t        v4l2_pixfmt;    /* V4L2_PIX_FMT_H264, etc. */
    VAProfile       *profiles;
    int             num_profiles;

    /* Called when slice data buffer is submitted */
    void (*handle_slice_data)(struct V4L2Context *ctx, V4L2Buffer *buf);

    /* Called to prepend codec-specific headers if needed */
    void (*prepare_bitstream)(struct V4L2Context *ctx);
} V4L2Codec;

/* VA-API context (created per vaCreateContext) */
typedef struct V4L2Context {
    struct V4L2Driver   *drv;
    VAProfile           profile;
    VAEntrypoint        entrypoint;
    uint32_t            width;
    uint32_t            height;

    /* V4L2 device state */
    int                 v4l2_fd;
    bool                streaming_output;
    bool                streaming_capture;

    /* OUTPUT queue (compressed bitstream) */
    V4L2MmapBuffer      output_buffers[MAX_OUTPUT_BUFFERS];
    int                 num_output_buffers;
    int                 output_buf_idx;     /* Next buffer to use */

    /* CAPTURE queue (decoded frames) */
    V4L2MmapBuffer      capture_buffers[MAX_CAPTURE_BUFFERS];
    int                 num_capture_buffers;

    /* Current decode operation */
    V4L2Surface         *render_target;
    BitstreamBuffer     bitstream;
    const V4L2Codec     *codec;

    /* Slice data accumulation */
    void                *last_slice_params;
    unsigned int        last_slice_count;

    /* Track buffers used in current frame for cleanup */
    VABufferID          frame_buffers[MAX_FRAME_BUFFERS];
    int                 num_frame_buffers;

    pthread_mutex_t     mutex;
} V4L2Context;

/* Driver config (created per vaCreateConfig) */
typedef struct {
    VAProfile           profile;
    VAEntrypoint        entrypoint;
    uint32_t            v4l2_pixfmt;
    const V4L2Codec     *codec;
} V4L2Config;

/* Main driver state */
typedef struct V4L2Driver {
    int                 drm_fd;             /* DRM device fd from vaGetDisplayDRM */
    char                v4l2_device[64];    /* e.g., "/dev/video0" */

    /* Object storage */
    V4L2Config          *configs[MAX_PROFILES];
    int                 num_configs;

    V4L2Context         *contexts[MAX_PROFILES];
    int                 num_contexts;

    V4L2Surface         *surfaces[MAX_SURFACES];
    int                 num_surfaces;

    V4L2Buffer          *buffers[MAX_BUFFERS];
    int                 num_buffers;

    /* Supported profiles detected from V4L2 */
    VAProfile           supported_profiles[MAX_PROFILES];
    int                 num_supported_profiles;

    pthread_mutex_t     mutex;
} V4L2Driver;

/* Utility functions */
void bitstream_append(BitstreamBuffer *bb, const void *data, size_t size);
void bitstream_reset(BitstreamBuffer *bb);
void bitstream_free(BitstreamBuffer *bb);

/* Logging */
void v4l2va_log(const char *file, const char *func, int line, const char *fmt, ...);
#define LOG(...) v4l2va_log(__FILE__, __func__, __LINE__, __VA_ARGS__)

/* V4L2 backend functions */
int v4l2_open_device(V4L2Driver *drv);
void v4l2_close_device(V4L2Driver *drv, int fd);
int v4l2_probe_capabilities(V4L2Driver *drv, int fd);
int v4l2_setup_output_queue(V4L2Context *ctx);
int v4l2_setup_capture_queue(V4L2Context *ctx);
int v4l2_queue_bitstream(V4L2Context *ctx, void *data, size_t size);
int v4l2_dequeue_frame(V4L2Context *ctx, V4L2Surface *surface);
int v4l2_requeue_capture(V4L2Context *ctx, int capture_idx);
int v4l2_export_dmabuf(V4L2Context *ctx, int capture_idx);

/* Codec registration */
extern const V4L2Codec h264_codec;
extern const V4L2Codec hevc_codec;
extern const V4L2Codec vp8_codec;
extern const V4L2Codec vp9_codec;

#endif /* VABACKEND_H */
