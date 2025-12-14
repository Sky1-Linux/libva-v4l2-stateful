/*
 * VA-API to V4L2 Stateful Decoder Backend - Main Implementation
 *
 * This is the main VA-API driver entry point that translates VA-API calls
 * into V4L2 stateful (memory-to-memory) decoder operations.
 *
 * The key insight: V4L2 stateful decoders accept raw bitstream data
 * on the OUTPUT queue and produce decoded frames on the CAPTURE queue.
 * Unlike stateless V4L2, we don't need to parse slice data - just forward
 * the raw NAL units with start codes.
 */

#define _GNU_SOURCE

#include "vabackend.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdarg.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <va/va_backend.h>
#include <va/va_drmcommon.h>
#include <linux/videodev2.h>
#include <drm_fourcc.h>

/* Logging */
static FILE *log_output = NULL;

__attribute__ ((constructor))
static void driver_init(void)
{
    char *log_env = getenv("V4L2VA_LOG");
    if (log_env != NULL) {
        if (strcmp(log_env, "1") == 0) {
            log_output = stderr;
        } else {
            log_output = fopen(log_env, "a");
            if (log_output == NULL) {
                log_output = stderr;
            }
        }
    }
}

__attribute__ ((destructor))
static void driver_cleanup(void)
{
    if (log_output != NULL && log_output != stderr) {
        fclose(log_output);
    }
}

void v4l2va_log(const char *file, const char *func, int line, const char *fmt, ...)
{
    if (log_output == NULL)
        return;

    va_list args;
    char msg[1024];

    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);

    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);

    fprintf(log_output, "%ld.%09ld [%d] %s:%d %s: %s\n",
            (long)ts.tv_sec, ts.tv_nsec, getpid(), file, line, func, msg);
    fflush(log_output);
}

/* Codec registry */
static const V4L2Codec *codecs[] = {
    &h264_codec,
    &hevc_codec,
    &vp8_codec,
    &vp9_codec,
    NULL
};

static const V4L2Codec *codec_for_profile(VAProfile profile)
{
    for (int i = 0; codecs[i] != NULL; i++) {
        const V4L2Codec *codec = codecs[i];
        for (int j = 0; j < codec->num_profiles; j++) {
            if (codec->profiles[j] == profile) {
                return codec;
            }
        }
    }
    return NULL;
}

/*
 * Object management - simple ID-based allocation
 * VA-API uses IDs (ints) to reference objects
 */
static VAGenericID allocate_config_id(V4L2Driver *drv)
{
    for (int i = 0; i < MAX_PROFILES; i++) {
        if (drv->configs[i] == NULL) {
            return i + 1;  /* 1-based IDs */
        }
    }
    return VA_INVALID_ID;
}

static VAGenericID allocate_context_id(V4L2Driver *drv)
{
    for (int i = 0; i < MAX_PROFILES; i++) {
        if (drv->contexts[i] == NULL) {
            return i + 1 + 0x1000;  /* Offset to avoid collision */
        }
    }
    return VA_INVALID_ID;
}

static VAGenericID allocate_surface_id(V4L2Driver *drv)
{
    for (int i = 0; i < MAX_SURFACES; i++) {
        if (drv->surfaces[i] == NULL) {
            return i + 1 + 0x2000;
        }
    }
    return VA_INVALID_ID;
}

static VAGenericID allocate_buffer_id(V4L2Driver *drv)
{
    VAGenericID id = VA_INVALID_ID;

    pthread_mutex_lock(&drv->mutex);
    for (int i = 0; i < MAX_BUFFERS; i++) {
        if (drv->buffers[i] == NULL) {
            id = i + 1 + 0x3000;
            break;
        }
    }
    pthread_mutex_unlock(&drv->mutex);
    return id;
}

/* Object lookup */
#define CONFIG_INDEX(id) ((id) - 1)
#define CONTEXT_INDEX(id) ((id) - 1 - 0x1000)
#define SURFACE_INDEX(id) ((id) - 1 - 0x2000)
#define BUFFER_INDEX(id) ((id) - 1 - 0x3000)

static V4L2Config *get_config(V4L2Driver *drv, VAConfigID id)
{
    int idx = CONFIG_INDEX(id);
    if (idx >= 0 && idx < MAX_PROFILES)
        return drv->configs[idx];
    return NULL;
}

static V4L2Context *get_context(V4L2Driver *drv, VAContextID id)
{
    int idx = CONTEXT_INDEX(id);
    if (idx >= 0 && idx < MAX_PROFILES)
        return drv->contexts[idx];
    return NULL;
}

static V4L2Surface *get_surface(V4L2Driver *drv, VASurfaceID id)
{
    int idx = SURFACE_INDEX(id);
    if (idx >= 0 && idx < MAX_SURFACES)
        return drv->surfaces[idx];
    return NULL;
}

static V4L2Buffer *get_buffer(V4L2Driver *drv, VABufferID id)
{
    int idx = BUFFER_INDEX(id);
    if (idx >= 0 && idx < MAX_BUFFERS)
        return drv->buffers[idx];
    return NULL;
}

/* Track per-frame buffers so we can free them after submission */
static void track_frame_buffer(V4L2Context *context, VABufferID id)
{
    if (context->num_frame_buffers >= MAX_FRAME_BUFFERS)
        return;
    context->frame_buffers[context->num_frame_buffers++] = id;
}

/* Forward declarations for cleanup helpers */
static VAStatus v4l2_DestroySurfaces(VADriverContextP ctx, VASurfaceID *surface_list, int num_surfaces);
static VAStatus v4l2_DestroyContext(VADriverContextP ctx, VAContextID context_id);

/*
 * VA-API Entry Points
 */

static VAStatus v4l2_Terminate(VADriverContextP ctx)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    LOG("Terminating V4L2 VA-API driver");

    /* Destroy surfaces first so any held CAPTURE buffers are returned while contexts exist */
    for (int i = 0; i < MAX_SURFACES; i++) {
        if (drv->surfaces[i]) {
            VASurfaceID id = i + 1 + 0x2000;
            v4l2_DestroySurfaces(ctx, &id, 1);
        }
    }

    /* Clean up contexts */
    for (int i = 0; i < MAX_PROFILES; i++) {
        if (drv->contexts[i]) {
            VAContextID id = i + 1 + 0x1000;
            v4l2_DestroyContext(ctx, id);
        }
    }

    /* Clean up any remaining buffers */
    for (int i = 0; i < MAX_BUFFERS; i++) {
        if (drv->buffers[i]) {
            free(drv->buffers[i]->data);
            free(drv->buffers[i]);
            drv->buffers[i] = NULL;
        }
    }

    /* Clean up configs */
    for (int i = 0; i < MAX_PROFILES; i++) {
        free(drv->configs[i]);
    }

    pthread_mutex_destroy(&drv->mutex);
    free(drv);
    ctx->pDriverData = NULL;

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_QueryConfigProfiles(
    VADriverContextP ctx,
    VAProfile *profile_list,
    int *num_profiles)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;

    for (int i = 0; i < drv->num_supported_profiles; i++) {
        profile_list[i] = drv->supported_profiles[i];
    }
    *num_profiles = drv->num_supported_profiles;

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_QueryConfigEntrypoints(
    VADriverContextP ctx,
    VAProfile profile,
    VAEntrypoint *entrypoint_list,
    int *num_entrypoints)
{
    /* We only support VLD (Variable Length Decoding) */
    const V4L2Codec *codec = codec_for_profile(profile);
    if (codec == NULL) {
        *num_entrypoints = 0;
        return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
    }

    entrypoint_list[0] = VAEntrypointVLD;
    *num_entrypoints = 1;

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_QueryConfigAttributes(
    VADriverContextP ctx,
    VAConfigID config,
    VAProfile *profile,
    VAEntrypoint *entrypoint,
    VAConfigAttrib *attrib_list,
    int *num_attribs)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Config *cfg = get_config(drv, config);

    if (cfg == NULL)
        return VA_STATUS_ERROR_INVALID_CONFIG;

    *profile = cfg->profile;
    *entrypoint = cfg->entrypoint;
    *num_attribs = 0;

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_GetConfigAttributes(
    VADriverContextP ctx,
    VAProfile profile,
    VAEntrypoint entrypoint,
    VAConfigAttrib *attrib_list,
    int num_attribs)
{
    const V4L2Codec *codec = codec_for_profile(profile);
    if (codec == NULL)
        return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;

    for (int i = 0; i < num_attribs; i++) {
        switch (attrib_list[i].type) {
        case VAConfigAttribRTFormat:
            attrib_list[i].value = VA_RT_FORMAT_YUV420;
            /* Add 10-bit support for HEVC/VP9/AV1 */
            if (profile == VAProfileHEVCMain10 ||
                profile == VAProfileVP9Profile2 ||
                profile == VAProfileAV1Profile0) {
                attrib_list[i].value |= VA_RT_FORMAT_YUV420_10;
            }
            break;
        case VAConfigAttribMaxPictureWidth:
            attrib_list[i].value = 4096;  /* Conservative estimate */
            break;
        case VAConfigAttribMaxPictureHeight:
            attrib_list[i].value = 4096;
            break;
        default:
            attrib_list[i].value = VA_ATTRIB_NOT_SUPPORTED;
            break;
        }
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_CreateConfig(
    VADriverContextP ctx,
    VAProfile profile,
    VAEntrypoint entrypoint,
    VAConfigAttrib *attrib_list,
    int num_attribs,
    VAConfigID *config_id)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;

    const V4L2Codec *codec = codec_for_profile(profile);
    if (codec == NULL) {
        LOG("Unsupported profile: %d", profile);
        return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
    }

    if (entrypoint != VAEntrypointVLD) {
        LOG("Unsupported entrypoint: %d", entrypoint);
        return VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT;
    }

    VAGenericID id = allocate_config_id(drv);
    if (id == VA_INVALID_ID)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    V4L2Config *cfg = calloc(1, sizeof(V4L2Config));
    if (cfg == NULL)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    cfg->profile = profile;
    cfg->entrypoint = entrypoint;
    cfg->v4l2_pixfmt = codec->v4l2_pixfmt;
    cfg->codec = codec;

    drv->configs[CONFIG_INDEX(id)] = cfg;
    *config_id = id;

    LOG("Created config %d for profile %d (%s)", id, profile, codec->name);
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_DestroyConfig(
    VADriverContextP ctx,
    VAConfigID config_id)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    int idx = CONFIG_INDEX(config_id);

    if (idx < 0 || idx >= MAX_PROFILES || drv->configs[idx] == NULL)
        return VA_STATUS_ERROR_INVALID_CONFIG;

    free(drv->configs[idx]);
    drv->configs[idx] = NULL;

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_CreateSurfaces(
    VADriverContextP ctx,
    int width,
    int height,
    int format,
    int num_surfaces,
    VASurfaceID *surfaces)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;

    for (int i = 0; i < num_surfaces; i++) {
        VAGenericID id = allocate_surface_id(drv);
        if (id == VA_INVALID_ID)
            return VA_STATUS_ERROR_ALLOCATION_FAILED;

        V4L2Surface *surface = calloc(1, sizeof(V4L2Surface));
        if (surface == NULL)
            return VA_STATUS_ERROR_ALLOCATION_FAILED;

        surface->width = width;
        surface->height = height;
        surface->fourcc = V4L2_PIX_FMT_NV12;  /* Default to NV12 */
        surface->capture_idx = -1;
        surface->dmabuf_fd = -1;
        surface->decoded = false;
        surface->no_output = false;
        surface->cached_image = VA_INVALID_ID;
        pthread_mutex_init(&surface->mutex, NULL);
        pthread_cond_init(&surface->cond, NULL);

        drv->surfaces[SURFACE_INDEX(id)] = surface;
        surfaces[i] = id;
    }

    LOG("Created %d surfaces (%dx%d)", num_surfaces, width, height);
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_CreateSurfaces2(
    VADriverContextP ctx,
    unsigned int format,
    unsigned int width,
    unsigned int height,
    VASurfaceID *surfaces,
    unsigned int num_surfaces,
    VASurfaceAttrib *attrib_list,
    unsigned int num_attribs)
{
    /* For now, just delegate to the simpler CreateSurfaces */
    return v4l2_CreateSurfaces(ctx, width, height, format, num_surfaces, surfaces);
}

static VAStatus v4l2_DestroySurfaces(
    VADriverContextP ctx,
    VASurfaceID *surface_list,
    int num_surfaces)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;

    for (int i = 0; i < num_surfaces; i++) {
        int idx = SURFACE_INDEX(surface_list[i]);
        if (idx >= 0 && idx < MAX_SURFACES && drv->surfaces[idx]) {
            V4L2Surface *surface = drv->surfaces[idx];
            /* Return any outstanding CAPTURE buffer to the queue */
            if (surface->context && surface->capture_idx >= 0) {
                v4l2_requeue_capture(surface->context, surface->capture_idx);
            }
            surface->cached_image = VA_INVALID_ID;
            if (surface->dmabuf_fd >= 0) {
                close(surface->dmabuf_fd);
            }
            pthread_mutex_destroy(&surface->mutex);
            pthread_cond_destroy(&surface->cond);
            free(surface);
            drv->surfaces[idx] = NULL;
        }
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_CreateContext(
    VADriverContextP ctx,
    VAConfigID config_id,
    int picture_width,
    int picture_height,
    int flag,
    VASurfaceID *render_targets,
    int num_render_targets,
    VAContextID *context_id)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Config *cfg = get_config(drv, config_id);

    if (cfg == NULL)
        return VA_STATUS_ERROR_INVALID_CONFIG;

    VAGenericID id = allocate_context_id(drv);
    if (id == VA_INVALID_ID)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    V4L2Context *context = calloc(1, sizeof(V4L2Context));
    if (context == NULL)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    context->drv = drv;
    context->profile = cfg->profile;
    context->entrypoint = cfg->entrypoint;
    context->width = picture_width;
    context->height = picture_height;
    context->codec = cfg->codec;
    context->v4l2_fd = -1;
    pthread_mutex_init(&context->mutex, NULL);

    /* Open V4L2 device */
    context->v4l2_fd = v4l2_open_device(drv);
    if (context->v4l2_fd < 0) {
        LOG("Failed to open V4L2 device");
        free(context);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    /* Setup OUTPUT queue (bitstream input) */
    /* NOTE: CAPTURE queue is setup later when streaming starts, after SOURCE_CHANGE event */
    if (v4l2_setup_output_queue(context) < 0) {
        LOG("Failed to setup OUTPUT queue");
        v4l2_close_device(drv, context->v4l2_fd);
        free(context);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    drv->contexts[CONTEXT_INDEX(id)] = context;
    *context_id = id;

    LOG("Created context %d for %s (%dx%d)", id, cfg->codec->name,
        picture_width, picture_height);
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_DestroyContext(
    VADriverContextP ctx,
    VAContextID context_id)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    int idx = CONTEXT_INDEX(context_id);

    if (idx < 0 || idx >= MAX_PROFILES || drv->contexts[idx] == NULL)
        return VA_STATUS_ERROR_INVALID_CONTEXT;

    V4L2Context *context = drv->contexts[idx];

    /* Stop streaming */
    if (context->streaming_output) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        ioctl(context->v4l2_fd, VIDIOC_STREAMOFF, &type);
        context->streaming_output = false;
    }
    if (context->streaming_capture) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        ioctl(context->v4l2_fd, VIDIOC_STREAMOFF, &type);
        context->streaming_capture = false;
    }

    /* Unmap OUTPUT buffers */
    for (int i = 0; i < context->num_output_buffers; i++) {
        if (context->output_buffers[i].start != NULL &&
            context->output_buffers[i].start != MAP_FAILED) {
            munmap(context->output_buffers[i].start,
                   context->output_buffers[i].length);
        }
    }

    /* Unmap and close CAPTURE buffers */
    for (int i = 0; i < context->num_capture_buffers; i++) {
        if (context->capture_buffers[i].plane0_ptr != NULL) {
            munmap(context->capture_buffers[i].plane0_ptr,
                   context->capture_buffers[i].plane0_len);
        }
        if (context->capture_buffers[i].plane1_ptr != NULL) {
            munmap(context->capture_buffers[i].plane1_ptr,
                   context->capture_buffers[i].plane1_len);
        }
        if (context->capture_buffers[i].fd >= 0) {
            close(context->capture_buffers[i].fd);
        }
    }

    if (context->v4l2_fd >= 0) {
        v4l2_close_device(drv, context->v4l2_fd);
    }

    bitstream_free(&context->bitstream);
    pthread_mutex_destroy(&context->mutex);
    free(context);
    drv->contexts[idx] = NULL;

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_CreateBuffer(
    VADriverContextP ctx,
    VAContextID context,
    VABufferType type,
    unsigned int size,
    unsigned int num_elements,
    void *data,
    VABufferID *buf_id)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;

    VAGenericID id = allocate_buffer_id(drv);
    if (id == VA_INVALID_ID)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    V4L2Buffer *buffer = calloc(1, sizeof(V4L2Buffer));
    if (buffer == NULL)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    buffer->type = type;
    buffer->num_elements = num_elements;
    buffer->element_size = size;
    buffer->data = malloc(size * num_elements);

    if (buffer->data == NULL) {
        free(buffer);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    if (data != NULL) {
        memcpy(buffer->data, data, size * num_elements);
    }

    drv->buffers[BUFFER_INDEX(id)] = buffer;
    *buf_id = id;

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_BufferSetNumElements(
    VADriverContextP ctx,
    VABufferID buf_id,
    unsigned int num_elements)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Buffer *buffer = get_buffer(drv, buf_id);

    if (buffer == NULL)
        return VA_STATUS_ERROR_INVALID_BUFFER;

    buffer->num_elements = num_elements;
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_MapBuffer(
    VADriverContextP ctx,
    VABufferID buf_id,
    void **pbuf)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Buffer *buffer = get_buffer(drv, buf_id);

    if (buffer == NULL)
        return VA_STATUS_ERROR_INVALID_BUFFER;

    /* Handle DeriveImage buffers - need to mmap the V4L2 CAPTURE buffer */
    if (buffer->type == VAImageBufferType && buffer->data == NULL) {
        V4L2Surface *surface = get_surface(drv, buffer->surface_id);
        if (surface == NULL || surface->context == NULL || surface->capture_idx < 0) {
            LOG("MapBuffer: Invalid surface for image buffer");
            return VA_STATUS_ERROR_INVALID_BUFFER;
        }

        V4L2Context *context = surface->context;
        int capture_idx = surface->capture_idx;
        buffer->capture_idx = capture_idx;
        buffer->in_use = true;

        /* Query buffer info to get memory offset */
        struct v4l2_buffer buf;
        struct v4l2_plane planes[2];
        memset(&buf, 0, sizeof(buf));
        memset(&planes, 0, sizeof(planes));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = capture_idx;
        buf.length = 2;
        buf.m.planes = planes;

        if (ioctl(context->v4l2_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            LOG("MapBuffer: Failed to query CAPTURE buffer: %s", strerror(errno));
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        /* mmap the buffer */
        size_t total_size = planes[0].length + (buf.length > 1 ? planes[1].length : 0);
        void *mapped = mmap(NULL, total_size, PROT_READ, MAP_SHARED,
                           context->v4l2_fd, planes[0].m.mem_offset);
        if (mapped == MAP_FAILED) {
            LOG("MapBuffer: Failed to mmap CAPTURE buffer: %s", strerror(errno));
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        buffer->data = mapped;
        buffer->element_size = total_size;
        LOG("MapBuffer: Mapped CAPTURE buffer %d at %p, size=%zu",
            capture_idx, mapped, total_size);
    }

    *pbuf = buffer->data;
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_UnmapBuffer(
    VADriverContextP ctx,
    VABufferID buf_id)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Buffer *buffer = get_buffer(drv, buf_id);

    if (buffer == NULL)
        return VA_STATUS_ERROR_INVALID_BUFFER;

    /* Unmap DeriveImage buffers - these are mmap'd directly from V4L2 */
    if (buffer->type == VAImageBufferType && buffer->surface_id != 0 &&
        buffer->data != NULL) {
        V4L2Surface *surface = get_surface(drv, buffer->surface_id);
        if (surface && surface->context && buffer->capture_idx >= 0) {
            v4l2_requeue_capture(surface->context, buffer->capture_idx);
        }
        munmap(buffer->data, buffer->element_size);
        buffer->data = NULL;
        buffer->capture_idx = -1;
        buffer->in_use = false;
        LOG("UnmapBuffer: Unmapped DeriveImage buffer %d", buf_id);
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_DestroyBuffer(
    VADriverContextP ctx,
    VABufferID buf_id)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    int idx = BUFFER_INDEX(buf_id);

    pthread_mutex_lock(&drv->mutex);
    if (idx < 0 || idx >= MAX_BUFFERS || drv->buffers[idx] == NULL) {
        pthread_mutex_unlock(&drv->mutex);
        return VA_STATUS_SUCCESS;
    }

    /* If an image buffer is still marked in use, defer free */
    if (drv->buffers[idx]->type == VAImageBufferType && drv->buffers[idx]->in_use) {
        pthread_mutex_unlock(&drv->mutex);
        LOG("DestroyBuffer: buffer %d still in use, deferring free", buf_id);
        return VA_STATUS_SUCCESS;
    }

    free(drv->buffers[idx]->data);
    free(drv->buffers[idx]);
    drv->buffers[idx] = NULL;
    pthread_mutex_unlock(&drv->mutex);

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_BeginPicture(
    VADriverContextP ctx,
    VAContextID context_id,
    VASurfaceID render_target)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Context *context = get_context(drv, context_id);
    V4L2Surface *surface = get_surface(drv, render_target);

    if (context == NULL)
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    if (surface == NULL)
        return VA_STATUS_ERROR_INVALID_SURFACE;

    pthread_mutex_lock(&context->mutex);

    /* If this surface held a previous capture buffer, return it so decoding can progress */
    if (surface->context && surface->capture_idx >= 0) {
        v4l2_requeue_capture(surface->context, surface->capture_idx);
    }
    surface->capture_idx = -1;

    /* Reset bitstream buffer for this picture */
    bitstream_reset(&context->bitstream);
    context->render_target = surface;
    context->last_slice_params = NULL;
    context->last_slice_count = 0;
    context->num_frame_buffers = 0;

    surface->context = context;
    surface->decoded = false;
    surface->no_output = false;

    pthread_mutex_unlock(&context->mutex);

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_RenderPicture(
    VADriverContextP ctx,
    VAContextID context_id,
    VABufferID *buffers,
    int num_buffers)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Context *context = get_context(drv, context_id);

    if (context == NULL)
        return VA_STATUS_ERROR_INVALID_CONTEXT;

    pthread_mutex_lock(&context->mutex);

    for (int i = 0; i < num_buffers; i++) {
        V4L2Buffer *buf = get_buffer(drv, buffers[i]);
        if (buf == NULL) {
            LOG("Invalid buffer %d", buffers[i]);
            continue;
        }

        /* Dispatch to codec-specific handler based on buffer type */
        switch (buf->type) {
        case VASliceDataBufferType:
            if (context->codec && context->codec->handle_slice_data) {
                context->codec->handle_slice_data(context, buf);
            }
            break;
        case VASliceParameterBufferType:
            /* Store for use by slice data handler */
            context->last_slice_params = buf->data;
            context->last_slice_count = buf->num_elements;
            break;
        case VAPictureParameterBufferType:
            /* Call codec-specific handler if available */
            if (context->codec && context->codec->handle_picture_params) {
                context->codec->handle_picture_params(context, buf);
            }
            break;
        case VAIQMatrixBufferType:
            /* For stateful V4L2, hardware handles IQ matrix internally */
            break;
        default:
            LOG("Unhandled buffer type: %d", buf->type);
            break;
        }
    }

    pthread_mutex_unlock(&context->mutex);
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_EndPicture(
    VADriverContextP ctx,
    VAContextID context_id)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Context *context = get_context(drv, context_id);

    if (context == NULL)
        return VA_STATUS_ERROR_INVALID_CONTEXT;

    pthread_mutex_lock(&context->mutex);

    /* Allow codec to do any final bitstream preparation */
    if (context->codec && context->codec->prepare_bitstream) {
        context->codec->prepare_bitstream(context);
    }

    /* Submit bitstream to V4L2 OUTPUT queue */
    if (context->bitstream.size > 0) {
        int ret = v4l2_queue_bitstream(context, context->bitstream.data,
                                        context->bitstream.size);
        if (ret < 0) {
            LOG("Failed to queue bitstream");
            pthread_mutex_unlock(&context->mutex);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
    }

    /* Try to dequeue a decoded frame */
    if (context->render_target) {
        v4l2_dequeue_frame(context, context->render_target);
    }

    pthread_mutex_unlock(&context->mutex);
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_SyncSurface(
    VADriverContextP ctx,
    VASurfaceID render_target)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Surface *surface = get_surface(drv, render_target);

    if (surface == NULL)
        return VA_STATUS_ERROR_INVALID_SURFACE;

    pthread_mutex_lock(&surface->mutex);

    /* If no decode context or already decoded, surface is ready */
    if (surface->context == NULL) {
        surface->decoded = true;
        pthread_mutex_unlock(&surface->mutex);
        return VA_STATUS_SUCCESS;
    }

    /* Try to dequeue from V4L2 with limited retries */
    V4L2Context *context = surface->context;
    int retries = 50;  /* 500ms max wait */

    while (!surface->decoded && retries-- > 0) {
        pthread_mutex_unlock(&surface->mutex);

        pthread_mutex_lock(&context->mutex);
        v4l2_dequeue_frame(context, surface);
        pthread_mutex_unlock(&context->mutex);

        pthread_mutex_lock(&surface->mutex);
        if (!surface->decoded) {
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += 10000000;  /* 10ms */
            if (ts.tv_nsec >= 1000000000) {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000;
            }
            pthread_cond_timedwait(&surface->cond, &surface->mutex, &ts);
        }
    }

    /* Mark as ready after timeout to prevent hangs */
    surface->decoded = true;
    pthread_mutex_unlock(&surface->mutex);

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_QuerySurfaceStatus(
    VADriverContextP ctx,
    VASurfaceID render_target,
    VASurfaceStatus *status)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Surface *surface = get_surface(drv, render_target);

    if (surface == NULL)
        return VA_STATUS_ERROR_INVALID_SURFACE;

    *status = surface->decoded ? VASurfaceReady : VASurfaceRendering;
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_QuerySurfaceError(
    VADriverContextP ctx,
    VASurfaceID surface,
    VAStatus error_status,
    void **error_info)
{
    *error_info = NULL;
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_PutSurface(
    VADriverContextP ctx,
    VASurfaceID surface,
    void *draw,
    short srcx, short srcy,
    unsigned short srcw, unsigned short srch,
    short destx, short desty,
    unsigned short destw, unsigned short desth,
    VARectangle *cliprects,
    unsigned int number_cliprects,
    unsigned int flags)
{
    /* Not implementing direct display - use DRM/DMABuf export instead */
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus v4l2_QueryImageFormats(
    VADriverContextP ctx,
    VAImageFormat *format_list,
    int *num_formats)
{
    /* Support NV12 (main output format from V4L2 decoders) */
    format_list[0].fourcc = VA_FOURCC_NV12;
    format_list[0].byte_order = VA_LSB_FIRST;
    format_list[0].bits_per_pixel = 12;
    *num_formats = 1;

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_CreateImage(
    VADriverContextP ctx,
    VAImageFormat *format,
    int width,
    int height,
    VAImage *image)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;

    /* Allocate a single ID for both image and buffer (simplifies lookup) */
    VAGenericID id = allocate_buffer_id(drv);
    if (id == VA_INVALID_ID)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    memset(image, 0, sizeof(*image));
    image->image_id = id;
    image->format = *format;
    image->width = width;
    image->height = height;

    if (format->fourcc == VA_FOURCC_NV12) {
        image->num_planes = 2;
        image->pitches[0] = width;
        image->pitches[1] = width;
        image->offsets[0] = 0;
        image->offsets[1] = width * height;
        image->data_size = width * height * 3 / 2;
    }

    /* Create buffer to hold image data */
    V4L2Buffer *buffer = calloc(1, sizeof(V4L2Buffer));
    if (buffer == NULL)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    buffer->type = VAImageBufferType;
    buffer->num_elements = 1;
    buffer->element_size = image->data_size;
    buffer->width = width;
    buffer->height = height;
    buffer->data = malloc(image->data_size);
    if (buffer->data == NULL) {
        free(buffer);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    drv->buffers[BUFFER_INDEX(id)] = buffer;
    image->buf = id;  /* Use same ID for buffer */

    LOG("CreateImage: id=%d, %dx%d NV12, data_size=%d",
        id, width, height, image->data_size);

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_DeriveImage(
    VADriverContextP ctx,
    VASurfaceID surface_id,
    VAImage *image)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Surface *surface = get_surface(drv, surface_id);

    if (surface == NULL)
        return VA_STATUS_ERROR_INVALID_SURFACE;

    LOG("DeriveImage: surface=%d, context=%p, capture_idx=%d, decoded=%d",
        surface_id, surface->context, surface->capture_idx, surface->decoded);

    /* Surface must have been decoded (associated with a V4L2 CAPTURE buffer) */
    if (surface->context == NULL) {
        LOG("DeriveImage: No context associated with surface");
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }

    if (surface->capture_idx < 0) {
        LOG("DeriveImage: Surface not decoded yet (no capture buffer)");
        return VA_STATUS_ERROR_SURFACE_BUSY;
    }

    V4L2Context *context = surface->context;

    /* Create a buffer to hold the image info */
    VAGenericID buf_id = allocate_buffer_id(drv);
    if (buf_id == VA_INVALID_ID)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    /* Setup image structure for NV12 format */
    memset(image, 0, sizeof(*image));
    image->image_id = buf_id;
    image->format.fourcc = VA_FOURCC_NV12;
    image->format.byte_order = VA_LSB_FIRST;
    image->format.bits_per_pixel = 12;
    image->width = surface->width;
    image->height = surface->height;
    image->num_planes = 2;
    image->pitches[0] = surface->width;  /* Y plane stride */
    image->pitches[1] = surface->width;  /* UV plane stride */
    image->offsets[0] = 0;
    image->offsets[1] = surface->width * surface->height;  /* UV offset */
    image->data_size = surface->width * surface->height * 3 / 2;

    /* Create a buffer object to track this */
    V4L2Buffer *buffer = calloc(1, sizeof(V4L2Buffer));
    if (buffer == NULL)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    buffer->type = VAImageBufferType;
    buffer->num_elements = 1;
    buffer->element_size = image->data_size;
    buffer->data = NULL;  /* Will be set on MapBuffer */

    /* Store the surface info for MapBuffer */
    buffer->surface_id = surface_id;

    drv->buffers[BUFFER_INDEX(buf_id)] = buffer;
    image->buf = buf_id;

    LOG("DeriveImage: Created image %d for surface %d (%dx%d NV12)",
        buf_id, surface_id, surface->width, surface->height);

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_DestroyImage(
    VADriverContextP ctx,
    VAImageID image)
{
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_SetImagePalette(
    VADriverContextP ctx,
    VAImageID image,
    unsigned char *palette)
{
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

static VAStatus v4l2_GetImage(
    VADriverContextP ctx,
    VASurfaceID surface_id,
    int x, int y,
    unsigned int width, unsigned int height,
    VAImageID image_id)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Surface *surface = get_surface(drv, surface_id);
    V4L2Buffer *image_buf = get_buffer(drv, image_id);

    if (surface == NULL) {
        LOG("GetImage: Invalid surface %d", surface_id);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }

    if (image_buf == NULL) {
        LOG("GetImage: Invalid image %d", image_id);
        return VA_STATUS_ERROR_INVALID_IMAGE;
    }

    LOG("GetImage: surface=%d, image=%d, capture_idx=%d, decoded=%d, context=%p",
        surface_id, image_id, surface->capture_idx, surface->decoded, surface->context);

    /* Surface must have been decoded */
    if (!surface->decoded || surface->context == NULL) {
        LOG("GetImage: Surface not decoded yet");
        return VA_STATUS_ERROR_SURFACE_BUSY;
    }

    V4L2Context *context = surface->context;

    if (surface->capture_idx < 0 || surface->capture_idx >= context->num_capture_buffers) {
        LOG("GetImage: Invalid capture_idx %d", surface->capture_idx);
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }

    /* Get cached mmap or create new one */
    V4L2MmapBuffer *cap_buf = &context->capture_buffers[surface->capture_idx];
    void *y_plane = cap_buf->plane0_ptr;
    void *uv_plane = cap_buf->plane1_ptr;

    if (y_plane == NULL || uv_plane == NULL) {
        /* Need to mmap the CAPTURE buffer planes */
        struct v4l2_buffer buf;
        struct v4l2_plane planes[2];
        memset(&buf, 0, sizeof(buf));
        memset(&planes, 0, sizeof(planes));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = surface->capture_idx;
        buf.length = 2;  /* NM12 has 2 planes */
        buf.m.planes = planes;

        if (ioctl(context->v4l2_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            LOG("GetImage: Failed to query CAPTURE buffer: %s", strerror(errno));
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        y_plane = mmap(NULL, planes[0].length, PROT_READ, MAP_SHARED,
                       context->v4l2_fd, planes[0].m.mem_offset);
        if (y_plane == MAP_FAILED) {
            LOG("GetImage: Failed to mmap Y plane: %s", strerror(errno));
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        uv_plane = mmap(NULL, planes[1].length, PROT_READ, MAP_SHARED,
                        context->v4l2_fd, planes[1].m.mem_offset);
        if (uv_plane == MAP_FAILED) {
            munmap(y_plane, planes[0].length);
            LOG("GetImage: Failed to mmap UV plane: %s", strerror(errno));
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        /* Cache the mmap pointers */
        cap_buf->plane0_ptr = y_plane;
        cap_buf->plane1_ptr = uv_plane;
        cap_buf->plane0_len = planes[0].length;
        cap_buf->plane1_len = planes[1].length;

        LOG("GetImage: Cached mmap for buffer %d (Y=%zu, UV=%zu)",
            surface->capture_idx, planes[0].length, planes[1].length);
    }

    /* Use image buffer dimensions for copy */
    size_t img_width = image_buf->width ? image_buf->width : surface->width;
    size_t img_height = image_buf->height ? image_buf->height : surface->height;
    size_t y_size = img_width * img_height;
    size_t uv_size = y_size / 2;

    /* Copy Y plane */
    memcpy(image_buf->data, y_plane, y_size);
    /* Copy UV plane */
    memcpy((uint8_t *)image_buf->data + y_size, uv_plane, uv_size);

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_PutImage(
    VADriverContextP ctx,
    VASurfaceID surface,
    VAImageID image,
    int src_x, int src_y,
    unsigned int src_width, unsigned int src_height,
    int dest_x, int dest_y,
    unsigned int dest_width, unsigned int dest_height)
{
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

/* Subpicture stubs */
static VAStatus v4l2_QuerySubpictureFormats(
    VADriverContextP ctx,
    VAImageFormat *format_list,
    unsigned int *flags,
    unsigned int *num_formats)
{
    *num_formats = 0;
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_CreateSubpicture(VADriverContextP ctx, VAImageID image, VASubpictureID *subpicture)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_DestroySubpicture(VADriverContextP ctx, VASubpictureID subpicture)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_SetSubpictureImage(VADriverContextP ctx, VASubpictureID subpicture, VAImageID image)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_SetSubpictureChromakey(VADriverContextP ctx, VASubpictureID subpicture,
    unsigned int chromakey_min, unsigned int chromakey_max, unsigned int chromakey_mask)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_SetSubpictureGlobalAlpha(VADriverContextP ctx, VASubpictureID subpicture, float global_alpha)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_AssociateSubpicture(VADriverContextP ctx, VASubpictureID subpicture,
    VASurfaceID *target_surfaces, int num_surfaces,
    short src_x, short src_y, unsigned short src_width, unsigned short src_height,
    short dest_x, short dest_y, unsigned short dest_width, unsigned short dest_height, unsigned int flags)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_DeassociateSubpicture(VADriverContextP ctx, VASubpictureID subpicture,
    VASurfaceID *target_surfaces, int num_surfaces)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

/* Display attribute stubs */
static VAStatus v4l2_QueryDisplayAttributes(VADriverContextP ctx, VADisplayAttribute *attr_list, int *num_attributes)
{ *num_attributes = 0; return VA_STATUS_SUCCESS; }

static VAStatus v4l2_GetDisplayAttributes(VADriverContextP ctx, VADisplayAttribute *attr_list, int num_attributes)
{ return VA_STATUS_SUCCESS; }

static VAStatus v4l2_SetDisplayAttributes(VADriverContextP ctx, VADisplayAttribute *attr_list, int num_attributes)
{ return VA_STATUS_SUCCESS; }

static VAStatus v4l2_QuerySurfaceAttributes(
    VADriverContextP ctx,
    VAConfigID config,
    VASurfaceAttrib *attrib_list,
    unsigned int *num_attribs)
{
    if (attrib_list == NULL) {
        *num_attribs = 4;
        return VA_STATUS_SUCCESS;
    }

    int i = 0;

    /* Memory type */
    attrib_list[i].type = VASurfaceAttribMemoryType;
    attrib_list[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    attrib_list[i].value.type = VAGenericValueTypeInteger;
    attrib_list[i].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_VA |
                                   VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
    i++;

    /* Pixel format */
    attrib_list[i].type = VASurfaceAttribPixelFormat;
    attrib_list[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    attrib_list[i].value.type = VAGenericValueTypeInteger;
    attrib_list[i].value.value.i = VA_FOURCC_NV12;
    i++;

    /* Min/max dimensions */
    attrib_list[i].type = VASurfaceAttribMinWidth;
    attrib_list[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
    attrib_list[i].value.type = VAGenericValueTypeInteger;
    attrib_list[i].value.value.i = 16;
    i++;

    attrib_list[i].type = VASurfaceAttribMaxWidth;
    attrib_list[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
    attrib_list[i].value.type = VAGenericValueTypeInteger;
    attrib_list[i].value.value.i = 4096;
    i++;

    *num_attribs = i;
    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_BufferInfo(VADriverContextP ctx, VABufferID buf_id,
    VABufferType *type, unsigned int *size, unsigned int *num_elements)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Buffer *buffer = get_buffer(drv, buf_id);

    if (buffer == NULL)
        return VA_STATUS_ERROR_INVALID_BUFFER;

    if (type) *type = buffer->type;
    if (size) *size = buffer->element_size;
    if (num_elements) *num_elements = buffer->num_elements;

    return VA_STATUS_SUCCESS;
}

static VAStatus v4l2_AcquireBufferHandle(VADriverContextP ctx, VABufferID buf_id,
    VABufferInfo *buf_info)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_ReleaseBufferHandle(VADriverContextP ctx, VABufferID buf_id)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_LockSurface(VADriverContextP ctx, VASurfaceID surface,
    unsigned int *fourcc, unsigned int *luma_stride, unsigned int *chroma_u_stride,
    unsigned int *chroma_v_stride, unsigned int *luma_offset, unsigned int *chroma_u_offset,
    unsigned int *chroma_v_offset, unsigned int *buffer_name, void **buffer)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_UnlockSurface(VADriverContextP ctx, VASurfaceID surface)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

/* MF (Multi-Frame) stubs */
static VAStatus v4l2_CreateMFContext(VADriverContextP ctx, VAMFContextID *mf_context)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_MFAddContext(VADriverContextP ctx, VAMFContextID mf_context, VAContextID context)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_MFReleaseContext(VADriverContextP ctx, VAMFContextID mf_context, VAContextID context)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_MFSubmit(VADriverContextP ctx, VAMFContextID mf_context, VAContextID *contexts, int num_contexts)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_CreateBuffer2(VADriverContextP ctx, VAContextID context,
    VABufferType type, unsigned int width, unsigned int height,
    unsigned int *unit_size, unsigned int *pitch, VABufferID *buf_id)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_QueryProcessingRate(VADriverContextP ctx, VAConfigID config,
    VAProcessingRateParameter *proc_buf, unsigned int *processing_rate)
{ return VA_STATUS_ERROR_UNIMPLEMENTED; }

static VAStatus v4l2_ExportSurfaceHandle(
    VADriverContextP ctx,
    VASurfaceID surface_id,
    uint32_t mem_type,
    uint32_t flags,
    void *descriptor)
{
    V4L2Driver *drv = (V4L2Driver *)ctx->pDriverData;
    V4L2Surface *surface = get_surface(drv, surface_id);

    if (surface == NULL)
        return VA_STATUS_ERROR_INVALID_SURFACE;

    if (mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME &&
        mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2) {
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    }

    if (surface->context == NULL || surface->capture_idx < 0)
        return VA_STATUS_ERROR_INVALID_SURFACE;

    /* Export DMABuf from V4L2 CAPTURE buffer */
    int fd = v4l2_export_dmabuf(surface->context, surface->capture_idx);
    if (fd < 0)
        return VA_STATUS_ERROR_OPERATION_FAILED;

    if (mem_type == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME) {
        /* Simple DRM PRIME export - just return the fd */
        VADRMPRIMESurfaceDescriptor *desc = (VADRMPRIMESurfaceDescriptor *)descriptor;
        memset(desc, 0, sizeof(*desc));
        desc->fourcc = VA_FOURCC_NV12;
        desc->width = surface->width;
        desc->height = surface->height;
        desc->num_objects = 1;
        desc->objects[0].fd = fd;
        desc->objects[0].size = surface->width * surface->height * 3 / 2;
        desc->objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR;
        desc->num_layers = 2;
        /* Y plane */
        desc->layers[0].drm_format = DRM_FORMAT_R8;
        desc->layers[0].num_planes = 1;
        desc->layers[0].object_index[0] = 0;
        desc->layers[0].offset[0] = 0;
        desc->layers[0].pitch[0] = surface->width;
        /* UV plane */
        desc->layers[1].drm_format = DRM_FORMAT_RG88;
        desc->layers[1].num_planes = 1;
        desc->layers[1].object_index[0] = 0;
        desc->layers[1].offset[0] = surface->width * surface->height;
        desc->layers[1].pitch[0] = surface->width;
    }

    return VA_STATUS_SUCCESS;
}

/* Macro to simplify vtable setup */
#define VTABLE(func) .va##func = v4l2_##func

static const struct VADriverVTable vtable = {
    VTABLE(Terminate),
    VTABLE(QueryConfigProfiles),
    VTABLE(QueryConfigEntrypoints),
    VTABLE(QueryConfigAttributes),
    VTABLE(CreateConfig),
    VTABLE(DestroyConfig),
    VTABLE(GetConfigAttributes),
    VTABLE(CreateSurfaces),
    VTABLE(CreateSurfaces2),
    VTABLE(DestroySurfaces),
    VTABLE(CreateContext),
    VTABLE(DestroyContext),
    VTABLE(CreateBuffer),
    VTABLE(BufferSetNumElements),
    VTABLE(MapBuffer),
    VTABLE(UnmapBuffer),
    VTABLE(DestroyBuffer),
    VTABLE(BeginPicture),
    VTABLE(RenderPicture),
    VTABLE(EndPicture),
    VTABLE(SyncSurface),
    VTABLE(QuerySurfaceStatus),
    VTABLE(QuerySurfaceError),
    VTABLE(PutSurface),
    VTABLE(QueryImageFormats),
    VTABLE(CreateImage),
    VTABLE(DeriveImage),
    VTABLE(DestroyImage),
    VTABLE(SetImagePalette),
    VTABLE(GetImage),
    VTABLE(PutImage),
    VTABLE(QuerySubpictureFormats),
    VTABLE(CreateSubpicture),
    VTABLE(DestroySubpicture),
    VTABLE(SetSubpictureImage),
    VTABLE(SetSubpictureChromakey),
    VTABLE(SetSubpictureGlobalAlpha),
    VTABLE(AssociateSubpicture),
    VTABLE(DeassociateSubpicture),
    VTABLE(QueryDisplayAttributes),
    VTABLE(GetDisplayAttributes),
    VTABLE(SetDisplayAttributes),
    VTABLE(QuerySurfaceAttributes),
    VTABLE(BufferInfo),
    VTABLE(AcquireBufferHandle),
    VTABLE(ReleaseBufferHandle),
    VTABLE(LockSurface),
    VTABLE(UnlockSurface),
    VTABLE(CreateMFContext),
    VTABLE(MFAddContext),
    VTABLE(MFReleaseContext),
    VTABLE(MFSubmit),
    VTABLE(CreateBuffer2),
    VTABLE(QueryProcessingRate),
    VTABLE(ExportSurfaceHandle),
};

/*
 * Driver initialization entry point
 * Called by libva when loading the driver
 */
VAStatus __vaDriverInit_1_0(VADriverContextP ctx);

__attribute__((visibility("default")))
VAStatus __vaDriverInit_1_0(VADriverContextP ctx)
{
    LOG("Initializing V4L2 Stateful VA-API Driver");

    V4L2Driver *drv = calloc(1, sizeof(V4L2Driver));
    if (drv == NULL)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    ctx->pDriverData = drv;
    pthread_mutex_init(&drv->mutex, NULL);

    /* Get DRM fd if provided */
    if (ctx->drm_state != NULL) {
        struct drm_state *drm = (struct drm_state *)ctx->drm_state;
        drv->drm_fd = drm->fd;
    } else {
        drv->drm_fd = -1;
    }

    /* Probe V4L2 device capabilities */
    int fd = v4l2_open_device(drv);
    if (fd < 0) {
        LOG("No V4L2 M2M decoder found");
        free(drv);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    v4l2_probe_capabilities(drv, fd);
    v4l2_close_device(drv, fd);

    if (drv->num_supported_profiles == 0) {
        LOG("No supported profiles found");
        free(drv);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    /* Setup VA-API context */
    ctx->max_profiles = MAX_PROFILES;
    ctx->max_entrypoints = 1;
    ctx->max_attributes = 8;
    ctx->max_display_attributes = 1;
    ctx->max_image_formats = 2;
    ctx->max_subpic_formats = 1;  /* Must be at least 1 for libva */
    ctx->str_vendor = "VA-API V4L2 Stateful driver for CIX Sky1";

    *ctx->vtable = vtable;

    LOG("Driver initialized with %d profiles", drv->num_supported_profiles);
    return VA_STATUS_SUCCESS;
}
