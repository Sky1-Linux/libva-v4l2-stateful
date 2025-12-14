/*
 * V4L2 Memory-to-Memory (Stateful) Backend
 *
 * This implements the interface to V4L2 stateful video decoders.
 * The key advantage of stateful decoders is simplicity:
 * - Submit raw compressed bitstream to OUTPUT queue
 * - Receive decoded frames from CAPTURE queue
 * - Hardware handles all parsing internally
 */

#define _GNU_SOURCE
#include "vabackend.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

/* CIX Sky1 VPU fourcc values */
#define V4L2_PIX_FMT_AV1  v4l2_fourcc('A', 'V', '0', '1')

/*
 * Open V4L2 decoder device
 * Returns file descriptor or -1 on error
 */
int v4l2_open_device(V4L2Driver *drv)
{
    /* Try common decoder device paths */
    const char *device_paths[] = {
        "/dev/video0",
        "/dev/video-dec0",
        NULL
    };

    for (int i = 0; device_paths[i]; i++) {
        int fd = open(device_paths[i], O_RDWR | O_NONBLOCK);
        if (fd < 0)
            continue;

        /* Verify it's a M2M decoder */
        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
            close(fd);
            continue;
        }

        /* Check for M2M capability */
        if (!(cap.capabilities & V4L2_CAP_VIDEO_M2M_MPLANE) &&
            !(cap.capabilities & V4L2_CAP_VIDEO_M2M)) {
            close(fd);
            continue;
        }

        LOG("Opened V4L2 device: %s (%s)", device_paths[i], cap.card);
        strncpy(drv->v4l2_device, device_paths[i], sizeof(drv->v4l2_device) - 1);
        return fd;
    }

    LOG("No V4L2 M2M decoder found");
    return -1;
}

void v4l2_close_device(V4L2Driver *drv, int fd)
{
    if (fd >= 0) {
        close(fd);
    }
    (void)drv;
}

/*
 * Probe V4L2 device capabilities
 * Detects supported codecs and populates drv->supported_profiles
 */
int v4l2_probe_capabilities(V4L2Driver *drv, int fd)
{
    struct v4l2_fmtdesc fmtdesc;
    int num_profiles = 0;

    memset(&fmtdesc, 0, sizeof(fmtdesc));
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;

    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        LOG("Found V4L2 format: %s (0x%08x)", fmtdesc.description, fmtdesc.pixelformat);

        /* Map V4L2 formats to VA-API profiles */
        switch (fmtdesc.pixelformat) {
        case V4L2_PIX_FMT_H264:
        case V4L2_PIX_FMT_H264_SLICE:
            drv->supported_profiles[num_profiles++] = VAProfileH264ConstrainedBaseline;
            drv->supported_profiles[num_profiles++] = VAProfileH264Main;
            drv->supported_profiles[num_profiles++] = VAProfileH264High;
            break;
        case V4L2_PIX_FMT_HEVC:
            drv->supported_profiles[num_profiles++] = VAProfileHEVCMain;
            drv->supported_profiles[num_profiles++] = VAProfileHEVCMain10;
            break;
        case V4L2_PIX_FMT_VP8:
            drv->supported_profiles[num_profiles++] = VAProfileVP8Version0_3;
            break;
        case V4L2_PIX_FMT_VP9:
            drv->supported_profiles[num_profiles++] = VAProfileVP9Profile0;
            drv->supported_profiles[num_profiles++] = VAProfileVP9Profile2;
            break;
        case V4L2_PIX_FMT_AV1:
            drv->supported_profiles[num_profiles++] = VAProfileAV1Profile0;
            break;
        case V4L2_PIX_FMT_MPEG2:
            drv->supported_profiles[num_profiles++] = VAProfileMPEG2Main;
            break;
        case V4L2_PIX_FMT_MPEG4:
            drv->supported_profiles[num_profiles++] = VAProfileMPEG4AdvancedSimple;
            break;
        default:
            break;
        }

        fmtdesc.index++;
    }

    drv->num_supported_profiles = num_profiles;
    LOG("Detected %d supported VA-API profiles", num_profiles);
    return 0;
}

/*
 * Setup OUTPUT queue (compressed bitstream input)
 */
int v4l2_setup_output_queue(V4L2Context *ctx)
{
    /* Subscribe to source change events (required for proper decoder operation) */
    struct v4l2_event_subscription sub;
    memset(&sub, 0, sizeof(sub));
    sub.type = V4L2_EVENT_SOURCE_CHANGE;
    if (ioctl(ctx->v4l2_fd, VIDIOC_SUBSCRIBE_EVENT, &sub) < 0) {
        LOG("Failed to subscribe to SOURCE_CHANGE event: %s (continuing anyway)", strerror(errno));
    } else {
        LOG("Subscribed to SOURCE_CHANGE events");
    }

    /* Also subscribe to EOS events */
    memset(&sub, 0, sizeof(sub));
    sub.type = V4L2_EVENT_EOS;
    if (ioctl(ctx->v4l2_fd, VIDIOC_SUBSCRIBE_EVENT, &sub) < 0) {
        LOG("Failed to subscribe to EOS event: %s (continuing anyway)", strerror(errno));
    }

    /* Set OUTPUT format */
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    fmt.fmt.pix_mp.width = ctx->width;
    fmt.fmt.pix_mp.height = ctx->height;
    fmt.fmt.pix_mp.pixelformat = ctx->codec->v4l2_pixfmt;
    fmt.fmt.pix_mp.num_planes = 1;
    fmt.fmt.pix_mp.plane_fmt[0].sizeimage = BITSTREAM_BUFFER_SIZE;

    LOG("Setting OUTPUT format: %dx%d, pixfmt=0x%08x (%c%c%c%c)",
        ctx->width, ctx->height, ctx->codec->v4l2_pixfmt,
        (ctx->codec->v4l2_pixfmt) & 0xff,
        (ctx->codec->v4l2_pixfmt >> 8) & 0xff,
        (ctx->codec->v4l2_pixfmt >> 16) & 0xff,
        (ctx->codec->v4l2_pixfmt >> 24) & 0xff);

    if (ioctl(ctx->v4l2_fd, VIDIOC_S_FMT, &fmt) < 0) {
        LOG("Failed to set OUTPUT format: %s", strerror(errno));
        return -1;
    }

    LOG("OUTPUT format set successfully");

    /* Request OUTPUT buffers */
    struct v4l2_requestbuffers reqbufs;
    memset(&reqbufs, 0, sizeof(reqbufs));
    reqbufs.count = MAX_OUTPUT_BUFFERS;
    reqbufs.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    reqbufs.memory = V4L2_MEMORY_MMAP;

    if (ioctl(ctx->v4l2_fd, VIDIOC_REQBUFS, &reqbufs) < 0) {
        LOG("Failed to request OUTPUT buffers: %s", strerror(errno));
        return -1;
    }

    ctx->num_output_buffers = reqbufs.count;
    LOG("Allocated %d OUTPUT buffers", ctx->num_output_buffers);

    /* mmap OUTPUT buffers */
    for (int i = 0; i < ctx->num_output_buffers; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[1];

        memset(&buf, 0, sizeof(buf));
        memset(&planes, 0, sizeof(planes));
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.length = 1;
        buf.m.planes = planes;

        if (ioctl(ctx->v4l2_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            LOG("Failed to query OUTPUT buffer %d: %s", i, strerror(errno));
            return -1;
        }

        ctx->output_buffers[i].length = planes[0].length;
        ctx->output_buffers[i].start = mmap(NULL, planes[0].length,
                                            PROT_READ | PROT_WRITE, MAP_SHARED,
                                            ctx->v4l2_fd, planes[0].m.mem_offset);
        ctx->output_buffers[i].index = i;
        ctx->output_buffers[i].queued = false;

        if (ctx->output_buffers[i].start == MAP_FAILED) {
            LOG("Failed to mmap OUTPUT buffer %d: %s", i, strerror(errno));
            return -1;
        }
    }

    return 0;
}

/*
 * Setup CAPTURE queue (decoded frame output)
 */
int v4l2_setup_capture_queue(V4L2Context *ctx)
{
    /* Get CAPTURE format (negotiated by driver) */
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    if (ioctl(ctx->v4l2_fd, VIDIOC_G_FMT, &fmt) < 0) {
        /* Set a default if not available - use YU12 like FFmpeg does */
        fmt.fmt.pix_mp.width = ctx->width;
        fmt.fmt.pix_mp.height = ctx->height;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_YUV420;  /* YU12 */
        fmt.fmt.pix_mp.num_planes = 1;

        LOG("Setting CAPTURE format: %dx%d YU12", ctx->width, ctx->height);
        if (ioctl(ctx->v4l2_fd, VIDIOC_S_FMT, &fmt) < 0) {
            LOG("Failed to set CAPTURE format: %s", strerror(errno));
            return -1;
        }
    } else {
        LOG("Got CAPTURE format: %dx%d pixfmt=0x%08x",
            fmt.fmt.pix_mp.width, fmt.fmt.pix_mp.height, fmt.fmt.pix_mp.pixelformat);
    }

    /* Request CAPTURE buffers with DMABUF export */
    struct v4l2_requestbuffers reqbufs;
    memset(&reqbufs, 0, sizeof(reqbufs));
    reqbufs.count = MAX_CAPTURE_BUFFERS;
    reqbufs.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    reqbufs.memory = V4L2_MEMORY_MMAP;

    if (ioctl(ctx->v4l2_fd, VIDIOC_REQBUFS, &reqbufs) < 0) {
        LOG("Failed to request CAPTURE buffers: %s", strerror(errno));
        return -1;
    }

    ctx->num_capture_buffers = reqbufs.count;
    LOG("Allocated %d CAPTURE buffers", ctx->num_capture_buffers);

    /* Queue all CAPTURE buffers */
    for (int i = 0; i < ctx->num_capture_buffers; i++) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[2];

        memset(&buf, 0, sizeof(buf));
        memset(&planes, 0, sizeof(planes));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.length = 2;
        buf.m.planes = planes;

        if (ioctl(ctx->v4l2_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            LOG("Failed to query CAPTURE buffer %d: %s", i, strerror(errno));
            /* Continue anyway - might be single plane */
        }

        ctx->capture_buffers[i].index = i;
        ctx->capture_buffers[i].fd = -1;
        ctx->capture_buffers[i].queued = false;

        /* Queue the buffer */
        buf.index = i;
        if (ioctl(ctx->v4l2_fd, VIDIOC_QBUF, &buf) < 0) {
            LOG("Failed to queue CAPTURE buffer %d: %s", i, strerror(errno));
        } else {
            ctx->capture_buffers[i].queued = true;
        }
    }

    LOG("Queued %d CAPTURE buffers", ctx->num_capture_buffers);
    return 0;
}

/*
 * Dequeue any completed OUTPUT buffers to make them available for reuse
 */
static void v4l2_reclaim_output_buffers(V4L2Context *ctx)
{
    if (!ctx->streaming_output)
        return;

    struct v4l2_buffer buf;
    struct v4l2_plane planes[1];

    while (1) {
        memset(&buf, 0, sizeof(buf));
        memset(&planes, 0, sizeof(planes));
        buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.length = 1;
        buf.m.planes = planes;

        if (ioctl(ctx->v4l2_fd, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN)
                break;  /* No more completed buffers */
            LOG("Error dequeuing OUTPUT buffer: %s", strerror(errno));
            break;
        }

        /* Mark buffer as available */
        if (buf.index < MAX_OUTPUT_BUFFERS) {
            ctx->output_buffers[buf.index].queued = false;
        }
    }
}

/*
 * Queue bitstream data for decoding
 */
int v4l2_queue_bitstream(V4L2Context *ctx, void *data, size_t size)
{
    /* First try to reclaim any completed OUTPUT buffers */
    v4l2_reclaim_output_buffers(ctx);

    /* Find available OUTPUT buffer */
    int buf_idx = -1;
    int queued_count = 0;
    for (int i = 0; i < ctx->num_output_buffers; i++) {
        if (ctx->output_buffers[i].queued) {
            queued_count++;
        } else if (buf_idx < 0) {
            buf_idx = i;
        }
    }

    LOG("Queue bitstream: size=%zu, available=%d/%d, streaming=%d",
        size, ctx->num_output_buffers - queued_count, ctx->num_output_buffers,
        ctx->streaming_output);

    /* If no buffer available and streaming, wait for one to become available */
    if (buf_idx < 0 && ctx->streaming_output) {
        struct v4l2_buffer buf;
        struct v4l2_plane planes[1];
        int retries = 100;  /* 1 second max wait */

        while (buf_idx < 0 && retries-- > 0) {
            memset(&buf, 0, sizeof(buf));
            memset(&planes, 0, sizeof(planes));
            buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.length = 1;
            buf.m.planes = planes;

            if (ioctl(ctx->v4l2_fd, VIDIOC_DQBUF, &buf) == 0) {
                if (buf.index < MAX_OUTPUT_BUFFERS) {
                    ctx->output_buffers[buf.index].queued = false;
                    buf_idx = buf.index;
                    LOG("Reclaimed OUTPUT buffer %d after wait", buf_idx);
                }
            } else if (errno == EAGAIN) {
                /* No buffer ready, wait a bit */
                usleep(10000);  /* 10ms */
            } else {
                LOG("Error waiting for OUTPUT buffer: %s", strerror(errno));
                break;
            }
        }
    }

    if (buf_idx < 0) {
        LOG("No available OUTPUT buffer (all %d queued)", ctx->num_output_buffers);
        return -1;
    }

    /* Copy bitstream to buffer */
    V4L2MmapBuffer *outbuf = &ctx->output_buffers[buf_idx];
    if (size > outbuf->length) {
        LOG("Bitstream too large: %zu > %zu", size, outbuf->length);
        return -1;
    }
    memcpy(outbuf->start, data, size);

    /* Queue buffer */
    struct v4l2_buffer buf;
    struct v4l2_plane planes[1];

    memset(&buf, 0, sizeof(buf));
    memset(&planes, 0, sizeof(planes));
    buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = buf_idx;
    buf.length = 1;
    buf.m.planes = planes;
    planes[0].bytesused = size;

    if (ioctl(ctx->v4l2_fd, VIDIOC_QBUF, &buf) < 0) {
        LOG("Failed to queue OUTPUT buffer: %s", strerror(errno));
        return -1;
    }

    outbuf->queued = true;

    /* Start OUTPUT streaming if not already */
    if (!ctx->streaming_output) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        if (ioctl(ctx->v4l2_fd, VIDIOC_STREAMON, &type) < 0) {
            LOG("Failed to start OUTPUT streaming: %s", strerror(errno));
            return -1;
        }
        ctx->streaming_output = true;
        LOG("Started OUTPUT streaming");

        /* Wait for SOURCE_CHANGE event to setup CAPTURE queue */
        struct v4l2_event ev;
        int wait_count = 0;
        bool got_source_change = false;

        while (wait_count < 100) {  /* Wait up to 1 second */
            memset(&ev, 0, sizeof(ev));
            if (ioctl(ctx->v4l2_fd, VIDIOC_DQEVENT, &ev) == 0) {
                LOG("Got V4L2 event type=%d", ev.type);
                if (ev.type == V4L2_EVENT_SOURCE_CHANGE) {
                    LOG("SOURCE_CHANGE event: changes=0x%x", ev.u.src_change.changes);
                    got_source_change = true;
                    break;
                }
            } else if (errno == ENOENT) {
                /* No event pending, wait a bit */
                usleep(10000);  /* 10ms */
                wait_count++;
            } else {
                LOG("Error dequeuing event: %s", strerror(errno));
                break;
            }
        }

        if (!got_source_change) {
            LOG("No SOURCE_CHANGE event received, setting up CAPTURE anyway");
        }

        /* Now setup CAPTURE queue with decoder-negotiated format */
        if (v4l2_setup_capture_queue(ctx) < 0) {
            LOG("Failed to setup CAPTURE queue");
            return -1;
        }

        type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        if (ioctl(ctx->v4l2_fd, VIDIOC_STREAMON, &type) < 0) {
            LOG("Failed to start CAPTURE streaming: %s", strerror(errno));
            return -1;
        }
        ctx->streaming_capture = true;
        LOG("Started CAPTURE streaming");
    }

    return 0;
}

/*
 * Dequeue decoded frame
 */
int v4l2_dequeue_frame(V4L2Context *ctx, V4L2Surface *surface)
{
    struct v4l2_buffer buf;
    struct v4l2_plane planes[2];

    memset(&buf, 0, sizeof(buf));
    memset(&planes, 0, sizeof(planes));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.length = 2;
    buf.m.planes = planes;

    if (ioctl(ctx->v4l2_fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno != EAGAIN) {
            LOG("Failed to dequeue CAPTURE buffer: %s", strerror(errno));
        }
        return -1;
    }

    surface->capture_idx = buf.index;
    surface->decoded = true;
    ctx->capture_buffers[buf.index].queued = false;

    return 0;
}

/*
 * Export CAPTURE buffer as DMABuf
 */
int v4l2_export_dmabuf(V4L2Context *ctx, int capture_idx)
{
    struct v4l2_exportbuffer expbuf;
    memset(&expbuf, 0, sizeof(expbuf));
    expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    expbuf.index = capture_idx;
    expbuf.plane = 0;
    expbuf.flags = O_RDONLY | O_CLOEXEC;

    if (ioctl(ctx->v4l2_fd, VIDIOC_EXPBUF, &expbuf) < 0) {
        LOG("Failed to export DMABuf: %s", strerror(errno));
        return -1;
    }

    ctx->capture_buffers[capture_idx].fd = expbuf.fd;
    return expbuf.fd;
}

/*
 * Utility: Append to growable buffer
 */
void bitstream_append(BitstreamBuffer *bb, const void *data, size_t size)
{
    if (bb->size + size > bb->allocated) {
        size_t new_size = bb->allocated ? bb->allocated * 2 : BITSTREAM_BUFFER_SIZE;
        while (new_size < bb->size + size)
            new_size *= 2;
        bb->data = realloc(bb->data, new_size);
        bb->allocated = new_size;
    }
    memcpy((uint8_t *)bb->data + bb->size, data, size);
    bb->size += size;
}

void bitstream_reset(BitstreamBuffer *bb)
{
    bb->size = 0;
}

void bitstream_free(BitstreamBuffer *bb)
{
    free(bb->data);
    bb->data = NULL;
    bb->size = 0;
    bb->allocated = 0;
}
