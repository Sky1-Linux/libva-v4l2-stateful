/*
 * Surface management for VA-API to V4L2 stateful backend
 *
 * VA-API surfaces map to V4L2 CAPTURE buffers.
 * When a frame is decoded, it's available in a CAPTURE buffer.
 * We export these as DMABuf for zero-copy sharing with display/compositor.
 */

#include "vabackend.h"
#include <string.h>

/*
 * Additional surface utility functions can be added here.
 * The main surface management is in vabackend.c for now.
 */
