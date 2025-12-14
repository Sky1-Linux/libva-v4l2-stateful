/*
 * Buffer management for VA-API to V4L2 stateful backend
 *
 * VA-API buffers hold:
 * - Picture parameters (VAPictureParameterBuffer*)
 * - Slice parameters (VASliceParameterBuffer*)
 * - Slice data (compressed bitstream)
 * - IQ matrices, etc.
 *
 * For stateful V4L2, we mainly care about slice data which contains
 * the raw bitstream to be passed to the hardware decoder.
 */

#include "vabackend.h"
#include <string.h>

/*
 * Additional buffer utility functions can be added here.
 * The main buffer management is in vabackend.c for now.
 */
