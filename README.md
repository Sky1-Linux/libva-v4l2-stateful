# libva-v4l2-stateful

VA-API driver for V4L2 stateful video decoders.

Enables hardware video decode in VA-API-only applications (Firefox, Chromium, mpv) on platforms with V4L2 stateful codec interfaces.

## Target Hardware

- CIX Sky1 VPU (ARM Linlon MVE v8)
- Radxa Orion O6

## Supported Codecs

| Codec | Decode |
|-------|--------|
| H.264 | Yes |
| HEVC | Yes |
| VP8 | Yes |
| VP9 | Yes |
| AV1 | Yes |

## Installation

```bash
# From Sky1 Linux APT repository
sudo apt install libva-v4l2-stateful

# Or build from source
meson setup builddir
ninja -C builddir
sudo ninja -C builddir install
```

## Usage

```bash
# Test with vainfo
LIBVA_DRIVER_NAME=v4l2 vainfo

# mpv with hardware decode
LIBVA_DRIVER_NAME=v4l2 mpv --hwdec=vaapi-copy video.mp4

# Set as default driver
echo "LIBVA_DRIVER_NAME=v4l2" | sudo tee -a /etc/environment
```

## Current Status

- **Working**: `vaapi-copy` mode (hardware decode with CPU readback)
- **TODO**: Zero-copy display via DMABuf export

## Building

### Dependencies

```bash
sudo apt install meson ninja-build libva-dev libdrm-dev
```

### Build

```bash
meson setup builddir
ninja -C builddir
```

Output: `builddir/v4l2_drv_video.so`

## License

MIT
