project(
    'libva-v4l2-stateful',
    'c',
    version: '0.1.0',
    default_options: [
        'prefix=/usr',
        'c_std=c11',
        'warning_level=2',
    ],
    license: 'MIT',
    meson_version: '>= 0.58.0',
)

cc = meson.get_compiler('c')

deps = [
    cc.find_library('m'),
    cc.find_library('dl', required : false),
    dependency('libdrm', version: '>=2.4.60'),
    dependency('threads'),
]
libva_deps = dependency('libva', version: '>= 1.8.0').partial_dependency(compile_args: true)
deps += [libva_deps]

gst_codecs_deps = dependency('gstreamer-codecparsers-1.0', required: false)
if gst_codecs_deps.found()
    deps += [gst_codecs_deps]
    add_project_arguments('-DHAVE_GST_CODECPARSERS=1', language: 'c')
endif

if cc.get_argument_syntax() == 'gcc'
    add_project_arguments(
        cc.get_supported_arguments([
            '-Wno-missing-field-initializers',
            '-Wno-unused-parameter',
            '-Werror=format',
            '-Werror=format-security',
            '-Werror=incompatible-pointer-types',
            '-Werror=init-self',
            '-Werror=int-conversion',
            '-Werror=missing-declarations',
            '-Werror=missing-prototypes',
            '-Werror=pointer-arith',
            '-Werror=undef',
            '-Werror=vla',
        ]),
        language: ['c'],
    )
endif

sources = [
    'src/vabackend.c',
    'src/v4l2-backend.c',
    'src/h264.c',
    'src/hevc.c',
    'src/vp8.c',
    'src/vp9.c',
    'src/surface.c',
    'src/buffer.c',
]

install_dir = libva_deps.get_variable(pkgconfig: 'driverdir')

shared_library(
    'v4l2_drv_video',
    name_prefix: '',
    sources: sources,
    dependencies: deps,
    install: true,
    install_dir: install_dir,
    gnu_symbol_visibility: 'hidden',
)

meson.add_devenv(environment({
    'V4L2VA_LOG': '1',
    'LIBVA_DRIVER_NAME': 'v4l2',
    'LIBVA_DRIVERS_PATH': meson.project_build_root(),
}))
