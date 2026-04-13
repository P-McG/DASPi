project('DASPi', 'c', 'cpp',
  default_options : [
    'werror=true',
    'warning_level=2',
    'cpp_std=c++20',
    'buildtype=release',
  ]
)

add_project_arguments(
  '-Wno-unused-parameter',
  '-Wno-error=deprecated-enum-enum-conversion',
  '-Wno-deprecated-enum-enum-conversion',  # 👈 add this
  language : 'cpp'
)

subdir('common')
subdir('common/logger')
subdir('common/scopedtimer')
subdir('common/shapefunction')
subdir('common/udp')


if get_option('build_aperturecomputemodule')
  subdir('aperturecomputemodule')
endif

if get_option('build_computemodule')
  subdir('common/cameramodel')
  subdir('computemodule')
endif
