aperture_inc = include_directories(
  'include',
  'src/udp/include',
  'src/logger/include',
  'src/scopedtimer/include',
)

aperture_src = files(
  'src/main.cpp',
  'src/DASPi-aperture.cpp',
  'src/event_loop.cpp',
  'src/udp/src/DASPi-udp-srv.cpp',
)

aperture_deps = [
  shapefunction_dep,
  dependency('libcamera', required : true),
  dependency('openmp', required : true),
  dependency('libevent_pthreads', required : true),
  dependency('tbb', required : true),
]

add_project_arguments('-Wno-unused-parameter', language : 'cpp')

executable(
  'aperturecomputemodule',
  aperture_src,
  include_directories : aperture_inc,
  dependencies : aperture_deps,
)
