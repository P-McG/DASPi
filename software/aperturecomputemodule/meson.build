aperture_inc = include_directories(
  'include',
)

aperture_src = files(
  'src/main.cpp',
  'src/DASPi-aperture.cpp',
  'src/event_loop.cpp',
)

aperture_deps = [
  logger_dep,
  scopedtimer_dep,
  shapefunction_dep,
  udp_dep,
  dependency('libcamera', required : true),
  dependency('openmp', required : true),
  dependency('libevent', required : true),
  dependency('libevent_pthreads', required : true),
  dependency('tbb', required : true),
  dependency('threads', required : true),
]

executable(
  'aperturecomputemodule',
  aperture_src,
  include_directories : aperture_inc,
  dependencies : aperture_deps,
)
