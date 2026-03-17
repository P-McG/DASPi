compute_inc = include_directories(
  'include',
)

compute_src = files(
  'src/DASPi-aperture-client.cpp',
  'src/main.cpp',
)

compute_deps = [
  logger_dep,
  shapefunction_dep,
  udp_dep,
  dependency('threads', required : true),
  dependency('tbb', required : true),
  dependency('openmp', required : true),
]

executable(
  'computemodule',
  compute_src,
  include_directories : compute_inc,
  dependencies : compute_deps,
  cpp_args : ['-Ddebug'],
)
