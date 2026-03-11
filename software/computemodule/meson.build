project('trial', 'c', 'cpp',
  default_options : [
    'werror=true',
    'warning_level=2',
    'cpp_std=c++20',
    'buildtype=release',
  ]
)

inc_dirs = include_directories(
  'include',
  'src/shapefunction/include',
  'src/udp/include',
  'src/logger/include',
)

src_files = files(
  'src/udp/src/DASPi-udp-clnt.cpp',
  'src/DASPi-aperture-client.cpp',
  'src/main.cpp',
)

deps = [
  dependency('threads', required : true),
  dependency('tbb', required : true),
  dependency('openmp', required : true),
]

add_project_arguments(
  '-Wno-unused-parameter',
  '-Ddebug',
  language : 'cpp'
)

executable(
  'computemodule',
  src_files,
  include_directories : inc_dirs,
  dependencies : deps,
)
