udp_inc = include_directories('include')

udp_lib = static_library(
  'udp',
  [
    'src/DASPi-udp-srv.cpp',
    'src/DASPi-udp-clnt.cpp',
  ],
  include_directories : udp_inc,
  dependencies : [logger_dep],
)

udp_dep = declare_dependency(
  include_directories : udp_inc,
  link_with : udp_lib,
  dependencies : [logger_dep],
)
