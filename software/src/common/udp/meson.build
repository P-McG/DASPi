common_inc = include_directories('../include')
udp_inc = include_directories('include')
logger_inc = include_directories('../logger/include')
shapefunction_inc = include_directories('../shapefunction/include')

udp_lib = static_library(
  'udp',
  [
    'src/DASPi-wire-utils.cpp',
    'src/DASPi-udp-srv.cpp',
    'src/DASPi-udp-clnt.cpp',
  ],
  include_directories : [common_inc, logger_inc, udp_inc, shapefunction_inc],
  dependencies : [logger_dep],
  cpp_args : ['-D_GNU_SOURCE'],
)

udp_dep = declare_dependency(
  include_directories : [common_inc, logger_inc, udp_inc],
  link_with : udp_lib,
  dependencies : [logger_dep],
)
