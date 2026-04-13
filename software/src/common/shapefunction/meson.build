shapefunction_inc = include_directories('include')

opencv_dep = dependency('opencv4', required : true)

shapefunction_lib = static_library(
  'shapefunction',
  [
#    'src/DASPi-overlapshapefunction.cpp',
#    'src/DASPi-globallinearshapefunction.cpp',
  ],
  include_directories : shapefunction_inc,
  dependencies : [opencv_dep],
)

shapefunction_dep = declare_dependency(
  include_directories : shapefunction_inc,
  link_with : shapefunction_lib,
  dependencies : [opencv_dep],
)

