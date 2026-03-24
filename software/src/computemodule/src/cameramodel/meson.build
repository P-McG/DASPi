opencv_dep = dependency('opencv4', required : true, include_type : 'system')
eigen_dep = dependency('eigen3', required : true)
openmp_dep = dependency('openmp', required : true)

camera_model_inc = include_directories('include')

camera_model_lib = static_library(
  'camera-model',
  [
    'src/DASPi-bayer-io.cpp',
    'src/DASPi-fisheye-camera-model.cpp',
    'src/DASPi-sphere-stitcher.cpp',
  ],
  include_directories : [camera_model_inc],
  dependencies : [opencv_dep, eigen_dep, openmp_dep],
)

camera_model_dep = declare_dependency(
  include_directories : [camera_model_inc],
  link_with : camera_model_lib,
  dependencies : [opencv_dep, eigen_dep, openmp_dep],
)
