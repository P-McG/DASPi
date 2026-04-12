camera_model_inc = include_directories('include')

eigen_inc = include_directories('/usr/include/eigen3')

camera_model_deps = [
  dependency('opencv4', required : true),
  dependency('openmp', required : true),
]

camera_model_lib = static_library(
  'cameramodel',
  [
    #'src/DASPi-camera.cpp',
    'src/DASPi-sphere-stitcher.cpp',
    'src/DASPi-spherical-math.cpp',

  ],
  include_directories : [camera_model_inc, eigen_inc],
  dependencies : camera_model_deps,
)

camera_model_dep = declare_dependency(
  include_directories : [camera_model_inc, eigen_inc],
  link_with : camera_model_lib,
  dependencies : camera_model_deps,
)
