cpp = meson.get_compiler('cpp')

opencv_dep = dependency('opencv4', required : true, include_type : 'system')
eigen_dep  = dependency('eigen3', required : true)
openmp_dep = dependency('openmp', required : true)

blas_dep   = cpp.find_library('blas', required : true)
lapack_dep = cpp.find_library('lapack', required : true)

camera_model_inc = include_directories('include')

camera_model_src = files(
  'src/DASPi-bayer-io.cpp',
  'src/DASPi-fisheye-camera-model.cpp',
  'src/DASPi-sphere-stitcher.cpp',
)

camera_model_headers = files(
  'include/DASPi-bayer-io.h',
  'include/DASPi-fisheye-camera-model.h',
  'include/DASPi-sphere-stitcher.h',
)

camera_model_deps = [
  opencv_dep,
  eigen_dep,
  openmp_dep,
  blas_dep,
  lapack_dep,
]

camera_model_lib = library(
  'camera_model',
  camera_model_src,
  include_directories : camera_model_inc,
  dependencies : camera_model_deps,
  install : true,
)

install_headers(
  camera_model_headers,
  subdir : 'camera_model',
)

camera_model_dep = declare_dependency(
  include_directories : camera_model_inc,
  link_with : camera_model_lib,
  dependencies : camera_model_deps,
)
