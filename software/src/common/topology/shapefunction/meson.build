shapefunction_inc = include_directories('include')

cpp = meson.get_compiler('cpp')

opencv_dep = dependency('opencv4', native : false)
armadillo_dep = dependency('armadillo', native : false, required : true)

blas_dep = cpp.find_library('blas', required : true)
lapack_dep = cpp.find_library('openblas', required : true)
#lapack_dep = cpp.find_library('lapack', required : true)
arpack_dep = cpp.find_library('arpack', required : true)

shapefunction_lib = static_library(
  'shapefunction',
  [],
  include_directories : shapefunction_inc,
  dependencies : [
    opencv_dep,
    armadillo_dep,
    blas_dep,
    lapack_dep,
    arpack_dep,],
)

shapefunction_dep = declare_dependency(
  include_directories : shapefunction_inc,
  link_with : shapefunction_lib,
  dependencies : [
    opencv_dep,
    armadillo_dep,
    blas_dep,
    lapack_dep,
    arpack_dep,],
)
