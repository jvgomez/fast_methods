macro(build_example _name)
  add_executable (${_name} ${_name}.cpp)
  target_link_libraries (${_name} fastmarching)
endmacro()

build_example(test_fmm)
build_example(test_fm2)
build_example(test_fmm3d)
build_example(test_fmm_benchmark)
