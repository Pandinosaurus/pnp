CMAKE_MINIMUM_REQUIRED(VERSION 3.10)
PROJECT(lambdatwist-pnp CXX)
message("================================================================================")

find_package(Ceres REQUIRED )
add_library(libpnp SHARED ${util-srcs} ${lambdatwist-srcs} pnp_ransac.cpp p4p.cpp utils/mlibtime.cpp)
target_compile_features(libpnp PUBLIC cxx_std_11)
target_link_libraries(libpnp PUBLIC ${CERES_LIBRARIES})
target_include_directories(libpnp PUBLIC ".")
target_include_directories(libpnp PRIVATE "lambdatwist-p3p/utils")

add_executable(main test_pnp.cpp)
target_link_libraries(main libpnp)

option(WITH_PYTHON_BINDINGS "compile python bindings" OFF)
if(WITH_PYTHON_BINDINGS)
# Build pybind11 bindinis
pybind11_add_module(pnp_python_binding pnp_python_binding.cpp)
target_link_libraries(pnp_python_binding PUBLIC libpnp )
# OBS! Pybind11 require that you specify python exectuable to build for, e.g. cmake .. -DPYTHON_EXECUTABLE=python3
endif()

message("================================================================================")
