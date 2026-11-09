cmake_minimum_required(VERSION 3.16)
project(palpeur_traxx_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(palpeur_core
  src/core/calibration.cpp
  src/core/can_bus.cpp
  src/core/can_protocol.cpp
  src/core/geometry.cpp
  src/sim/geometry_simulation.cpp
)
target_include_directories(palpeur_core PUBLIC src)

add_executable(palpeur_native_tests test/native/test_main.cpp)
target_link_libraries(palpeur_native_tests PRIVATE palpeur_core)

add_executable(palpeur_geometry_study tools/geometry_study.cpp)
target_link_libraries(palpeur_geometry_study PRIVATE palpeur_core)

enable_testing()
add_test(NAME palpeur_native_tests COMMAND palpeur_native_tests)