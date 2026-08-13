cmake_minimum_required(VERSION 3.16)
project(flightSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# --------------------------------------------------------------------------
# 物理コア (外部依存なし)
# --------------------------------------------------------------------------
add_library(fs_core STATIC
  src/atmosphere/atmosphere.cpp
  src/aerodynamics/aerodynamics.cpp
  src/aircraft/aircraft.cpp
  src/aircraft/planform.cpp
  src/dynamics/dynamics.cpp
  src/stability/stability.cpp
  src/game/game.cpp
)
target_include_directories(fs_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
if(MSVC)
  target_compile_options(fs_core PRIVATE /W4)
else()
  target_compile_options(fs_core PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# --------------------------------------------------------------------------
# 検証テスト (外部依存なし)
# --------------------------------------------------------------------------
enable_testing()
add_executable(fs_tests tests/test_flight.cpp)
target_link_libraries(fs_tests PRIVATE fs_core)
add_test(NAME flight_model COMMAND fs_tests)

# --------------------------------------------------------------------------
# GUI (raylib)。 -DFS_BUILD_GUI=OFF でスキップできる
# --------------------------------------------------------------------------
option(FS_BUILD_GUI "Build the raylib GUI" ON)
if(FS_BUILD_GUI)
  find_package(raylib 4.5 QUIET)
  if(NOT raylib_FOUND)
    include(FetchContent)
    FetchContent_Declare(raylib
      GIT_REPOSITORY https://github.com/raysan5/raylib.git
      GIT_TAG        5.5
      GIT_SHALLOW    TRUE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(raylib)
  endif()

  add_executable(flightsim src/gui/main.cpp)
  target_link_libraries(flightsim PRIVATE fs_core raylib)
  if(APPLE)
    target_link_libraries(flightsim PRIVATE
      "-framework IOKit" "-framework Cocoa" "-framework OpenGL")
  endif()
endif()
