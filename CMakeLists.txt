# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
)

set(
  COMPONENTS
  "main esptool_py motorgo-mini esp-now filters timer"
  CACHE STRING
  "List of components to include"
  )

project(esp-motorgo-mini-gesture-control)

set(CMAKE_CXX_STANDARD 23)
