cmake_minimum_required(VERSION 3.20.0)
# This CMakeLists enables the module to contribute sources/Kconfig.
# It is executed when this repo is present in WEST manifest or ZEPHYR_MODULES.
if(NOT DEFINED ZEPHYR_BASE)
  find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
endif()
project(arducam_mega_module)
add_subdirectory(drivers/video)
