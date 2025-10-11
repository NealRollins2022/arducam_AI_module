# CMakeLists.txt for Arducam module (C:\ncs\v2.7.0\modules\arducam\CMakeLists.txt)

# Only include the driver if CONFIG_VIDEO_ARDUCAM_MEGA is enabled
if(CONFIG_VIDEO_ARDUCAM_MEGA)

  # Add the module's include directory (maps to C:\ncs\v2.7.0\modules\arducam\include)
  # Use BEFORE to prioritize module headers over Zephyr core headers
  target_include_directories(zephyr_interface BEFORE INTERFACE
    ${ZEPHYR_CURRENT_MODULE_DIR}/include
  )

  # Explicitly include the driver source file
  zephyr_library_sources(
    ${ZEPHYR_CURRENT_MODULE_DIR}/drivers/video/arducam_mega.c
  )

  # Add drivers/video subdirectory if it contains additional CMake logic
  add_subdirectory(drivers/video)

endif()
