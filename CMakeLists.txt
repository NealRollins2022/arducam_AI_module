# CMakeLists.txt for Arducam module (C:\ncs\v2.7.0\modules\arducam\CMakeLists.txt)

if(CONFIG_VIDEO_ARDUCAM_MEGA)
  # Add include directory for drivers/video/arducam_mega.h
  target_include_directories(zephyr_interface BEFORE INTERFACE
    ${ZEPHYR_CURRENT_MODULE_DIR}/include
  )

  # Include driver source file (updated path)
  zephyr_library_sources(
    ${ZEPHYR_CURRENT_MODULE_DIR}/drivers/video/arducam_mega/arducam_mega.c
  )

  # Add drivers/video subdirectory only if it contains a CMakeLists.txt
  if(EXISTS ${ZEPHYR_CURRENT_MODULE_DIR}/drivers/video/CMakeLists.txt)
    add_subdirectory(drivers/video)
  endif()
endif()
