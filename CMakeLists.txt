# CMakeLists.txt for arducam module
# Existing content (e.g., if statements for configs)
if(CONFIG_VIDEO_ARDUCAM_MEGA)  # Or whatever enable config you have

  # Prepend include path to override core Zephyr headers
  # This ensures your custom video-control.h and video.h are found first
  target_include_directories(zephyr_interface BEFORE INTERFACE ${ZEPHYR_CURRENT_MODULE_DIR}/include)

  # Existing subdir additions
  add_subdirectory(drivers/video)
 

  # If needed, add sources or other logic here

endif()