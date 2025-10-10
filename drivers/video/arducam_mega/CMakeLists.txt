# Existing content
zephyr_library_sources(arducam_mega.c)

# Prepend for local overrides
target_include_directories(zephyr_interface BEFORE INTERFACE ${ZEPHYR_CURRENT_MODULE_DIR}/include)
