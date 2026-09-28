zephyr_library()
zephyr_library_sources(max77961.c)
zephyr_include_directories(.)
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)