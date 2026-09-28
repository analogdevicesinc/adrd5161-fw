
cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C)

zephyr_include_directories(src)

zephyr_include_directories(objdict)
target_link_libraries(app PRIVATE modules__canopennode)
target_sources(app PRIVATE
	src/bms_fw.c
	src/drivers/adp5589.c
	src/button_manager/button_manager.c
	objdict/OD.c
)
