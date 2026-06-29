# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# ---------------------------------------------------------------------------
# FT4 support — compile-time opt-in (default: ON).
#
# When ON, FT4 protocol code is included and the user can select FT8 or FT4
# at runtime via the Mode menu (Settings → Mode: FT8 | FT4). The choice is
# saved to Station.txt and takes effect on next boot.
#
# Set OFF to build a smaller FT8-only binary with no mode-switch menu item:
#   idf.py build -DENABLE_FT4=OFF
# ---------------------------------------------------------------------------
option(ENABLE_FT4 "Include FT4 protocol support and runtime mode switch" ON)

if(ENABLE_FT4)
    add_compile_definitions(ENABLE_FT4=1)
    message(STATUS "FT4 support: ON (runtime mode switch via menu)")
else()
    add_compile_definitions(ENABLE_FT4=0)
    message(STATUS "FT4 support: OFF (FT8-only build)")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(CP705)
# This release requires the full merged image because it installs the FATFS-only
# partition table. First boot formats the new 3 MB FATFS when needed.
add_custom_command(TARGET app POST_BUILD
    COMMAND ${PYTHON} -m esptool --chip esp32s3 merge_bin -o "CP705_Merged_Auto.bin" @flash_args
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Auto-generating merged binary CP705_Merged_Auto.bin for release..."
)
