# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# This sample does not use the ESP-IDF Component Manager.
# Disabling it avoids failures when the global Python env is inconsistent.
set(ENV{IDF_COMPONENT_MANAGER} "0")
set(IDF_TARGET "esp32s3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hello_world)
