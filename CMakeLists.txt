# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32s3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(hello_world)

# Work around a GCC internal compiler error seen on ESP-IDF v6.0 when building
# esp-dsp's ANSI float convolution source for this project setup. Restrict the
# workaround to the single managed source file instead of lowering optimization
# across the whole firmware.
set(ESP_DSP_ICE_SOURCE
    "${CMAKE_SOURCE_DIR}/managed_components/espressif__esp-dsp/modules/conv/float/dspi_conv_f32_ansi.c")
if(EXISTS "${ESP_DSP_ICE_SOURCE}")
    set_source_files_properties("${ESP_DSP_ICE_SOURCE}" PROPERTIES COMPILE_OPTIONS "-O1")
endif()
