cmake_minimum_required(VERSION 3.16)
project(indi-esphome-observatory CXX C)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")

include(GNUInstallDirs)

set(INDI_ESPHOME_OBSERVATORY_VERSION_MAJOR 1)
set(INDI_ESPHOME_OBSERVATORY_VERSION_MINOR 0)

find_package(INDI REQUIRED)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/indi_esphome_observatory.xml.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/indi_esphome_observatory.xml
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${INDI_INCLUDE_DIR})

set(indi_esphome_observatory_SRCS
    ${CMAKE_CURRENT_SOURCE_DIR}/esphome_observatory.cpp
)

add_executable(indi_esphome_observatory ${indi_esphome_observatory_SRCS})
target_compile_features(indi_esphome_observatory PRIVATE cxx_std_17)
target_link_libraries(indi_esphome_observatory ${INDI_LIBRARIES})

install(TARGETS indi_esphome_observatory RUNTIME DESTINATION bin)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_esphome_observatory.xml DESTINATION ${INDI_DATA_DIR})
