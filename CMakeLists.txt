cmake_minimum_required(VERSION 2.8.11)
project(QtSerialCom)

## User defined library/include paths
include(PkgConfigPath.cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${MIO_INCLUDE_DIR}/mio/cmake/Modules")

## Setup Release and Debug variables
include(${MIO_INCLUDE_DIR}/mio/cmake/DefaultConfigTypes.cmake)

## mio
include_directories(${MIO_INCLUDE_DIR})

## Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

## libmio_serialCom
add_library(mio_serial_com ${MIO_INCLUDE_DIR}/mio/serial_com/serial_com.cpp)

qt5_wrap_ui(qt_serial_com_ui qt_serial_com.ui)
add_executable(qt_serial_com main.cpp qt_serial_com.cpp ${qt_serial_com_ui})
target_link_libraries(qt_serial_com Qt5::Widgets mio_serial_com)
