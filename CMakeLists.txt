cmake_minimum_required(VERSION 2.8.11)
project(QtSerialCom)

## User defined library/include paths
include(PkgConfigPath.cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${MIO_INCLUDE_DIR}/mio/CMakeModules")

## mio
include_directories(${MIO_INCLUDE_DIR})

## Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

## libmio_serialCom
add_library(mio_serialCom ${MIO_INCLUDE_DIR}/mio/serialCom/serialCom.cpp)

qt5_wrap_ui(ui_qtSerialCom qtSerialCom.ui)
add_executable(qtSerialCom main.cpp qtSerialCom.cpp ${ui_qtSerialCom})
target_link_libraries(qtSerialCom Qt5::Widgets mio_serialCom)
