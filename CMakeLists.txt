cmake_minimum_required(VERSION 2.8.11)
project(QtSerialCom)

set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Configs" FORCE)
if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  message(STATUS "CMAKE_BUILD_TYPE was not set. Giving default value: Release")
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE Release)
endif()

set(QT_SERIAL_COM_CXX_FLAGS "-std=c++11" CACHE STRING "Foam default CXX flags" FORCE) #-pedantic
set(CMAKE_CXX_FLAGS_RELEASE "${QT_SERIAL_COM_CXX_FLAGS}")
set(CMAKE_C_FLAGS_RELEASE "-O2")
set(CMAKE_CXX_FLAGS_DEBUG "${QT_SERIAL_COM_CXX_FLAGS} -g")
set(CMAKE_C_FLAGS_DEBUG "-g")
# Handle some compile flag options
option(DEBUG_FLAG_GLIBCXX_DEBUG "use gcc flag -D_GLIBCXX_DEBUG when CMAKE_BUILD_TYPE is Debug" OFF)
if(DEBUG_FLAG_GLIBCXX_DEBUG)
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_GLIBCXX_DEBUG")
endif()
message(STATUS "C++ flags (Release):${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE}")
message(STATUS "C++ flags (Debug):${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "C flags (Release):${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "C flags (Debug):${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_DEBUG}")

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
