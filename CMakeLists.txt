cmake_minimum_required(VERSION 3.5.1)
project(QtSerialCom)

## modules
set(MODULES_DIR "${CMAKE_CURRENT_LIST_DIR}/modules")

## Setup Release and Debug variables
include("${MODULES_DIR}/mio/cmake/DefaultConfigTypes.cmake")

## Qt5
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_subdirectory(modules)
