cmake_minimum_required(VERSION 3.5.1)
project(QT_SERIAL_COM)

set(CMAKE_INSTALL_PREFIX "/usr/local")

## modules
set(MODULES_DIR "${CMAKE_CURRENT_LIST_DIR}/modules")

## Setup Release and Debug variables
include("${MODULES_DIR}/mio/cmake/DefaultConfigTypes.cmake")

add_subdirectory(modules)
