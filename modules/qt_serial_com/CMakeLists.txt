## libmio_serial_om
add_library(mio_serial_com "${MODULES_DIR}/mio/serial_com/serial_com.cpp")

## Qt5
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
# Allow cmake to use env var Qt5_ROOT
# You can set it in ~/.bashrc
# eg. export Qt5_ROOT=/opt/Qt/5.12.8/gcc_64
cmake_policy(SET CMP0074 NEW)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

# Place the qt_dragon binary into the root project binary directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${QT_SERIAL_COM_BINARY_DIR})

qt5_wrap_ui(qt_serial_com_ui qt_serial_com.ui)
add_executable(qt_serial_com main.cpp qt_serial_com.cpp ${qt_serial_com_ui})
target_link_libraries(qt_serial_com Qt5::Widgets mio_serial_com)

install(
  FILES qt_serial_com.desktop
  DESTINATION "/home/$ENV{USER}/.local/share/applications")

install(
  FILES qt_serial_com.png
  DESTINATION "/home/$ENV{USER}/.local/share/applications/icons")

install(TARGETS qt_serial_com DESTINATION bin)
