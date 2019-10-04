## libmio_serial_om
add_library(mio_serial_com "${MODULES_DIR}/mio/serial_com/serial_com.cpp")

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
