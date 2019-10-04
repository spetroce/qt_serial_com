#include <QApplication>
#include "qt_serial_com.h"


int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  QtSerialCom w;
  w.show();

  return a.exec();
}
