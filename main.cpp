#include "qtSerialCom.h"
#include <QApplication>

int main(int argc, char *argv[]){
  QApplication a(argc, argv);
  QtSerialCom w;
  w.show();

  return a.exec();
}
