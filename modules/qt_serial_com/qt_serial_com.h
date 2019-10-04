#ifndef QTSERIALCOM_H
#define QTSERIALCOM_H

#include <QWidget>
#include <QSocketNotifier>
#include <vector>
#include <mutex>
#include "mio/serial_com/serial_com.h"


namespace Ui {
  class QtSerialCom;
}

class QtSerialCom : public QWidget {
  Q_OBJECT

  public:
    explicit QtSerialCom(QWidget *parent = 0);
    ~QtSerialCom();

  private slots:
    void OpenPort();
    void ClosePort();
    void Write();
    void Read(int);
    void DeviceSearch();

  private:
    Ui::QtSerialCom *ui;
    SerialCom ser_port_;
    std::vector<unsigned int> avail_baud_rate_vec_;
    int default_baud_idx_;
    QSocketNotifier *qt_sock_notifier_;
    char in_buf_[5012];
    static std::mutex dev_search_results_vec_mtx_, dev_search_c_str_mtx_;
    static std::vector<std::string> dev_search_results_vec_;
    static char *dev_search_c_str_;

    bool BuildBaudRateList();
    void BuildParityList();
    static int NFTWCallback(const char *fpath, const struct stat *sb, int tflag, struct FTW *ftwbuf);
};

#endif // QTSERIALCOM_H
