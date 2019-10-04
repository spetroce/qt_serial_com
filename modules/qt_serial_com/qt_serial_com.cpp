#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <ftw.h>
#include "qt_serial_com.h"
#include "ui_qt_serial_com.h"
#include "mio/altro/str.h"


//Newline, NL or LF, 10 or 0x0a, \n
//Carriage return, CR, 13 or 0x0d, \r
//Escape, ESC, 27 or 0x1B, (\e which is a non-standard extension by GCC)

// static member definitions
std::mutex QtSerialCom::dev_search_results_vec_mtx_, QtSerialCom::dev_search_c_str_mtx_;
std::vector<std::string> QtSerialCom::dev_search_results_vec_;
char *QtSerialCom::dev_search_c_str_;


QtSerialCom::QtSerialCom(QWidget *parent) : QWidget(parent), ui(new Ui::QtSerialCom), default_baud_idx_(-1) {
  ui->setupUi(this);
  if (BuildBaudRateList()) {
    ui->comboBox_baudRate->setCurrentIndex(default_baud_idx_);
    BuildParityList();

    connect(ui->pushButton_open, SIGNAL(clicked()), this, SLOT(OpenPort()));
    connect(ui->pushButton_close, SIGNAL(clicked()), this, SLOT(ClosePort()));
    connect(ui->pushButton_write, SIGNAL(clicked()), this, SLOT(Write()));
    connect(ui->pushButton_dev_search, SIGNAL(clicked()), this, SLOT(DeviceSearch()));
  }
}


QtSerialCom::~QtSerialCom() {
  ClosePort();
  delete ui;
}


bool QtSerialCom::BuildBaudRateList() {
  std::vector<unsigned int> baud_rates = {0, 50, 75, 110, 134, 150, 200, 300, 600, 1200, 1800, 2400, 4800, 7200, 9600, 
                                          14400, 19200, 28800, 38400, 57600, 76800, 115200, 128000, 230400, 460800, 
                                          500000, 576000, 921600, 1000000, 1152000, 1500000, 2000000, 2500000, 3000000, 
                                          3500000, 4000000};
  std::vector<unsigned int> default_baud_vec = {9600, 19200, 115200};
  const size_t kDefaultBaudVecSize = default_baud_vec.size();

  default_baud_idx_ = -1;
  avail_baud_rate_vec_.clear();
  for (size_t i = 0, baud_rates_size = baud_rates.size(); i < baud_rates_size; ++i)
    if (SerialCom::GetSpeedVal(baud_rates[i]) != -1) {
      ui->comboBox_baudRate->addItem(QString().sprintf("%d", baud_rates[i]));
      avail_baud_rate_vec_.push_back(baud_rates[i]);
      if (default_baud_idx_ < 0)
        for (size_t j = 0; j < kDefaultBaudVecSize; ++j)
          if (avail_baud_rate_vec_.back() == default_baud_vec[j])
            default_baud_idx_ = avail_baud_rate_vec_.size() - 1;
    }
  EXP_CHK_M(!avail_baud_rate_vec_.empty(), return(false), "couldn't find any available baud rates");
  if (default_baud_idx_ < 0)
    default_baud_idx_ = 0;

  return true;
}


void QtSerialCom::BuildParityList() {
  ui->comboBox_parity->addItem("None");
  ui->comboBox_parity->addItem("Even");
  ui->comboBox_parity->addItem("Odd");
#ifdef CMSPAR
  ui->comboBox_parity->addItem("Space");
  ui->comboBox_parity->addItem("Mark");
#endif
}


void QtSerialCom::OpenPort() {
  const int access_mode_idx = ui->comboBox_accessMode->currentIndex();
  int flags = O_NDELAY;
  if (access_mode_idx == 0)
    flags |= O_RDWR;
  else if (access_mode_idx == 1)
    flags |= O_RDONLY;
  else if (access_mode_idx == 1)
    flags |= O_WRONLY;
  else{
    printf("%s - internal error, invalid access mode index, setting as Read Only", CURRENT_FUNC);
    flags |= O_RDONLY;
  }
  
  const char *port_name = ui->comboBox_deviceName->currentText().toUtf8().constData();
  EXP_CHK(ser_port_.Init(port_name, flags) == 0, return)
  ser_port_.SetDefaultControlFlags();
  ser_port_.SetOutputType(OutputType::RawOutput);
  ser_port_.SetInputType(InputType::RawInput);
  ser_port_.SetOutBaudRate(avail_baud_rate_vec_[ui->comboBox_baudRate->currentIndex()]);
  ser_port_.SetInBaudRate(avail_baud_rate_vec_[ui->comboBox_baudRate->currentIndex()]);
  ser_port_.SetHardwareFlowControl(ui->checkBox_hardware->isChecked());
  ser_port_.SetSoftwareFlowControl(ui->checkBox_software->isChecked());
  ser_port_.SetCharSize(ui->spinBox_charSize->value());
  std::vector<ParityType> parity_type_vec = {ParityType::NoneParity, ParityType::EvenParity, ParityType::OddParity
#ifdef CMSPAR
  , ParityType::SpaceParity, ParityType::MarkParity
#endif
  };
  ser_port_.SetParity(parity_type_vec[ui->comboBox_parity->currentIndex()]);
  ser_port_.SetStopBits(ui->spinBox_stopBits->value());
  //ser_port_.nFlushIO();

  ui->comboBox_deviceName->setEnabled(false);
  ui->comboBox_accessMode->setEnabled(false);
  ui->pushButton_open->setEnabled(false);
  ui->pushButton_close->setEnabled(true);

  if (access_mode_idx < 2) {
    qt_sock_notifier_ = new QSocketNotifier(ser_port_.GetPortFD(), QSocketNotifier::Read, this);
    connect(qt_sock_notifier_, SIGNAL(activated(int)), this, SLOT(Read(int)));
  }
}


void QtSerialCom::ClosePort() {
  ser_port_.Uninit(true);
  ui->comboBox_deviceName->setEnabled(true);
  ui->comboBox_accessMode->setEnabled(true);
  ui->pushButton_open->setEnabled(true);
  ui->pushButton_close->setEnabled(false);
}


void QtSerialCom::Write() {
  char *str = strdup(ui->lineEdit_write->text().toUtf8().data());
  const int data_type_idx = ui->comboBox_writeDataType->currentIndex();
  uint8_t *out_buffer, *out_buffer_tmp = NULL;
  size_t buffer_length = 0;

  switch(data_type_idx) {
    case 0:
      {
        char **sub_strings = mio::StrSplit(str, ' ', buffer_length), *endptr;
        if (buffer_length > 0) {
          out_buffer_tmp = new uint8_t[buffer_length];
          for (int i = 0; i < buffer_length; ++i)
            out_buffer_tmp[i] = strtol(sub_strings[i], &endptr, 16);
          out_buffer = out_buffer_tmp;
        }
        free(sub_strings);
        break;
      }
    case 1:
      out_buffer = (uint8_t *)str;
      buffer_length = strlen(str);
      break;
    default:
      printf("%s - internal error, wrong write data type index [%d]\n", data_type_idx, CURRENT_FUNC);
  }

  if (buffer_length > 0) {
    int time_out = 1;
    int num_time_out_limit = 3;
    ser_port_.Write(out_buffer, buffer_length, true, time_out, num_time_out_limit);
    ui->plainTextEdit_write->appendPlainText(ui->lineEdit_write->text().simplified());
    ui->plainTextEdit_write->ensureCursorVisible();
    ui->lineEdit_write->clear();
    //Read();
  }

  if (out_buffer_tmp)
    delete out_buffer_tmp;
  free(str);
}


void QtSerialCom::Read(int fd) {
  if (fd != ser_port_.GetPortFD())
    return;

  usleep(5000);
  int in_buffer_len;
  unsigned int bytes_read;
  const int time_out = 1, num_time_out_limit = 3;
  int err = ioctl(fd, FIONREAD, &in_buffer_len);
  ser_port_.Read(in_buf_, in_buffer_len, bytes_read, time_out, num_time_out_limit);

  int i;
  printf("%d chars read: [", in_buffer_len);
  for (i = 0; i < in_buffer_len-1; ++i)
    printf("%d ", in_buf_[i]);
  printf("%d]\n", in_buf_[i]);

  in_buf_[in_buffer_len] = 0; //add NULL terminating char
  std::string str = in_buf_;
  int cnt = 0;
  while (str.back() == 10 || str.back() == 13) { //remove trailing new line and carriage return characters
    str.pop_back();
    cnt++;
  }
  printf("Removed %d characters from end of response\n", cnt);
  std::string str_read_box = std::string("[") + str + "]";
  ui->plainTextEdit_read->appendPlainText(QString(str_read_box.c_str()));
  ui->plainTextEdit_read->ensureCursorVisible();
}


int QtSerialCom::NFTWCallback(const char *fpath, const struct stat *sb, int tflag, struct FTW *ftwbuf) {
  const size_t fpath_str_len = strlen(fpath) + 1;
  char *fpath_lower_case = (char *)malloc(fpath_str_len);
  for (size_t i = 0; i < fpath_str_len; ++i)
    fpath_lower_case[i] = tolower(fpath[i]);
  dev_search_c_str_mtx_.lock();
  if (strstr(fpath_lower_case, dev_search_c_str_) != NULL && tflag == FTW_F) {
    dev_search_c_str_mtx_.unlock();
    dev_search_results_vec_mtx_.lock();
    dev_search_results_vec_.push_back(std::string(fpath));
    dev_search_results_vec_mtx_.unlock();
  }
  else
    dev_search_c_str_mtx_.unlock();

  free(fpath_lower_case);
  return 0; // tell nftw() to continue
}


void QtSerialCom::DeviceSearch() {
  std::string dev_search_str;
  if (ui->lineEdit_dev_search->text().isEmpty())
    dev_search_str = ui->comboBox_dev_search->currentText().toStdString();
  else
    dev_search_str = ui->lineEdit_dev_search->text().toStdString();

  const size_t kDevSearchCStrLen = dev_search_str.size() + 1;
  dev_search_c_str_mtx_.lock();
  dev_search_c_str_ = (char *)malloc(kDevSearchCStrLen);
  strcpy(dev_search_c_str_, dev_search_str.c_str());
  for (size_t i = 0; i < kDevSearchCStrLen; ++i)
    dev_search_c_str_[i] = tolower(dev_search_c_str_[i]);
  dev_search_c_str_mtx_.unlock();

  dev_search_results_vec_mtx_.lock();
  for (size_t i = 0; i < dev_search_results_vec_.size(); ++i)
    ui->comboBox_deviceName->removeItem(ui->comboBox_deviceName->count()-1);
  dev_search_results_vec_.clear();
  dev_search_results_vec_mtx_.unlock();
  int flags = 0;
  EXP_CHK_ERRNO(nftw("/dev", QtSerialCom::NFTWCallback, 20, flags) == 0, return)

  dev_search_c_str_mtx_.lock();
  free(dev_search_c_str_);
  dev_search_c_str_mtx_.unlock();

  const size_t kNumDefaultDev = ui->comboBox_deviceName->count();
  dev_search_results_vec_mtx_.lock();
  for (std::string &res : dev_search_results_vec_)
    ui->comboBox_deviceName->addItem(QString::fromStdString(res));
  dev_search_results_vec_mtx_.unlock();
  ui->comboBox_deviceName->setCurrentIndex(kNumDefaultDev != ui->comboBox_deviceName->count() ? kNumDefaultDev : 3);
}

