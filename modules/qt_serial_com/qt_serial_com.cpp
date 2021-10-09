#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <ftw.h>
#include "qt_serial_com.h"
#include "ui_qt_serial_com.h"
#include "mio/altro/error.h"
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
    ui->combo_box_baud_rate->setCurrentIndex(default_baud_idx_);
    BuildParityList();

    connect(ui->push_button_open, SIGNAL(clicked()), this, SLOT(OpenPort()));
    connect(ui->push_button_close, SIGNAL(clicked()), this, SLOT(ClosePort()));
    connect(ui->push_button_write, SIGNAL(clicked()), this, SLOT(Write()));
    connect(ui->push_button_dev_search, SIGNAL(clicked()), this, SLOT(DeviceSearch()));
    connect(ui->push_button_clear_input, SIGNAL(clicked()), ui->plain_text_edit_input, SLOT(clear()));
    connect(ui->push_button_clear_output, SIGNAL(clicked()), ui->plain_text_edit_output, SLOT(clear()));
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
  std::vector<unsigned int> default_baud_vec = {115200, 19200, 9600};
  const size_t kDefaultBaudVecSize = default_baud_vec.size();

  default_baud_idx_ = -1;
  avail_baud_rate_vec_.clear();
  for (size_t i = 0, baud_rates_size = baud_rates.size(); i < baud_rates_size; ++i) {
    if (SerialCom::GetSpeedVal(baud_rates[i]) != -1) {
      ui->combo_box_baud_rate->addItem(QString().sprintf("%d", baud_rates[i]));
      avail_baud_rate_vec_.push_back(baud_rates[i]);
    }
  }
  for (auto & db : default_baud_vec) {
    auto it = std::find(avail_baud_rate_vec_.begin(), avail_baud_rate_vec_.end(), db);
    if (it != avail_baud_rate_vec_.end()) {
      default_baud_idx_ = it - avail_baud_rate_vec_.begin();
      break;
    }
  }
  EXP_CHK_M(!avail_baud_rate_vec_.empty(), return(false), "couldn't find any available baud rates");
  if (default_baud_idx_ < 0)
    default_baud_idx_ = 0;

  return true;
}


void QtSerialCom::BuildParityList() {
  ui->combo_box_parity->addItem("None");
  ui->combo_box_parity->addItem("Even");
  ui->combo_box_parity->addItem("Odd");
#ifdef CMSPAR
  ui->combo_box_parity->addItem("Space");
  ui->combo_box_parity->addItem("Mark");
#endif
}


void QtSerialCom::OpenPort() {
  const int access_mode_idx = ui->combo_box_access_mode->currentIndex();
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
  
  const char *port_name = ui->combo_box_device_name->currentText().toUtf8().constData();
  EXP_CHK(ser_port_.Init(port_name, flags) == 0, return)
  ser_port_.SetDefaultControlFlags();
  ser_port_.SetIgnoreBreakCondition(false);
  ser_port_.SetOutputType(OutputType::RawOutput);
  ser_port_.SetInputType(InputType::RawInput);
  ser_port_.SetOutBaudRate(avail_baud_rate_vec_[ui->combo_box_baud_rate->currentIndex()]);
  ser_port_.SetInBaudRate(avail_baud_rate_vec_[ui->combo_box_baud_rate->currentIndex()]);
  ser_port_.SetHardwareFlowControl(ui->check_box_hardware->isChecked());
  ser_port_.SetSoftwareFlowControl(ui->check_box_software->isChecked());
  ser_port_.SetCharSize(ui->spin_box_char_size->value());

  std::vector<ParityType> parity_type_vec = {ParityType::NoneParity, ParityType::EvenParity, ParityType::OddParity
#ifdef CMSPAR
  , ParityType::SpaceParity, ParityType::MarkParity
#endif
  };
  ser_port_.SetParity(parity_type_vec[ui->combo_box_parity->currentIndex()]);
  // ser_port_.SetParityChecking(false, false, false, false)
  ser_port_.SetStopBits(ui->spin_box_stop_bits->value());
  // ser_port_.FlushIO();

  ui->combo_box_device_name->setEnabled(false);
  ui->combo_box_access_mode->setEnabled(false);
  ui->push_button_open->setEnabled(false);
  ui->push_button_close->setEnabled(true);

  if (access_mode_idx < 2) {
    qt_sock_notifier_ = new QSocketNotifier(ser_port_.GetPortFD(), QSocketNotifier::Read, this);
    connect(qt_sock_notifier_, SIGNAL(activated(int)), this, SLOT(Read(int)));
    std::cout << "Attached QSocketNotifier to " << ser_port_.GetPortFD() << std::endl;
  }
}


void QtSerialCom::ClosePort() {
  ser_port_.Uninit(true);
  ui->combo_box_device_name->setEnabled(true);
  ui->combo_box_access_mode->setEnabled(true);
  ui->push_button_open->setEnabled(true);
  ui->push_button_close->setEnabled(false);
}


void QtSerialCom::Write() {
  char *str = strdup(ui->line_edit_write->text().toUtf8().data());
  const size_t str_len = strlen(str);
  if (str_len < 1) {
    std::cout << FFL_STRM << "Write Line-Edit is empty..." << std::endl;
    return;
  }
  const int data_type_idx = ui->combo_box_write_data_type->currentIndex();
  std::vector<uint8_t> out_buffer;
  size_t out_buf_len = 0;

  switch (data_type_idx) {
    // HEX with space separations
    case 0:
      {
        char **sub_strings = mio::StrSplit(str, ' ', out_buf_len), *endptr;
        if (out_buf_len > 0) {
          out_buffer.resize(out_buf_len);
          for (int i = 0; i < out_buf_len; ++i)
            out_buffer[i] = strtol(sub_strings[i], &endptr, 16);
        }
        free(sub_strings);
        break;
      }
    // ASCII
    case 1:
      out_buf_len = str_len;
      out_buffer.assign(str, str + out_buf_len);
      break;
    // ASCII + CR (add a CR to data before sending it out)
    case 2:
      out_buf_len = str_len + 1;
      out_buffer.reserve(out_buf_len);
      out_buffer.assign(str, str + str_len);
      out_buffer.push_back(13);
      break;
    default:
      printf("%s - internal error, wrong write data type index [%d]\n", data_type_idx, CURRENT_FUNC);
  }

  if (out_buf_len > 0) {
    int time_out = 1;
    int num_time_out_limit = 3;
    ser_port_.Write(out_buffer.data(), out_buf_len, true, time_out, num_time_out_limit);
    ui->plain_text_edit_output->appendPlainText(ui->line_edit_write->text().simplified());
    ui->plain_text_edit_output->ensureCursorVisible();
    ui->line_edit_write->clear();
  }

  free(str);
}


void QtSerialCom::Read(int fd) {
  if (fd != ser_port_.GetPortFD())
    return;

  usleep(5000);
  int in_buffer_len;
  unsigned int bytes_read;
  // With time_out and num_time_out_limit both set to zero, we perform a non-blocking read
  const int time_out = 0,
            num_time_out_limit = 0;
  int err = ioctl(fd, FIONREAD, &in_buffer_len);
  ser_port_.Read(in_buf_, in_buffer_len, bytes_read, time_out, num_time_out_limit);

  int i;
  printf("%d bytes read: [", in_buffer_len);
  for (i = 0; i < in_buffer_len-1; ++i)
    printf("%d ", in_buf_[i]);
  printf("%d]\n", in_buf_[i]);

  in_buf_[in_buffer_len] = 0;  // add NULL terminating char
  std::string str = reinterpret_cast<char*>(in_buf_);
  int cnt = 0;
  while (str.back() == 10 || str.back() == 13) {  // remove trailing new line and carriage return characters
    str.pop_back();
    ++cnt;
  }
  if (cnt > 0)
    printf("Removed %d characters from end of response\n", cnt);
  std::string str_read_box = std::string("[") + str + "]";
  ui->plain_text_edit_input->appendPlainText(QString(str_read_box.c_str()));
  ui->plain_text_edit_input->ensureCursorVisible();
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
  if (ui->line_edit_dev_search->text().isEmpty())
    dev_search_str = ui->combo_box_dev_search->currentText().toStdString();
  else
    dev_search_str = ui->line_edit_dev_search->text().toStdString();

  const size_t dev_search_c_str_len = dev_search_str.size() + 1;
  dev_search_c_str_mtx_.lock();
  dev_search_c_str_ = (char *)malloc(dev_search_c_str_len);
  strcpy(dev_search_c_str_, dev_search_str.c_str());
  for (size_t i = 0; i < dev_search_c_str_len; ++i)
    dev_search_c_str_[i] = tolower(dev_search_c_str_[i]);
  dev_search_c_str_mtx_.unlock();

  dev_search_results_vec_mtx_.lock();
  for (size_t i = 0; i < dev_search_results_vec_.size(); ++i)
    ui->combo_box_device_name->removeItem(ui->combo_box_device_name->count()-1);
  dev_search_results_vec_.clear();
  dev_search_results_vec_mtx_.unlock();
  int flags = 0;
  EXP_CHK_ERRNO(nftw("/dev", QtSerialCom::NFTWCallback, 20, flags) == 0, return)

  dev_search_c_str_mtx_.lock();
  free(dev_search_c_str_);
  dev_search_c_str_mtx_.unlock();

  const size_t kNumDefaultDev = ui->combo_box_device_name->count();
  dev_search_results_vec_mtx_.lock();
  for (std::string &res : dev_search_results_vec_)
    ui->combo_box_device_name->addItem(QString::fromStdString(res));
  dev_search_results_vec_mtx_.unlock();
  ui->combo_box_device_name->setCurrentIndex(kNumDefaultDev != ui->combo_box_device_name->count() ? kNumDefaultDev : 3);
}

