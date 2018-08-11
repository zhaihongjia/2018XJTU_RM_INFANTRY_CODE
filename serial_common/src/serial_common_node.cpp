#include "serial_common_node.h"
using namespace std;

SerialComNode::SerialComNode(void)
  : fd_(0), is_open_(false), stop_receive_(true), stop_send_(true) {

    length_beam_ = 90;
    length_column_ = 50;
    baudrate_=921600;
    port_ = "/dev/ttyUSB0";//delete[]array4;

    if(!SerialInitialization(port_, baudrate_, 0, 8, 1, 'N'))
    {
      std::cout<<"serialport init error"<<std::endl;
    }
    is_open_ = true;
    stop_receive_ = false;
    stop_send_ = false;
    is_sim_ = false;
    //----------for debug---------
    is_debug_= true;
    pack_length_ = 0;
    total_length_ = 0;
    free_length_ = UART_BUFF_SIZE;

    mode_pub_ = nh_.advertise<serial_common::Infantrymode>("read", 30);

    if (is_debug_) {
    fp_ = fopen("debug_com.txt", "w+");
    }
}

bool SerialComNode::SerialInitialization(std::string port,
                                         int baudrate,
                                         int flow_control,
                                         int data_bits,
                                         int stop_bits,
                                         int parity) {
  fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  //  CHECK(fd_ != -1) << "Serial port open failed!";
  tcgetattr(fd_, &termios_options_);
  termios_options_original_ = termios_options_;
  ConfigBaudrate(baudrate);
  termios_options_.c_cflag |= CLOCAL;
  termios_options_.c_cflag |= CREAD;
  termios_options_.c_cflag &= ~CSIZE;
  switch (flow_control) {
    case 0 :termios_options_.c_cflag &= ~CRTSCTS;
      break;
    case 1 :termios_options_.c_cflag |= CRTSCTS;
      break;
    case 2 :termios_options_.c_cflag |= IXON | IXOFF | IXANY;
      break;
    default: termios_options_.c_cflag &= ~CRTSCTS;
      break;
  }
  switch (data_bits) {
    case 5 :termios_options_.c_cflag |= CS5;
      break;
    case 6 :termios_options_.c_cflag |= CS6;
      break;
    case 7 :termios_options_.c_cflag |= CS7;
      break;
    case 8 :termios_options_.c_cflag |= CS8;
      break;
    default:
      return false;
  }
  switch (parity) {
    case 'n':
    case 'N':termios_options_.c_cflag &= ~(PARENB | PARODD);
      termios_options_.c_iflag &= ~INPCK;
      break;
    case 'o':
    case 'O':termios_options_.c_cflag |= (PARODD | PARENB);
      termios_options_.c_iflag |= INPCK;
      break;
    case 'e':
    case 'E':termios_options_.c_cflag |= PARENB;
      termios_options_.c_cflag &= ~PARODD;
      termios_options_.c_iflag |= INPCK;
      break;
    case 's':
    case 'S':termios_options_.c_cflag &= ~PARENB;
      termios_options_.c_cflag &= ~CSTOPB;
      break;
    default:
      return false;
  }
  switch (stop_bits) {
    case 1: termios_options_.c_cflag &= ~CSTOPB;
      break;
    case 2: termios_options_.c_cflag |= CSTOPB;
      break;
    default:
      return false;
  }
  termios_options_.c_lflag = 0;
  termios_options_.c_oflag = 0;
  termios_options_.c_oflag &= ~OPOST;
  termios_options_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  termios_options_.c_cc[VTIME] = 1;
  termios_options_.c_cc[VMIN] = 60;
  termios_options_.c_iflag = IGNBRK;
  tcsetattr(fd_, TCSANOW, &termios_options_);
  return true;
}

bool SerialComNode::ConfigBaudrate(int baudrate) {
  int i;
  int speed_arr[] = {B921600, B576000, B460800, B230400, B115200, B19200, B9600, B4800, B2400, B1200, B300};
  int name_arr[] = {921600, 576000, 460800, 230400, 115200, 19200, 9600, 4800, 2400, 1200, 300};
  for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
    if (baudrate == name_arr[i]) {
      cfsetispeed(&termios_options_, speed_arr[i]);
      cfsetospeed(&termios_options_, speed_arr[i]);
      return true;
    }
  }
  return false;
}

void SerialComNode::Run() {
  receive_loop_thread_ = new std::thread(boost::bind(&SerialComNode::ReceiveLoop, this));

  enemy_sub = nh_.subscribe("enemy", 1, &SerialComNode::GimbalEnemyControlCallback, this);
  buff_sub = nh_.subscribe("buff", 1, &SerialComNode::GimbalBuffControlCallback, this);
  wifi_sub = nh_.subscribe("wifi", 1, &SerialComNode::WifiBuffCallback, this);
  send_loop_thread_ = new std::thread(boost::bind(&SerialComNode::SendPack, this));
  ros::spin();
}



void SerialComNode::ReceiveLoop() {
  while (is_open_ && !stop_receive_ && ros::ok()) {
    read_buff_index_ = 0;
    read_len_ = ReceiveData(fd_, UART_BUFF_SIZE);
    if (read_len_ > 0) {
      while (read_len_--) {
        byte_ = rx_buf_[read_buff_index_++];
        switch (unpack_step_e_) {
          case STEP_HEADER_SOF: {
            if (byte_ == UP_REG_ID) {
              protocol_packet_[index_++] = byte_;
              unpack_step_e_ = STEP_LENGTH_LOW;
            } else {
              index_ = 0;
            }
          }
            break;
          case STEP_LENGTH_LOW: {
            data_length_ = byte_;
            protocol_packet_[index_++] = byte_;
            unpack_step_e_ = STEP_LENGTH_HIGH;
          }
            break;
          case STEP_LENGTH_HIGH: {
            data_length_ |= (byte_ << 8);
            protocol_packet_[index_++] = byte_;
            if (data_length_ < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CMD_LEN - CRC_LEN)) {
              unpack_step_e_ = STEP_FRAME_SEQ;
            } else {
              std::cout << "Data length too big";
              unpack_step_e_ = STEP_HEADER_SOF;
              index_ = 0;
            }
          }
            break;
          case STEP_FRAME_SEQ: {
            protocol_packet_[index_++] = byte_;
            unpack_step_e_ = STEP_HEADER_CRC8;
          }
            break;
          case STEP_HEADER_CRC8: {
            protocol_packet_[index_++] = byte_;
            bool crc8_result = VerifyCrcOctCheckSum(protocol_packet_, HEADER_LEN);
            if (!crc8_result) {
              std::cout << "CRC 8 error";
            }
            if ((index_ == HEADER_LEN) && crc8_result) {
              if (index_ < HEADER_LEN) {
                std::cout << "CRC 8 index less.";
              }
              unpack_step_e_ = STEP_DATA_CRC16;
            } else {
              unpack_step_e_ = STEP_HEADER_SOF;
              index_ = 0;
            }
          }
            break;
          case STEP_DATA_CRC16: {
            if (index_ < (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
              protocol_packet_[index_++] = byte_;
            } else if (index_ > (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
              std::cout << "Index Beyond";
            }
            if (index_ == (HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
              unpack_step_e_ = STEP_HEADER_SOF;
              index_ = 0;
              if (VerifyCrcHexCheckSum(protocol_packet_, HEADER_LEN + CMD_LEN + data_length_ + CRC_LEN)) {
                DataHandle();
              } else {
                std::cout << "CRC16 error";
              }
            }
          }
            break;
          default: {
            std::cout << "Unpack not well";
            unpack_step_e_ = STEP_HEADER_SOF;
            index_ = 0;
          }
            break;
        }
      }
    }
  }
}


int SerialComNode::ReceiveData(int fd, int data_length) {
  int received_length, selected;
  fd_set fs_read;
  struct timeval time;
  static int is_init = 0;
  if (is_init == 0) {
    is_init++;
    FD_ZERO(&fs_read);
    FD_SET(fd, &fs_read);
  }
  FD_ZERO(&fs_read);
  FD_SET(fd, &fs_read);
  time.tv_sec = 0;
  time.tv_usec = 0;
  selected = select(fd + 1, &fs_read, NULL, NULL, &time);
  if (selected > 0) {
    received_length = read(fd, rx_buf_, data_length);
    int count = received_length;
    int p = 0;
    if (is_debug_) {
      fprintf(fp_, "%d,", received_length);
      while (count--) {
        fprintf(fp_, "%x,", rx_buf_[p++]);
      }
      fprintf(fp_, "\n");
      //fwrite(rx_buf_, sizeof(uint8_t), received_length, fp_);
      fflush(fp_);
    }
  } else if (selected == 0) {
    received_length = 0;
  } else {
    received_length = 0;
    std::cout << "Select function error";
  }
  return received_length;
}
serial_common::Infantrymode mode_msg_;

//--------------工控机收到的消息-----步兵用的mode----------
void SerialComNode::DataHandle() {
  ros::Time current_time = ros::Time::now();
  std::lock_guard<std::mutex> guard(mutex_receive_);
  auto *p_header = (FrameHeader *) protocol_packet_;
  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id = *(uint16_t *) (protocol_packet_ + HEADER_LEN);
  uint8_t *data_addr = protocol_packet_ + HEADER_LEN + CMD_LEN;
  switch (cmd_id) {

    case INFANTRY_SHOOT_MODE_ID:
      memcpy(&Gimbal_mode_msg_, data_addr, data_length);
      mode_msg_.mode = Gimbal_mode_msg_.mode;
      mode_pub_.publish(mode_msg_);
      std::cout<<"receive a message:"<<mode_msg_.mode<<std::endl;
      break;
    default:
      break;
  }
}

void SerialComNode::GimbalEnemyControlCallback(const serial_common::EnemyPos::ConstPtr &msg) {
  if (msg->enemy_dist == 0)
  {
    std::cout<<"自瞄距离数据为0，数据不传输！"<<std::endl;
    return;
  }
  static int count = 0, time_ms = 0, compress = 0;
  static double frequency = 0;
  static struct timeval time_last, time_current;
  gettimeofday(&time_current, nullptr);
  if (count == 0) {
    count++;
  } else {
    time_ms = (time_current.tv_sec - time_last.tv_sec) * 1000 + (time_current.tv_usec - time_last.tv_usec) / 1000;
    frequency = 1000.0 / time_ms;
  }
  time_last = time_current;
  std::unique_lock<std::mutex> lock(mutex_pack_);
  uint8_t pack[PACK_MAX_SIZE];
  GimbalShootControl gimbal_control_data;
  //TODO(krik): choose the mode by command
  gimbal_control_data.enemy_dist    =  msg->enemy_dist;
  gimbal_control_data.enemy_pitch   =  msg->enemy_pitch ;
  gimbal_control_data.enemy_yaw     =  msg->enemy_yaw ;
  gimbal_control_data.mode          =  msg->mode;

  std::cout  <<"yaw:"<<gimbal_control_data.enemy_yaw << "  pitch:"<<gimbal_control_data.enemy_pitch<<"  dist:"<< gimbal_control_data.enemy_dist <<"  mode :"<<gimbal_control_data.mode<< std::endl;
  int length = sizeof(GimbalShootControl), total_length = length + HEADER_LEN + CMD_LEN + CRC_LEN;
  SendDataHandle(INFANTRY_ENEMY_ID, (uint8_t *) &gimbal_control_data, pack, length);
  if (total_length <= free_length_) {
    memcpy(tx_buf_ + total_length_, pack, total_length);
    free_length_ -= total_length;
    total_length_ += total_length;
  } else {
    std::cout  << "Overflow in Gimbal CB";
  }
}
void SerialComNode::GimbalBuffControlCallback(const serial_common::EnemyPos::ConstPtr &msg) {
  if (msg->enemy_dist == 0)
  {
    std::cout<<"buff距离数据为0，数据不传输！"<<std::endl;
    return;
  }
  static int count = 0, time_ms = 0, compress = 0;
  static double frequency = 0;
  static struct timeval time_last, time_current;
  gettimeofday(&time_current, nullptr);
  if (count == 0) {
    count++;
  } else {
    time_ms = (time_current.tv_sec - time_last.tv_sec) * 1000 + (time_current.tv_usec - time_last.tv_usec) / 1000;
    frequency = 1000.0 / time_ms;
  }
  time_last = time_current;
  std::unique_lock<std::mutex> lock(mutex_pack_);
  uint8_t pack[PACK_MAX_SIZE];
  GimbalShootControl gimbal_control_data;
  //TODO(krik): choose the mode by command
  gimbal_control_data.enemy_dist    =  msg->enemy_dist;
  gimbal_control_data.enemy_pitch   =  msg->enemy_pitch ;
  gimbal_control_data.enemy_yaw     =  msg->enemy_yaw ;
  gimbal_control_data.mode          =  msg->mode;

  std::cout  <<"yaw:"<<gimbal_control_data.enemy_yaw << "  pitch:"<<gimbal_control_data.enemy_pitch<<"  dist:"<< gimbal_control_data.enemy_dist <<"  mode :"<<gimbal_control_data.mode<< std::endl;  int length = sizeof(GimbalShootControl), total_length = length + HEADER_LEN + CMD_LEN + CRC_LEN;
  SendDataHandle(INFANTRY_BUFF_ID, (uint8_t *) &gimbal_control_data, pack, length);
  if (total_length <= free_length_) {
    memcpy(tx_buf_ + total_length_, pack, total_length);
    free_length_ -= total_length;
    total_length_ += total_length;
  } else {
    std::cout  << "Overflow in Gimbal CB";
  }
}

void SerialComNode::WifiBuffCallback(const serial_common::wifi_buff::ConstPtr &msg) {

  static int count = 0, time_ms = 0, compress = 0;
  static double frequency = 0;
  static struct timeval time_last, time_current;
  gettimeofday(&time_current, nullptr);
  if (count == 0) {
    count++;
  } else {
    time_ms = (time_current.tv_sec - time_last.tv_sec) * 1000 + (time_current.tv_usec - time_last.tv_usec) / 1000;
    frequency = 1000.0 / time_ms;
  }
  time_last = time_current;
  std::unique_lock<std::mutex> lock(mutex_pack_);
  uint8_t pack[PACK_MAX_SIZE];
  Wifi_Send wifi_buff_data;

  //TODO(krik): choose the mode by command
  wifi_buff_data.left    =  msg->left;
  wifi_buff_data.right   =  msg->right ;

  std::cout  <<endl;
  std::cout  <<"  left:"<<wifi_buff_data.left << "  right:"<<wifi_buff_data.right<<endl;
  int length = sizeof(wifi_buff_data), total_length = length + HEADER_LEN + CMD_LEN + CRC_LEN;
  SendDataHandle(INFANTRY_WIFI_ID, (uint8_t *) &wifi_buff_data, pack, length);
  if (total_length <= free_length_) {
    memcpy(tx_buf_ + total_length_, pack, total_length);
    free_length_ -= total_length;
    total_length_ += total_length;
  } else {
    std::cout  << "Overflow in Gimbal CB";
  }
}

void SerialComNode::ChassisControlCallback(const geometry_msgs::Twist::ConstPtr &vel) {
  static int count = 0, time_ms = 0, compress = 0;
  static double frequency = 0;
  static struct timeval time_last, time_current;
  gettimeofday(&time_current, nullptr);
  if (count == 0) {
    count++;
  } else {
    time_ms = (time_current.tv_sec - time_last.tv_sec) * 1000 + (time_current.tv_usec - time_last.tv_usec) / 1000;
    frequency = 1000.0 / time_ms;
  }
  time_last = time_current;
  compress++;
  if (compress == COMPRESS_TIME) {
    std::unique_lock<std::mutex> lock(mutex_pack_);
    compress = 0;
    uint8_t pack[PACK_MAX_SIZE];
    ChassisControl chassis_control_data;
    //TODO(Krik): get the effective command from the decision module
    //chassis_control_data.ctrl_mode = chassis_ctrl_mode;    //AUTO_FOLLOW_GIMBAL

    chassis_control_data.x_speed = vel->linear.x * 1000.0;
    chassis_control_data.y_speed = vel->linear.y * 1000.0;
    chassis_control_data.w_info.x_offset = 0;
    chassis_control_data.w_info.y_offset = 0;
    chassis_control_data.w_info.w_speed = vel->angular.z * 180.0 / M_PI;
    int length = sizeof(ChassisControl), pack_length = length + HEADER_LEN + CMD_LEN + CRC_LEN;
    SendDataHandle(CHASSIS_CTRL_ID, (uint8_t *) &chassis_control_data, pack, length);
    if (pack_length <= free_length_) {
      memcpy(tx_buf_ + total_length_, pack, pack_length);
      free_length_ -= pack_length;
      total_length_ += pack_length;
    } else {
      std::cout << "Overflow in Chassis CB";
    }
  }
}

void SerialComNode::SendDataHandle(uint16_t cmd_id,
                                   uint8_t *topack_data,
                                   uint8_t *packed_data,
                                   uint16_t len
) {
  FrameHeader *p_header = (FrameHeader *) packed_data;
  p_header->sof = UP_REG_ID;
  p_header->data_length = len;
  memcpy(packed_data + HEADER_LEN, (uint8_t *) &cmd_id, CMD_LEN);
  AppendCrcOctCheckSum(packed_data, HEADER_LEN);
  memcpy(packed_data + HEADER_LEN + CMD_LEN, topack_data, len);
  AppendCrcHexCheckSum(packed_data, HEADER_LEN + CMD_LEN + CRC_LEN + len);
}

void SerialComNode::SendPack() {
  while (is_open_ && !stop_send_ && ros::ok()) {
    if (total_length_ > 0) {
      mutex_send_.lock();
      SendData(total_length_);
      total_length_ = 0;
      free_length_ = UART_BUFF_SIZE;
      mutex_send_.unlock();
    } else {
      usleep(100);
    }
  }
}

int SerialComNode::SendData(int data_len) {
  int length = 0;
  length = write(fd_, tx_buf_, data_len);
  lseek(fd_, 0, SEEK_CUR);
  std::cout << "Com sending: " << length << std::endl;
  if (length == data_len) {
    return length;
  } else {
    std::cout << "Serial write error";
    return -1;
  }
}

SerialComNode::~SerialComNode() {
  if (receive_loop_thread_ != nullptr) {
    stop_receive_ = true;
    receive_loop_thread_->join();
    delete receive_loop_thread_;
  }
  if (send_loop_thread_ != nullptr) {
    stop_send_ = true;
    send_loop_thread_->join();
    delete send_loop_thread_;
  }
  tcsetattr(fd_, TCSANOW, &termios_options_original_);
  close(fd_);
  is_open_ = false;
  if (is_debug_) {
    fclose(fp_);
  }
}
void SerialComNode::Stop() {
  stop_send_ = true;
  stop_receive_ = true;
}

void SerialComNode::Resume() {
  stop_send_ = false;
  stop_receive_ = false;
}

uint8_t SerialComNode::GetCrcOctCheckSum(uint8_t *message, uint32_t length, uint8_t crc) {
  uint8_t index;
  while (length--) {
    index = crc ^ (*message++);
    crc = kCrcOctTable[index];
  }
  return (crc);
}

bool SerialComNode::VerifyCrcOctCheckSum(uint8_t *message, uint16_t length) {
  uint8_t expected = 0;
  if ((message == 0) || (length <= 2)) {
    std::cout << "Verify CRC8 false";
    return false;
  }
  expected = GetCrcOctCheckSum(message, length - 1, kCrc8);
  return (expected == message[length - 1]);
}

void SerialComNode::AppendCrcOctCheckSum(uint8_t *message, uint16_t length) {
  uint8_t crc = 0;
  if ((message == 0) || (length <= 2)) {
    std::cout << "Append CRC8 NULL";
    return;
  };
  crc = GetCrcOctCheckSum(message, length - 1, kCrc8);
  message[length - 1] = crc;
}

uint16_t SerialComNode::GetCrcHexCheckSum(uint8_t *message, uint32_t length, uint16_t crc) {
  uint8_t data;
  if (message == NULL) {
    return 0xFFFF;
  }
  while (length--) {
    data = *message++;
    (crc) = ((uint16_t) (crc) >> 8) ^ kCrcTable[((uint16_t) (crc) ^ (uint16_t) (data)) & 0x00ff];
  }
  return crc;
}

bool SerialComNode::VerifyCrcHexCheckSum(uint8_t *message, uint32_t length) {
  uint16_t expected = 0;
  if ((message == NULL) || (length <= 2)) {
    std::cout << "Verify CRC16 bad";
    return false;
  }
  expected = GetCrcHexCheckSum(message, length - 2, kCrc);
  return ((expected & 0xff) == message[length - 2] && ((expected >> 8) & 0xff) == message[length - 1]);
}

void SerialComNode::AppendCrcHexCheckSum(uint8_t *message, uint32_t length) {
  uint16_t crc = 0;
  if ((message == NULL) || (length <= 2)) {
    std::cout << "Append CRC 16 NULL";
    return;
  }
  crc = GetCrcHexCheckSum(message, length - 2, kCrc);
  message[length - 2] = (uint8_t) (crc & 0x00ff);
  message[length - 1] = (uint8_t) ((crc >> 8) & 0x00ff);
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"serial_common_node",ros::init_options::NoSigintHandler);
  SerialComNode serial_common_node_;
  serial_common_node_.Run();
  return 0;
}
