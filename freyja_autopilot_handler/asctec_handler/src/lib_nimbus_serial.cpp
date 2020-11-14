/*
 * lib_nimbus_serial.cpp
 *
 *  Created on: Jul 3, 2014
 *      Author: danthony
 */

#include "lib_nimbus_serial.h"

int32_t LibNimbusSerial::initSerialInterface(
  const std::string &serial_device,
  const int &serial_baudrate,
  int &serial_fd)
  {
  struct termios serial_params;

  /* Try to open device */
  if((serial_fd = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
    {
    ROS_ERROR("Error opening serial device %s", serial_device.c_str());
    return -1;
      }

  /* Clear pending IO */
  tcflush(serial_fd, TCOFLUSH);
  tcflush(serial_fd, TCIFLUSH);

  /* Set blocking mode */
  fcntl(serial_fd, F_SETFL, fcntl(serial_fd, F_GETFL) & ~O_NONBLOCK);

  /* Get current parameters */
  if(tcgetattr(serial_fd, &serial_params))
    {
    ROS_ERROR("Error getting serial device params");
    return -1;
    }

  /* Set new parameters */
  serial_params.c_cflag = CS8 | CLOCAL | CREAD;
  setBaudRate(serial_params, serial_baudrate);
  serial_params.c_iflag = IGNPAR | IGNBRK;
  serial_params.c_oflag = 0;
  serial_params.c_lflag = 0;
  cfmakeraw(&serial_params);
  serial_params.c_cc[VTIME] = 1;
  serial_params.c_cc[VMIN] = 0;

  tcflush(serial_fd, TCIFLUSH);
  if(tcsetattr(serial_fd, TCSANOW, &serial_params))
    {
    ROS_ERROR("Error setting serial device params");
    return -1;
    }

  return 0;
  }

int32_t LibNimbusSerial::initSerialInterface(
  const std::string &serial_device,
  const int &serial_baudrate,
  void (*callback) (int),
  int &serial_fd)
  {
  struct termios new_serial_params;
  struct sigaction saio;

  /* Try to open device */
  if((serial_fd = open(serial_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
    {
    ROS_ERROR("Error opening serial device %s", serial_device.c_str());
    return -1;
    }

  /* Install the signal handler */
  saio.sa_handler = callback;
  sigemptyset(&saio.sa_mask);
  /* Resume function if interrupted */
  saio.sa_flags = SA_RESTART;
  #ifndef __APPLE__
  saio.sa_restorer = NULL;
  #endif
  if(sigaction(SIGIO, &saio, NULL) == -1)
    {
    ROS_BREAK();
    }

  /* Set the process to receive the signal interrupt */
  fcntl(serial_fd, F_SETOWN, getpid());
  /* Make the file descriptor asynchronous and nonblocking, while saving the other attributes */
  fcntl(serial_fd, F_SETFL, O_ASYNC | O_NONBLOCK | fcntl(serial_fd, F_GETFL));

  /* Set up the new settings */
  new_serial_params.c_cflag = (CS8 | CLOCAL | CREAD);
  new_serial_params.c_cflag &= ~CRTSCTS;
  setBaudRate(new_serial_params, serial_baudrate);

  /* Turn off flow control */
  new_serial_params.c_iflag &= ~(IXON | IXOFF | IXANY);
  /* Don't need parity checks */
  new_serial_params.c_iflag |= IGNPAR;
  new_serial_params.c_oflag = 0;
  /* Set for raw mode */
  new_serial_params.c_lflag = ~(ICANON | ECHO | ECHOE | ISIG);
  new_serial_params.c_cc[VMIN] = 1;
  new_serial_params.c_cc[VMIN] = 0;

  /* Clear pending IO */
  tcflush(serial_fd, TCOFLUSH);
  tcflush(serial_fd, TCIFLUSH);

  if(tcsetattr(serial_fd, TCSANOW, &new_serial_params))
    {
    ROS_ERROR("Error setting serial device parameters");
    return -1;
    }

  return 0;
  }
  
int32_t LibNimbusSerial::closeSerialInterface( const int32_t &serial_fd )
{
  if( close( serial_fd ) < 0 )
    ROS_ERROR( "Failed to close serial interface. Good luck!" );
  else
    ROS_INFO( "Serial interface closed." );
}

// write data to serial device
int32_t LibNimbusSerial::sendSerial(
  const int32_t &serial_fd,
  const std::vector<uint8_t> &data)
  {
  // local stack
  int32_t return_val;
  uint32_t sent_bytes = 0;

  // write data to serial device
  while(sent_bytes < data.size())
    {
    if((return_val = write(serial_fd, &(data[sent_bytes]), data.size() - sent_bytes)) < 0)
      {
      ROS_ERROR("Error writing serial buffer");
      return -1;
      }
    sent_bytes += return_val;
    }

  // clear buffers
  tcflush(serial_fd, TCOFLUSH);

  // return number of sent bytes
  return static_cast<int32_t>(sent_bytes);
  }

void LibNimbusSerial::setBaudRate(termios &param, const int32_t rate)
  {
  #ifdef __APPLE__
  switch(rate)
      {
      case(230400):
      param.c_cflag |= B230400;
      break;

    case(115200):
      param.c_cflag |= B115200;
      break;

    case(57600):
      param.c_cflag |= B57600;
      break;

    case(38400):
      param.c_cflag |= B38400;
      break;

    case(19200):
      param.c_cflag |= B19200;
      break;

    case(9600):
      param.c_cflag |= B9600;
      break;

    default:
      ROS_ERROR("Unknown baudrate %d", rate);
      break;
        }
  #else
  switch(rate)
    {
    case(921600):
      param.c_cflag |= B921600;
      break;

    case(576000):
      param.c_cflag |= B576000;
      break;

    case(500000):
      param.c_cflag |= B500000;
      break;

    case(460800):
      param.c_cflag |= B460800;
      break;

    case(230400):
      param.c_cflag |= B230400;
      break;

    case(115200):
      param.c_cflag |= B115200;
      break;

    case(57600):
      param.c_cflag |= B57600;
      break;

    case(38400):
      param.c_cflag |= B38400;
      break;

    case(19200):
      param.c_cflag |= B19200;
      break;

    case(9600):
      param.c_cflag |= B9600;
      break;

    default:
      ROS_ERROR("Unknown baudrate %d", rate);
      break;
        }
  #endif
  }

// read data from serial device
int32_t LibNimbusSerial::readSerial(
  const int32_t &serial_fd,
  const int32_t &data_size,
  std::vector<uint8_t> &data)
  {
  // local stack
  int32_t recv_bytes = 0;

  // read data from serial device
  if((recv_bytes = read(serial_fd, data.data(), data_size)) < 0)
    {
    ROS_ERROR("Error reading serial buffer: %s\n", strerror(errno));
    return -1;
    }

  // clear buffers
  tcflush(serial_fd, TCIFLUSH);

  // return number of received bytes
  return recv_bytes;
  }

uint16_t LibNimbusSerial::calcCrc16(const std::vector<uint8_t> &buf, const uint16_t len)
  {
  uint8_t bits = 0;
  uint16_t crc = 0xFFFF;
  int32_t idx = 0;

  for(idx = 0; idx < len; idx++)
    {
    bits = ((unsigned char)(crc >> 8)) ^ buf[idx];
    bits ^= (bits >> 4);
    /* Do polynomial division */
    crc = (crc << 8) ^
      ((unsigned short)(bits << 12)) ^
      ((unsigned short)(bits << 5)) ^
      ((unsigned short)bits);
    }

  return crc;
  }

bool LibNimbusSerial::validCrc16(
  const std::vector<uint8_t> &buf,
  const uint16_t len,
  const uint16_t rcvd_crc)
  {
  return (rcvd_crc == calcCrc16(buf, len));
  }

/* From AscTec Documentation "Communication with the Low Level Processor" */
bool LibNimbusSerial::validCrcAsctec(
  const std::vector<uint8_t> &buf,
  const uint16_t rcvd_chksum)
  {
  uint16_t calculated_crc = 0xFF;

  for(uint8_t new_byte : buf)
    {
    asctecCrcHelper(calculated_crc, new_byte);
    }

  return (calculated_crc == rcvd_chksum);
  }

void LibNimbusSerial::asctecCrcHelper(uint16_t &crc, uint8_t new_byte)
  {
  new_byte ^= (crc & 0xFF);
  new_byte ^= (new_byte << 4);
  crc = (((((uint16_t)new_byte) << 8) |
    ((crc >> 8) & (uint16_t)0xFF)) ^
    (uint8_t)(new_byte >> 4) ^
    (((uint16_t) new_byte) << 3));
  }

void LibNimbusSerial::pack8(const int8_t val, const uint8_t idx, std::vector<uint8_t> &buf)
  {
  buf[idx] = static_cast<uint8_t>(val);
  }

void LibNimbusSerial::pack16(const int16_t val, const uint8_t idx, std::vector<uint8_t> &buf)
  {
  buf[idx - 1] = static_cast<uint8_t>(val >> 8);
  buf[idx] = static_cast<uint8_t>(val);
  }

void LibNimbusSerial::pack32(const int32_t val, const uint8_t idx, std::vector<uint8_t> &buf)
  {
  buf[idx - 3] = static_cast<uint8_t>(val >> 24);
  buf[idx - 2] = static_cast<uint8_t>(val >> 16);
  buf[idx - 1] = static_cast<uint8_t>(val >> 8);
  buf[idx] = static_cast<uint8_t>(val);
  }

void LibNimbusSerial::packU8(const uint8_t val, const uint8_t idx, std::vector<uint8_t> &buf)
  {
  buf[idx] = val;
  }

void LibNimbusSerial::packU16(const uint16_t val, const uint8_t idx, std::vector<uint8_t> &buf)
  {
  buf[idx - 1] = static_cast<uint8_t>(val >> 8);
  buf[idx] = static_cast<uint8_t>(val);
  }

void LibNimbusSerial::packU32(const uint32_t val, const uint8_t idx, std::vector<uint8_t> &buf)
  {
  buf[idx - 3] = static_cast<uint8_t>(val >> 24);
  buf[idx - 2] = static_cast<uint8_t>(val >> 16);
  buf[idx - 1] = static_cast<uint8_t>(val >> 8);
  buf[idx] = static_cast<uint8_t>(val);
  }

void LibNimbusSerial::packReversed16(const int16_t val, const uint8_t idx, std::vector<uint8_t> &buf)
  {
  buf[idx] = static_cast<uint8_t>(val >> 8);
  buf[idx - 1] = static_cast<uint8_t>(val);
  }

void LibNimbusSerial::packReversed32(const int32_t val, const uint8_t idx, std::vector<uint8_t> &buf)
  {
  buf[idx] = static_cast<uint8_t>(val >> 24);
  buf[idx - 1] = static_cast<uint8_t>(val >> 16);
  buf[idx - 2] = static_cast<uint8_t>(val >> 8);
  buf[idx - 3] = static_cast<uint8_t>(val);
  }

void LibNimbusSerial::packReversedU16(const uint16_t val, const uint8_t idx, std::vector<uint8_t> &buf)
  {
  buf[idx] = static_cast<uint8_t>(val >> 8);
  buf[idx - 1] = static_cast<uint8_t>(val);
  }

void LibNimbusSerial::packReversedU32(const uint32_t val, const uint8_t idx, std::vector<uint8_t> &buf)
  {
  buf[idx] = static_cast<uint8_t>(val >> 24);
  buf[idx - 1] = static_cast<uint8_t>(val >> 16);
  buf[idx - 2] = static_cast<uint8_t>(val >> 8);
  buf[idx - 3] = static_cast<uint8_t>(val);
  }

int8_t LibNimbusSerial::unpack8(
  const std::vector<uint8_t> &buf,
  const uint8_t idx)
  {
  int8_t ret_val = 0;
  ret_val |= buf[idx];
  return ret_val;
  }

int16_t LibNimbusSerial::unpack16(
  const std::vector<uint8_t> &buf,
  const uint8_t idx)
  {
  int16_t ret_val = 0;
  ret_val |= (buf[idx - 1] << 8);
  ret_val |= buf[idx];
  return ret_val;
  }

int32_t LibNimbusSerial::unpack32(
  const std::vector<uint8_t> &buf,
  const uint8_t idx)
  {
  int32_t ret_val = 0;
  ret_val |= (buf[idx - 3] << 24);
  ret_val |= (buf[idx - 2] << 16);
  ret_val |= (buf[idx - 1] << 8);
  ret_val |= buf[idx];
  return ret_val;
  }

uint8_t LibNimbusSerial::unpackU8(
  const std::vector<uint8_t> &buf,
  const uint8_t idx)
  {
  uint8_t ret_val = 0;
  ret_val |= buf[idx];
  return ret_val;
  }

uint16_t LibNimbusSerial::unpackU16(
  const std::vector<uint8_t> &buf,
  const uint8_t idx)
  {
  uint16_t ret_val = 0;
  ret_val |= (buf[idx - 1] << 8);
  ret_val |= buf[idx];
  return ret_val;
  }

uint32_t LibNimbusSerial::unpackU32(
  const std::vector<uint8_t> &buf,
  const uint8_t idx)
  {
  uint32_t ret_val = 0;
  ret_val |= (buf[idx - 3] << 24);
  ret_val |= (buf[idx - 2] << 16);
  ret_val |= (buf[idx - 1] << 8);
  ret_val |= buf[idx];
  return ret_val;
  }

/* Unpack a signed 16 bit number with the endianness reversed */
int16_t LibNimbusSerial::unpackReversed16(
  const std::vector<uint8_t> &buf,
  const uint8_t idx)
  {
  int16_t ret_val = 0;
  ret_val |= (((int16_t)buf[idx]) << 8);
  ret_val |= (int16_t)buf[idx - 1];
  return ret_val;
  }

/* Unpack a signed 32 bit number with the endianness reversed */
int32_t LibNimbusSerial::unpackReversed32(
  const std::vector<uint8_t> &buf,
  const uint8_t idx)
  {
  int32_t ret_val = 0;
  ret_val |= (((int32_t)buf[idx]) << 24);
  ret_val |= (((int32_t)buf[idx - 1]) << 16);
  ret_val |= (((int32_t)buf[idx - 2]) << 8);
  ret_val |= (int32_t)buf[idx - 3];
  return ret_val;
  }

/* Unpack an unsigned 16 bit number with the endianness reversed */
uint16_t LibNimbusSerial::unpackReversedU16(
  const std::vector<uint8_t> &buf,
  const uint8_t idx)
  {
  uint16_t ret_val = 0;
  ret_val |= (((uint16_t)buf[idx]) << 8);
  ret_val |= (uint16_t)buf[idx - 1];
  return ret_val;
  }

/* Unpack an unsigned 32 bit number with the endianness reversed */
uint32_t LibNimbusSerial::unpackReversedU32(
  const std::vector<uint8_t> &buf,
  const uint8_t idx)
  {
  uint32_t ret_val = 0;
  ret_val |= (((uint32_t)buf[idx]) << 24);
  ret_val |= (((uint32_t)buf[idx - 1]) << 16);
  ret_val |= (((uint32_t)buf[idx - 2]) << 8);
  ret_val |= (uint32_t)buf[idx - 3];
  return ret_val;
  }

