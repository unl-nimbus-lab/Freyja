/*
 * serial_interface.h
 *
 *  Created on: Jul 3, 2014
 *      Author: danthony
 */

#ifndef LIB_NIMBUS_SERIAL_H_
#define LIB_NIMBUS_SERIAL_H_

// includes
#include <ros/ros.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>
#include <errno.h>

namespace LibNimbusSerial
  {
  /* Open up a serial port for polling  or writing to */
  int32_t initSerialInterface(
    const std::string &serial_device,
    const int &serial_baudrate,
    int &serial_fd);

  /* Open up an asynchronous serial port for reading or writing from */
  int32_t initSerialInterface(
    const std::string &serial_device,
    const int &serial_baudrate,
    void (*callback) (int),
    int &serial_fd);
    
  int32_t closeSerialInterface( const int32_t &serial_fd );

  /* Write data out through serial port */
  int32_t sendSerial(
    const int32_t &serial_fd,
    const std::vector<uint8_t> &data);

  /* Set baudrate of serial port */
    void setBaudRate(termios &param, const int32_t rate);

  /* Read data from serial port */
  int32_t readSerial(
    const int32_t &serial_fd,
    const int32_t &data_size,
        std::vector<uint8_t> &data);

  uint16_t calcCrc16(const std::vector<uint8_t> &buf, const uint16_t len);

  bool validCrc16(
    const std::vector<uint8_t> &buf,
    const uint16_t len,
    const uint16_t rcvd_crc);

  bool validCrcAsctec(
    const std::vector<uint8_t> &buf,
    const uint16_t rcvd_chksum);
  void asctecCrcHelper(uint16_t &crc, uint8_t new_byte);

  /* Pack buffers with raw data types */
  void pack8(const int8_t val, const uint8_t idx, std::vector<uint8_t> &buf);
  void pack16(const int16_t val, const uint8_t idx, std::vector<uint8_t> &buf);
  void pack32(const int32_t val, const uint8_t idx, std::vector<uint8_t> &buf);
  void packU8(const uint8_t val, const uint8_t idx, std::vector<uint8_t> &buf);
  void packU16(const uint16_t val, const uint8_t idx, std::vector<uint8_t> &buf);
  void packU32(const uint32_t val, const uint8_t idx, std::vector<uint8_t> &buf);
  void packReversed16(const int16_t val, const uint8_t idx, std::vector<uint8_t> &buf);
  void packReversed32(const int32_t val, const uint8_t idx, std::vector<uint8_t> &buf);
  void packReversedU16(const uint16_t val, const uint8_t idx, std::vector<uint8_t> &buf);
  void packReversedU32(const uint32_t val, const uint8_t idx, std::vector<uint8_t> &buf);

  /* Convert data in serial buffer to primitive data type */
  int8_t unpack8(const std::vector<uint8_t> &buf, const uint8_t idx);
  int16_t unpack16(const std::vector<uint8_t> &buf, const uint8_t idx);
  int32_t unpack32(const std::vector<uint8_t> &buf, const uint8_t idx);
  uint8_t unpackU8(const std::vector<uint8_t> &buf, const uint8_t idx);
  uint16_t unpackU16(const std::vector<uint8_t> &buf, const uint8_t idx);
  uint32_t unpackU32(const std::vector<uint8_t> &buf, const uint8_t idx);
  int16_t unpackReversed16(const std::vector<uint8_t> &buf, const uint8_t idx);
  int32_t unpackReversed32(const std::vector<uint8_t> &buf, const uint8_t idx);
  uint16_t unpackReversedU16(const std::vector<uint8_t> &buf, const uint8_t idx);
  uint32_t unpackReversedU32(const std::vector<uint8_t> &buf, const uint8_t idx);
  }
#endif /* LIB_NIMBUS_SERIAL_H_ */
