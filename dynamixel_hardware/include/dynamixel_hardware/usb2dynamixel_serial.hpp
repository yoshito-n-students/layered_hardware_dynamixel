#ifndef DYNAMIXEL_HARDWARE_USB2DYNAMIXEL_SERIAL_HPP
#define DYNAMIXEL_HARDWARE_USB2DYNAMIXEL_SERIAL_HPP

#include <stdexcept>
#include <string>
#include <vector>

#include <dynamixel_hardware/common_namespaces.hpp>
#include <serial/serial.h>

#include <dynamixel/errors.hpp>
#include <dynamixel/instruction_packet.hpp>
#include <dynamixel/status_packet.hpp>

#include <boost/lexical_cast.hpp>

namespace dynamixel_hardware {

// An equivarent of dynamixel::Usb2Dynamixel based on serial::Serial.
// This has to be implemented because the original Usb2Dynamixel
// contains a busy read loop that leads a heavy CPU usage.
class Usb2DynamixelSerial {
public:
  Usb2DynamixelSerial(const std::string &port, const uint32_t baudrate = 115200,
                      const double timeout = 0.1) {
    set_recv_timeout(timeout);
    open_serial(port, baudrate);
  }

  Usb2DynamixelSerial() { set_recv_timeout(0.1); }

  virtual ~Usb2DynamixelSerial() {}

  void open_serial(const std::string &port, const uint32_t baudrate = 115200) {
    try {
      serial_.setPort(port);
      serial_.setBaudrate(baudrate);
      serial_.open();
    } catch (const std::exception &err) {
      throw de::Error(std::string("Usb2DynamixelSerial::open_serial(): ") + err.what());
    }
  }

  void close_serial() {
    // won't throw
    serial_.close();
  }

  bool is_open() const {
    // won't throw
    return serial_.isOpen();
  }

  void flush() {
    // won't throw
    serial_.flush();
  }

  double recv_timeout() const {
    // won't throw
    return serial_.getTimeout().read_timeout_constant / 1000.;
  }

  void set_recv_timeout(const double recv_timeout) {
    // won't throw
    serial::Timeout timeout(
        serial::Timeout::simpleTimeout(static_cast< uint32_t >(recv_timeout * 1000)));
    serial_.setTimeout(timeout);
  }

  // general send
  template < typename T > void send(const dynamixel::InstructionPacket< T > &packet) {
    if (!is_open()) {
      return;
    }

    size_t res;
    try {
      res = serial_.write(packet.data(), packet.size());
    } catch (const std::exception &err) {
      throw de::Error(std::string("Usb2DynamixelSerial::send(): ") + err.what());
    }

    if (res != packet.size()) {
      throw de::Error("Usb2DynamixelSerial::send(): Packet not fully written (" +
                      boost::lexical_cast< std::string >(res) + " of " +
                      boost::lexical_cast< std::string >(packet.size()) + " bytes)");
    }
  }

  // general receive
  template < typename T > bool recv(dynamixel::StatusPacket< T > &status) {
    if (!is_open()) {
      return false;
    }

    typename T::DecodeState state = T::DecodeState::ONGOING;
    std::vector< uint8_t > packet;
    do {
      uint8_t byte;
      size_t res;
      try {
        res = serial_.read(&byte, 1);
      } catch (const std::exception &err) {
        throw de::Error(std::string("Usb2DynamixelSerial::recv(): ") + err.what());
      }

      if (res > 0) {
        packet.push_back(byte);

        state = status.decode_packet(packet, report_bad_packet_);
        if (state == T::DecodeState::INVALID) {
          packet.clear();
        }
      } else if (res == 0) {
        // timeout
        return false;
      }
    } while (state != T::DecodeState::DONE);

    return true;
  }

  void set_report_bad_packet(const bool report_bad_packet) {
    report_bad_packet_ = report_bad_packet;
  }

  bool report_bad_packet() { return report_bad_packet_; }

private:
  serial::Serial serial_;
  bool report_bad_packet_;
};
} // namespace dynamixel_hardware

#endif