#include "midi.h"

#include <array>
#include <boost/asio.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <chrono>
#include <iostream>
#include <mutex>
#include <queue>

#include "constants.h"

using namespace boost;
namespace bip = boost::interprocess;

namespace superposition::midi {

std::string DEFAULT_DEVICE("/dev/ttyUSB0");
unsigned int DEFAULT_BAUDRATE = 38400;

MidiTask::MidiTask() : MidiTask(DEFAULT_DEVICE, DEFAULT_BAUDRATE) {
}

MidiTask::MidiTask(std::string &device, unsigned int baudrate)
    : common::AbstractTask(),
      _device(device),
      _baudrate(baudrate),
      _message_buffer(100) {
    _serial = new asio::serial_port(_io);
}

MidiTask::~MidiTask() {
    stop();
    delete _serial;
}

void MidiTask::stop() {
    AbstractTask::stop();
    if (_serial->is_open()) {
        _serial->cancel();
        _serial->close();
    }
}

void MidiTask::process() {
    try {
        std::array<unsigned char, 3> buffer;
        size_t read_size;
        midi::ControlChange *cc;

        if (!_serial->is_open()) {
            _serial->open(_device);
            _serial->set_option(asio::serial_port_base::character_size(8));
            _serial->set_option(asio::serial_port_base::stop_bits(
                asio::serial_port_base::stop_bits::one));
            _serial->set_option(asio::serial_port_base::parity(
                asio::serial_port_base::parity::none));
            _serial->set_option(asio::serial_port_base::flow_control(
                asio::serial_port_base::flow_control::none));
            _serial->set_option(asio::serial_port_base::baud_rate(_baudrate));
        }

        while (_is_started.load()) {
            read_size = asio::read(*_serial, asio::buffer(buffer, 1));
            if (read_size < 1) {
                continue;
            }

            unsigned char v = buffer[0];
            if (v & 0x80 == 0) {  // v < 0x80
                continue;
            }

            auto status = v & 0xF0;
            auto channel = v & 0x0F;

            if (status != 0xB0) {
                continue;
            }

            read_size = asio::read(*_serial, asio::buffer(&buffer[1], 2));
            if (read_size != 2) {
                continue;
            }
            auto control_number = buffer[0];
            auto value = buffer[0];

            cc = (midi::ControlChange *)&buffer;

            _message_buffer.push(*cc);
        }
    } catch (std::exception &e) {
        std::cerr << "error: " << e.what() << std::endl;
    }
}

}  // namespace superposition::midi