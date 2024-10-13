#ifndef __SUPERPOSITION_MIDI_H__
#define __SUPERPOSITION_MIDI_H__
#include <atomic>
#include <boost/asio.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <string>
#include <thread>

#include "common.h"

using namespace superposition;
using namespace boost;

namespace superposition::midi {

namespace bip = boost::interprocess;

#pragma pack(push, 1)  // disable padding
struct ControlChange {
    unsigned char channel : 4;     // 下位4ビットがチャンネル
    unsigned char status : 4;      // 上位4ビットがステータス
    unsigned char control_number;  // コントロール番号
    unsigned char value;           // コントロール値
} __attribute__((packed));
#pragma pack(pop)  // re-enable padding

class MidiTask : public common::AbstractTask {
   public:
    MidiTask();
    MidiTask(std::string& device, unsigned int baudrate);
    virtual ~MidiTask();

    virtual void process();
    virtual void stop();

    common::MessageBuffer<ControlChange>* message_buffer() {
        return &_message_buffer;
    }

   private:
    asio::serial_port* _serial;
    asio::io_context _io;
    std::string& _device;
    unsigned int _baudrate;
    common::MessageBuffer<ControlChange> _message_buffer;
};

}  // namespace superposition::midi
#endif