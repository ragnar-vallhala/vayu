// fifo_transport.h — non-blocking FIFO I/O wrappers for vsim_d.
//
// Each FIFO is opened O_RDWR | O_NONBLOCK so the open() never blocks
// regardless of which side connects first. Writes that would block
// (no reader, buffer full) drop the frame silently -- this is fine for
// every channel because all four are "latest wins" with sequence
// numbers, and a missed frame is recovered by the next one.
#ifndef VSIM_FIFO_TRANSPORT_H
#define VSIM_FIFO_TRANSPORT_H

#include "vsim_proto.h"

#include <cstddef>
#include <cstdint>
#include <string>

namespace vsim {

class FifoIn {
public:
    explicit FifoIn(const std::string& path) : path_(path) {}
    ~FifoIn();

    // Create the FIFO if it doesn't exist (mkfifo 0666) and open it
    // non-blocking. Idempotent on the file system side; safe to call
    // multiple times.
    bool open();

    // Drain whatever bytes are available, append to the internal buffer,
    // and return the latest complete frame of the expected `type`. Frames
    // older than the latest are dropped (latest-wins). Returns false if
    // no new frame is available.
    bool poll(uint16_t type, void* out_frame, size_t out_size);

    int fd() const { return fd_; }

private:
    std::string path_;
    int         fd_ = -1;
    std::string buf_;
};

class FifoOut {
public:
    explicit FifoOut(const std::string& path) : path_(path) {}
    ~FifoOut();

    bool open();

    // Write a single frame. Drops on EAGAIN (no reader / pipe full).
    // Returns true on full write, false otherwise.
    bool write(const void* frame, size_t size);

    int fd() const { return fd_; }

private:
    std::string path_;
    int         fd_ = -1;
};

}  // namespace vsim

#endif  // VSIM_FIFO_TRANSPORT_H
