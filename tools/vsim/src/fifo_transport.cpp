#include "fifo_transport.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

namespace vsim {

namespace {

// Hunt for the next VSIM_MAGIC in `buf`, return offset or std::string::npos.
size_t findMagic(const std::string& buf, size_t from = 0) {
    const uint32_t magic = VSIM_MAGIC;
    const char* m = reinterpret_cast<const char*>(&magic);
    for (size_t i = from; i + 4 <= buf.size(); ++i) {
        if (std::memcmp(buf.data() + i, m, 4) == 0) return i;
    }
    return std::string::npos;
}

bool ensureFifo(const std::string& path) {
    struct stat st;
    if (::stat(path.c_str(), &st) == 0) {
        // Exists. Trust it's a FIFO; we never put anything else at /tmp/vsim_*.
        return true;
    }
    if (::mkfifo(path.c_str(), 0666) != 0 && errno != EEXIST) {
        std::fprintf(stderr, "vsim: mkfifo %s failed: %s\n",
                     path.c_str(), std::strerror(errno));
        return false;
    }
    return true;
}

}  // namespace

FifoIn::~FifoIn() {
    if (fd_ >= 0) ::close(fd_);
}

bool FifoIn::open() {
    if (!ensureFifo(path_)) return false;
    fd_ = ::open(path_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
        std::fprintf(stderr, "vsim: open %s failed: %s\n",
                     path_.c_str(), std::strerror(errno));
        return false;
    }
    return true;
}

bool FifoIn::poll(uint16_t type, void* out_frame, size_t out_size) {
    if (fd_ < 0) return false;

    // Drain everything available right now into buf_.
    char chunk[1024];
    while (true) {
        ssize_t r = ::read(fd_, chunk, sizeof(chunk));
        if (r <= 0) break;
        buf_.append(chunk, static_cast<size_t>(r));
    }

    // Walk forward, dispatching frames. Keep only the last one of the
    // requested type; everything earlier is dropped (latest-wins).
    bool got = false;
    size_t pos = 0;
    while (pos + sizeof(vsim_hdr_t) <= buf_.size()) {
        // Resync to next magic if we don't have one at pos.
        const vsim_hdr_t* hdr = reinterpret_cast<const vsim_hdr_t*>(buf_.data() + pos);
        if (hdr->magic != VSIM_MAGIC) {
            size_t nxt = findMagic(buf_, pos + 1);
            if (nxt == std::string::npos) {
                // Throw away the unrecognized prefix; keep the tail for
                // the next poll in case a magic is being assembled.
                buf_.erase(0, buf_.size());
                pos = 0;
                break;
            }
            buf_.erase(0, nxt);
            pos = 0;
            continue;
        }

        const size_t frame_size = sizeof(vsim_hdr_t) + hdr->payload_bytes;
        if (pos + frame_size > buf_.size()) {
            // Frame not yet complete; wait for more bytes.
            break;
        }
        if (hdr->version == VSIM_PROTO_VERSION &&
            hdr->type    == type &&
            frame_size   == out_size) {
            std::memcpy(out_frame, buf_.data() + pos, out_size);
            got = true;
        }
        pos += frame_size;
    }
    if (pos > 0) buf_.erase(0, pos);

    return got;
}

FifoOut::~FifoOut() {
    if (fd_ >= 0) ::close(fd_);
}

bool FifoOut::open() {
    if (!ensureFifo(path_)) return false;
    // O_RDWR is the trick that lets us open without blocking even if no
    // reader is connected yet. The peer doesn't care which mode we hold.
    fd_ = ::open(path_.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
        std::fprintf(stderr, "vsim: open %s failed: %s\n",
                     path_.c_str(), std::strerror(errno));
        return false;
    }
    return true;
}

bool FifoOut::write(const void* frame, size_t size) {
    if (fd_ < 0) return false;
    ssize_t w = ::write(fd_, frame, size);
    if (w == static_cast<ssize_t>(size)) return true;
    // EAGAIN = no reader / pipe full; drop silently. Anything else is
    // worth a stderr line so it's diagnosable.
    if (w < 0 && errno != EAGAIN) {
        std::fprintf(stderr, "vsim: write %d bytes failed: %s\n",
                     static_cast<int>(size), std::strerror(errno));
    }
    return false;
}

}  // namespace vsim
