#include "camera/usb/vive_tracker.h"

#include "core/clock.h"
#include "core/log.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>
#include <utility>
#include <vector>

namespace {
const babble::log::Logger LOG("usb");
}

namespace babble::usb::vive {

namespace {

// UVC class-specific requests to the control interface.
constexpr uint8_t kSetCur = 0x01, kGetCur = 0x81, kGetLen = 0x85;
constexpr uint8_t kToInterface = 0x21, kFromInterface = 0xa1;
// The tracker answers a control request in a few milliseconds. A short timeout keeps a stalled tracker from holding
// the camera thread for long, because stop() joins it and the main thread waits.
constexpr unsigned kTimeoutMs = 200;

// The Extension Unit control that SRanipal talks to, and the unit id the tracker has had so far.
constexpr uint8_t kSelector = 2, kUsualUnit = 4;
// The commands in the first byte of a control block.
constexpr uint8_t kTaskSet = 0x50;   // 0x51 reads a register back; the runtime only writes.
constexpr uint8_t kRegSensor = 0xab, kStreamEnable = 0x14;
// The first byte of a GET_CUR answer: the command is still running, or it finished and is echoed back.
constexpr uint8_t kPending = 0x55, kDone = 0x56;

class Unit {
public:
    Unit(Device &dev, uint8_t iface, uint8_t unit) : dev_(dev), iface_(iface), unit_(unit) {}

    // Returns the control's data length from GET_LEN, or 0 if the unit does not answer.
    uint16_t length() {
        uint8_t len[2] = {0, 0};
        const int r = dev_.control(kFromInterface, kGetLen, uint16_t(kSelector << 8), uint16_t(unit_ << 8 | iface_),
                                   len, 2, kTimeoutMs);
        return r == 2 ? uint16_t(len[0] | len[1] << 8) : 0;
    }

    void set_length(uint16_t n) {
        send_.assign(n, 0);
        recv_.assign(n, 0);
    }

    // Sends a command without waiting for the tracker to finish it. Only the first n bytes of the block change;
    // the rest keeps whatever an earlier command left there (the 0x53/0x54 marks of the test block, for example),
    // which is what SRanipal and the Babble App send and what the tracker is known to accept.
    bool set_no_response(const uint8_t *cmd, size_t n, std::string *error) {
        memcpy(send_.data(), cmd, std::min(n, send_.size()));
        if (LOG.debug_enabled()) {
            char hex[3 * 17 + 1] = "";
            for (size_t i = 0; i < std::min<size_t>(n, 17); i++)
                snprintf(hex + 3 * i, 4, "%02x ", cmd[i]);
            LOG.debug("vive: SET_CUR [%s]", hex);
        }
        const int r = dev_.control(kToInterface, kSetCur, uint16_t(kSelector << 8), uint16_t(unit_ << 8 | iface_),
                                   send_.data(), uint16_t(send_.size()), kTimeoutMs);
        if (r < 0) {
            if (error)
                *error = std::string("Vive tracker SET_CUR: ") + strerror(-r);
            return false;
        }
        return true;
    }

    // Sends a command and polls GET_CUR until the tracker echoes it back as finished.
    bool set(const uint8_t *cmd, size_t n, std::string *error, unsigned timeout_ms = 500) {
        if (!set_no_response(cmd, n, error))
            return false;
        const uint64_t deadline = mono_ns() + uint64_t(timeout_ms) * 1000000ull;
        for (;;) {
            std::fill(recv_.begin(), recv_.end(), 0);
            const int r = dev_.control(kFromInterface, kGetCur, uint16_t(kSelector << 8),
                                       uint16_t(unit_ << 8 | iface_), recv_.data(), uint16_t(recv_.size()), kTimeoutMs);
            if (r < 0) {
                if (error)
                    *error = std::string("Vive tracker GET_CUR: ") + strerror(-r);
                return false;
            }
            if (recv_[0] == kDone) {
                // The answer repeats the command after the status byte; the first 16 bytes are checked.
                if (recv_.size() >= 17 && memcmp(recv_.data() + 1, send_.data(), 16) == 0)
                    return true;
                if (error)
                    *error = "Vive tracker answered another command";
                return false;
            }
            if (recv_[0] != kPending) {
                if (error)
                    *error = "Vive tracker gave an unexpected answer (0x" + hex(recv_[0]) + ")";
                return false;
            }
            if (mono_ns() > deadline) {
                if (error)
                    *error = "Vive tracker did not finish a command in time";
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    const std::vector<uint8_t> &received() const { return recv_; }

private:
    static std::string hex(uint8_t v) {
        char buf[4];
        snprintf(buf, sizeof buf, "%02x", v);
        return buf;
    }
    Device &dev_;
    uint8_t iface_, unit_;
    std::vector<uint8_t> send_, recv_;
};

// The handshake block that SRanipal sends before each group of commands.
std::vector<uint8_t> test_block(size_t n) {
    std::vector<uint8_t> b(n, 0);
    b[0] = 0x51;
    b[1] = 0x52;
    if (n >= 256) {
        b[254] = 0x53;
        b[255] = 0x54;
    }
    return b;
}

// A register access: 1-byte address and 1-byte value, both sent big-endian in 4 bytes, on page 0x90010001.
// The Babble App's Python writes "(x > 24) & 0xff" where it means "(x >> 24) & 0xff", so it sends 0x01 in the
// three high bytes of most addresses and values; the tracker accepts that, which shows it reads only the low byte
// when the width fields say 1. This sends the intended zeros. If a tracker ever stays dark, this is the first
// place to compare against a USB capture.
std::array<uint8_t, 17> register_block(uint8_t task, uint8_t reg, uint32_t address, uint32_t value) {
    std::array<uint8_t, 17> b{};
    b[0] = task;
    b[1] = reg;
    b[2] = 0x60;
    b[3] = 1;   // The address width in bytes.
    b[4] = 1;   // The value width in bytes.
    for (int i = 0; i < 4; i++) {
        b[5 + i] = uint8_t(address >> (24 - 8 * i));
        b[13 + i] = uint8_t(value >> (24 - 8 * i));
    }
    b[9] = 0x90;
    b[10] = 0x01;
    b[11] = 0x00;
    b[12] = 0x01;
    return b;
}

bool set_sensor_register(Unit &xu, uint8_t address, uint8_t value, std::string *error) {
    const auto b = register_block(kTaskSet, kRegSensor, address, value);
    if (!xu.set(b.data(), b.size(), error))
        return false;
    return true;
}

bool set_stream(Unit &xu, bool on, std::string *error) {
    const uint8_t b[4] = {kTaskSet, kStreamEnable, 0x00, uint8_t(on ? 0x01 : 0x00)};
    return xu.set_no_response(b, sizeof b, error);
}

}

bool is_tracker(uint16_t vendor, uint16_t product) { return vendor == kVendor && product == kProduct; }
bool is_tracker(const Found &dev) { return is_tracker(dev.vendor, dev.product); }

bool activate(Device &dev, const uvc::Info &info, std::string *error) {
    if (info.control_iface == 0xff) {
        if (error)
            *error = "Vive tracker has no video control interface";
        return false;
    }
    // Find the Extension Unit that answers GET_LEN with the 384-byte (or 64-byte) buffer of the tracker. The
    // descriptor lists the units; the usual id goes first, and 4 is tried when the descriptor lists none.
    std::vector<uint8_t> units;
    for (uint8_t u : info.extension_units)
        if (u == kUsualUnit)
            units.push_back(u);
    for (uint8_t u : info.extension_units)
        if (u != kUsualUnit)
            units.push_back(u);
    if (units.empty())
        units.push_back(kUsualUnit);

    uint8_t unit = 0;
    uint16_t len = 0;
    for (uint8_t u : units) {
        Unit probe(dev, info.control_iface, u);
        const uint16_t n = probe.length();
        LOG.debug("vive: extension unit %u answers GET_LEN %u", u, n);
        if (n == 384 || n == 64) {
            unit = u;
            len = n;
            break;
        }
    }
    if (!unit) {
        if (error)
            *error = "no Extension Unit answered like a Vive Facial Tracker (GET_LEN 384 or 64)";
        return false;
    }
    LOG.info("vive: Vive Facial Tracker on control interface %u, extension unit %u, %u-byte blocks",
             info.control_iface, unit, len);

    Unit xu(dev, info.control_iface, unit);
    xu.set_length(len);
    const auto test = test_block(len);

    // Stop a stream that an earlier run left on, set the sensor up, then turn the stream on. The waits are the ones
    // SRanipal makes.
    if (!xu.set(test.data(), test.size(), error) || !set_stream(xu, false, error))
        return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    if (!xu.set(test.data(), test.size(), error))
        return false;
    // Sensor setup: mode, then exposure (0x02-0x04) all the way up and gain (0x05-0x07) at 0xb2, as SRanipal sets them
    // for the IR-lit mouth.
    const std::pair<uint8_t, uint8_t> regs[] = {
        {0x00, 0x40}, {0x08, 0x01}, {0x70, 0x00}, {0x02, 0xff}, {0x03, 0xff}, {0x04, 0xff},
        {0x0e, 0x00}, {0x05, 0xb2}, {0x06, 0xb2}, {0x07, 0xb2}, {0x0f, 0x03},
    };
    for (const auto &[address, value] : regs)
        if (!set_sensor_register(xu, address, value, error)) {
            if (error) {
                char which[40];
                snprintf(which, sizeof which, " (sensor register 0x%02x = 0x%02x)", address, value);
                *error += which;
            }
            return false;
        }

    if (!xu.set(test.data(), test.size(), error) || !set_stream(xu, true, error))
        return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    LOG.info("vive: sensor set up and stream turned on");
    return true;
}

namespace {

// The tone curve of Baballonia's VFTCommon: 255 * (i / 2048) ^ (1 / 2.5), as a lookup table.
const std::array<uint8_t, 256> &tone_lut() {
    static const std::array<uint8_t, 256> lut = [] {
        std::array<uint8_t, 256> t{};
        for (int i = 0; i < 256; i++)
            t[size_t(i)] = uint8_t(std::pow(i / 2048.0, 1.0 / 2.5) * 255.0);
        return t;
    }();
    return lut;
}

}

void babble_look(uint8_t *gray, int w, int h, std::vector<uint8_t> &tmp) {
    if (w < 5 || h < 5)
        return;
    // OpenCV's GaussianBlur with a 5x5 kernel and sigma 0 uses the fixed kernel [1 4 6 4 1] / 16, separably, with
    // reflected borders.
    static constexpr int k[5] = {1, 4, 6, 4, 1};
    auto reflect = [](int i, int n) { return i < 0 ? -i : i >= n ? 2 * n - i - 2 : i; };
    tmp.resize(size_t(w) * h);
    // Horizontal pass into tmp.
    for (int y = 0; y < h; y++) {
        const uint8_t *row = gray + size_t(y) * w;
        uint8_t *out = tmp.data() + size_t(y) * w;
        for (int x = 0; x < w; x++) {
            int sum = 0;
            for (int t = -2; t <= 2; t++)
                sum += k[t + 2] * row[reflect(x + t, w)];
            out[x] = uint8_t((sum + 8) >> 4);
        }
    }
    // Vertical pass back into gray, with the tone curve applied to the result.
    const auto &lut = tone_lut();
    for (int y = 0; y < h; y++) {
        uint8_t *out = gray + size_t(y) * w;
        for (int x = 0; x < w; x++) {
            int sum = 0;
            for (int t = -2; t <= 2; t++)
                sum += k[t + 2] * tmp[size_t(reflect(y + t, h)) * w + x];
            out[x] = lut[size_t((sum + 8) >> 4)];
        }
    }
}

void keep_left_half(uint8_t *gray, int w, int h, int *out_w) {
    const int half = w / 2;
    for (int y = 1; y < h; y++)   // Row 0 is already in place.
        memmove(gray + size_t(y) * half, gray + size_t(y) * w, size_t(half));
    *out_w = half;
}

}
