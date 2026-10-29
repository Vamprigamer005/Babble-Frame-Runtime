// Tests for the Vive Facial Tracker support that need no hardware: the descriptor parsing and the frame split.
#include <doctest/doctest.h>

#include "camera/usb/uvc.h"
#include "camera/usb/vive_tracker.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

using namespace babble;

TEST_CASE("vive: the device id is HTC 0bb4:0321") {
    CHECK(usb::vive::is_tracker(0x0bb4, 0x0321));
    CHECK_FALSE(usb::vive::is_tracker(0x303a, 0x8000));
    usb::Found f;
    f.vendor = 0x0bb4;
    f.product = 0x0321;
    CHECK(usb::vive::is_tracker(f));
    CHECK(usb::vive::default_mode().format == uvc::FormatKind::Yuy2);
    CHECK(usb::vive::default_mode().width == 400);
    CHECK(usb::vive::default_mode().height == 400);
}

TEST_CASE("vive: keep_left_half keeps the left sensor image of each row") {
    const int w = 8, h = 3;
    std::vector<uint8_t> px(w * h);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            px[y * w + x] = uint8_t(y * 16 + x);
    int out_w = 0;
    usb::vive::keep_left_half(px.data(), w, h, &out_w);
    CHECK(out_w == 4);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < 4; x++)
            CHECK(px[y * 4 + x] == uint8_t(y * 16 + x));
}

TEST_CASE("uvc: parse_config lists the Extension Units of the control interface") {
    // A minimal configuration: a VC interface with a header and two extension units (ids 3 and 4), a VS interface
    // with a YUY2 format and one 400x400 frame, and a bulk IN endpoint.
    std::vector<uint8_t> d;
    auto put = [&](std::initializer_list<uint8_t> b) { d.insert(d.end(), b); };
    // Interface 0: video control.
    put({9, 0x04, 0, 0, 1, 0x0e, 0x01, 0x00, 0});
    put({13, 0x24, 0x01, 0x10, 0x01, 13, 0, 0, 0, 0, 0, 1, 1});   // VC header, UVC 1.10, one VS interface.
    // Extension units: bLength, 0x24, 0x06, bUnitID, GUID(16), bNumControls, bNrInPins, baSourceID, bControlSize, bmControls, iExtension.
    for (uint8_t id : {uint8_t(3), uint8_t(4)}) {
        put({26, 0x24, 0x06, id});
        for (int i = 0; i < 16; i++)
            d.push_back(uint8_t(i + id));
        put({2, 1, 1, 1, 0xff, 0});
    }
    // Interface 1: video streaming with a YUY2 format and a frame.
    put({9, 0x04, 1, 0, 1, 0x0e, 0x02, 0x00, 0});
    put({27, 0x24, 0x04, 1, 1, 'Y', 'U', 'Y', '2', 0, 0, 0x10, 0, 0x80, 0, 0, 0xaa, 0, 0x38, 0x9b, 0x71, 16, 1, 0, 0, 0, 0});
    put({30, 0x24, 0x05, 1, 0, 0x90, 0x01, 0x90, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xe2, 0x04, 0, 0x2a, 0x8b, 0x02, 0, 1, 0x2a, 0x8b, 0x02, 0});
    put({7, 0x05, 0x81, 0x02, 0x00, 0x02, 0});
    std::string err;
    auto info = uvc::parse_config(d.data(), d.size(), &err);
    REQUIRE_MESSAGE(info, err);
    REQUIRE(info->extension_units.size() == 2);
    CHECK(info->extension_units[0] == 3);
    CHECK(info->extension_units[1] == 4);
    CHECK(info->control_iface == 0);
    CHECK(info->stream_iface == 1);
    CHECK(info->bulk);
    REQUIRE(info->formats.size() == 1);
    CHECK(info->formats[0].kind == uvc::FormatKind::Yuy2);
    REQUIRE(info->formats[0].frames.size() == 1);
    CHECK(info->formats[0].frames[0].width == 400);
    CHECK(info->formats[0].frames[0].height == 400);
}

TEST_CASE("vive: babble_look applies Baballonia's tone curve and keeps a flat image flat") {
    const int w = 8, h = 6;
    std::vector<uint8_t> px(size_t(w) * h, 255), tmp;
    usb::vive::babble_look(px.data(), w, h, tmp);
    for (uint8_t v : px)
        CHECK(v == 110);   // 255 * (255 / 2048) ^ 0.4 = 110.8, truncated as Baballonia's byte cast does
    std::fill(px.begin(), px.end(), 0);
    usb::vive::babble_look(px.data(), w, h, tmp);
    for (uint8_t v : px)
        CHECK(v == 0);
    std::fill(px.begin(), px.end(), 64);
    usb::vive::babble_look(px.data(), w, h, tmp);
    for (uint8_t v : px)
        CHECK(v == uint8_t(std::pow(64 / 2048.0, 0.4) * 255.0));
}
