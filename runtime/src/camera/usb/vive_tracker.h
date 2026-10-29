#pragma once

// The HTC Vive Facial Tracker as a USB mouth camera. It is a UVC camera (YUY2 400x400 at 60 fps) that sends no
// frames until its sensor is set up and its stream is turned on through a vendor Extension Unit. The sequence
// comes from Project Babble's reverse engineering of SRanipal (docs.babble.diy/blog/reverse-engineering-the-vive-
// facial-tracker) and the Babble App's vivefacialtracker module (MIT, DragonDreams GmbH 2024).
//
// Each 400x400 frame holds the two sensor images side by side (200x400 each). The runtime feeds the left one to
// the mouth model, as the Babble App does.

#include "camera/usb/usb_device.h"
#include "camera/usb/uvc.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace babble::usb::vive {

constexpr uint16_t kVendor = 0x0bb4, kProduct = 0x0321;

// Returns true if this device is a Vive Facial Tracker.
bool is_tracker(const Found &dev);
bool is_tracker(uint16_t vendor, uint16_t product);

// The stream mode that the tracker offers and that the runtime asks for.
inline uvc::Mode default_mode() { return {uvc::FormatKind::Yuy2, 400, 400, 60.0}; }

// Sets up the sensor and turns the stream on. The device must be open with its control interface claimed.
// It returns false with error set when the tracker does not answer as a Vive Facial Tracker.
bool activate(Device &dev, const uvc::Info &info, std::string *error);

// Keeps the left sensor image of a w x h grayscale frame: the left half of each row, in place. It sets *out_w to
// w / 2 and leaves the height alone.
void keep_left_half(uint8_t *gray, int w, int h, int *out_w);

// Makes a Vive frame look the way Baballonia's VFT capture makes it before its model sees it: a 5x5 Gaussian blur
// and the tone curve out = 255 * (in / 2048) ^ (1 / 2.5), which darkens the bright, close-lit IR picture (255 maps
// to 110). In place on a w x h grayscale image; tmp is a work buffer the call resizes.
void babble_look(uint8_t *gray, int w, int h, std::vector<uint8_t> &tmp);

}
