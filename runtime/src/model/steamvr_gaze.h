#pragma once

#include "core/metrics.h"
#include "core/threading.h"
#include "model/pipeline.h"
#include "model/postprocess/one_euro.h"

#include <array>
#include <atomic>
#include <optional>
#include <thread>
#include <vector>

namespace babble {

namespace gaze {

struct Offsets {
    double roll_deg = 0, yaw_deg = 0, pitch_deg = 0;
};

// Angle sends angle / max_deg as Steam Link does, and Tangent sends tan(angle) as VRCFT expects.
enum class Scaling { Tangent, Angle };

// Convert a head-local look point in m to the left yaw, right yaw and pitch, as SteamLinkGaze.convert does.
std::optional<std::array<double, 3>> convert(double x, double y, double z, double ipd, double max_deg, bool flip_y,
                                             const Offsets &off, Scaling scaling = Scaling::Angle);

// Convert the head-space gaze direction of one eye to the yaw and pitch of that eye.
std::optional<std::array<double, 2>> from_direction(double x, double y, double z, double max_deg, bool flip_y,
                                                    const Offsets &off, Scaling scaling = Scaling::Angle);

// One direction for both eyes: the mean of the two unit eye directions, so the eyes stay parallel.
std::optional<std::array<double, 2>> parallel(const std::array<float, 3> &l, const std::array<float, 3> &r,
                                              double max_deg, bool flip_y, const Offsets &off,
                                              Scaling scaling = Scaling::Angle);

// The optional One Euro smoothing of the four gaze values, with one preset for each level above 0 (off).
class Smoother {
public:
    explicit Smoother(std::vector<OneEuroParams> presets = {}) : presets_(std::move(presets)) {}
    int levels() const { return int(presets_.size()); }
    // Writes the smoothed raw values to out. A new level or a reset starts the filter again from raw.
    void apply(const float raw[4], float out[4], int level, uint64_t now_ns);
    void reset() { level_ = 0; }

private:
    std::vector<OneEuroParams> presets_;
    OneEuro filter_{4};
    int level_ = 0;   // The level that filter_ was made for, or 0 if it starts again.
    uint64_t last_ns_ = 0;
};

// Optional gaze prediction: estimates how fast each of the four gaze values is moving and sends where it will be a
// little later, which hides part of the eye tracker's lag. Each level above 0 (off) has its own look-ahead time.
class Predictor {
public:
    explicit Predictor(std::vector<double> lookahead_s = {}, double max_delta = 0.25)
        : lookahead_s_(std::move(lookahead_s)), max_delta_(max_delta) {}
    int levels() const { return int(lookahead_s_.size()); }
    // Writes the predicted values to out. A sample equal to the last one keeps the speed estimate unchanged.
    void apply(const float in[4], float out[4], int level, uint64_t now_ns);
    void reset() { have_ = false; }

private:
    std::vector<double> lookahead_s_;
    double max_delta_;
    std::array<double, 4> prev_{}, vel_{};
    uint64_t last_ns_ = 0;
    bool have_ = false;
};

}

// The "steamvr_gaze" pipeline gets eye gaze from the SteamVR eye tracking and sends it for each eye.
class SteamVrGaze : public IPipeline {
public:
    SteamVrGaze(const nlohmann::json &cfg, PipelineDeps deps);
    ~SteamVrGaze() override;

    bool init() override;
    void start() override;
    void stop() override;
    const std::string &feed() const override { return feed_; }
    const std::string &name() const override { return name_; }
    void set_tracking(bool on) override { tracking_ = on; }
    bool set_option(const std::string &key, const nlohmann::json &value) override;

private:
    void run();

    std::string name_;
    nlohmann::json cfg_;
    PipelineDeps deps_;
    ThreadSpec thread_spec_;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::string feed_;
    std::atomic<bool> tracking_{true};   // When this is false, the pipeline centres the gaze one time and stops.
    std::atomic<bool> independent_{true};   // This is true if gaze_source is "independent".
    // When this is true, both eyes get one direction (the gaze from the head centre) instead of each eye aiming at
    // the fixation point. A short or noisy fixation depth then cannot cross the eyes.
    std::atomic<bool> parallel_{false};

    // Each tick function returns true if it published a valid sample.
    bool tick_independent(uint64_t now);
    bool tick_combined(uint64_t now);
    void publish(double lx, double ly, double rx, double ry, uint64_t now, bool centre = false);
    void centre(uint64_t now) { publish(0, 0, 0, 0, now, true); }

    int dominant_eye_ = -1;   // The "Track Dominant Eye Only" eye of the last independent sample, or -1.
    std::optional<double> ipd_override_;
    uint64_t setup_connection_ = 0;
    uint64_t action_set_ = 0, action_ = 0;
    std::array<SignalId, 4> ids_{};   // The order is LeftEyeX, LeftEyeY, RightEyeX, RightEyeY.

    RateStat samples_;
    Counter invalid_, reconnects_, fallback_;
    // Diagnostics for independent gaze: how old each eye-service sample is when it is sent (one clock, mono_ns on
    // the headset), and how many ticks sent a sample that was already sent because no new one had come.
    LatencyStat age_;
    Counter repeats_;
    uint64_t last_seq_ = 0;
    double max_deg_ = 45, stale_ns_ = 5e8;
    bool flip_y_ = false;
    gaze::Scaling scaling_ = gaze::Scaling::Angle;
    gaze::Offsets off_;

    // The smoothing level is 0 (off) or a preset of the smoother, and set_option changes it live.
    gaze::Smoother smoother_;
    std::atomic<int> smoothing_{0};
    std::atomic<bool> smoothing_reset_{false};
    // Gaze prediction level (0 is off), and instant sending of each new eye-service sample in independent mode.
    gaze::Predictor predictor_;
    std::atomic<int> prediction_{0};
    std::atomic<bool> instant_{true};
    bool stats_registered_ = false;
};

}
