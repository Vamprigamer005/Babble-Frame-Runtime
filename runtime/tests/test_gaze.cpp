#include <doctest/doctest.h>

#include "model/steamvr_gaze.h"

#include <algorithm>
#include <array>
#include <cmath>

using namespace babble;

TEST_CASE("angle scaling: eye_osc_pc.py's per-eye yaw, gaze_angles() pitch") {
    struct Case {
        double x, y, z, roll, yaw, pitch;
        bool flip_y, valid;
        double lx, rx, py;
    };
    // The yaw values come from the reference SteamLinkGaze.convert with ipd 0.063 and 45 deg.
    const Case cases[] = {
        {0.0, 0.0, -2.0, 0, 0, 0, false, true, 0.02005186490066484, -0.02005186490066484, 0.0},
        {0.5, 0.3, -1.0, 0, 0, 0, false, true, 0.6220143195789958, 0.5578464486092127, 0.3710943163109694},
        {-0.8, -0.4, -0.6, 0, 0, 0, false, true, -1.0, -1.0, -0.7486681672439954},
        {0.1, 0.0, -0.3, 0, 0, 0, false, true, 0.5259875822390145, 0.2858230303810862, 0.0},
        {3.0, -1.0, 2.0, 0, 0, 0, false, true, 1.0, 1.0, -0.590334470601733},
        {0.4, 0.2, -1.5, 5, -3, 10, true, true, 0.2723296429298719, 0.22204079170398106, 0.02738765650615556},
        {0.001, 0.0, 0.002, 0, 0, 0, false, false, 0, 0, 0},
    };
    for (const auto &c : cases) {
        CAPTURE(c.x);
        CAPTURE(c.y);
        CAPTURE(c.z);
        auto g = gaze::convert(c.x, c.y, c.z, 0.063, 45.0, c.flip_y, {c.roll, c.yaw, c.pitch}, gaze::Scaling::Angle);
        REQUIRE(bool(g) == c.valid);
        if (!g)
            continue;
        CHECK((*g)[0] == doctest::Approx(c.lx).epsilon(1e-12));
        CHECK((*g)[1] == doctest::Approx(c.rx).epsilon(1e-12));
        CHECK((*g)[2] == doctest::Approx(c.py).epsilon(1e-12));
    }
}

TEST_CASE("tangent scaling gives VRCFT's Gaze (tan of the angle; atan() recovers it)") {
    const double h = 0.063 / 2;
    // For points without a clamp, each value is the tangent of the angle from that eye to the point.
    struct P { double x, y, z; } pts[] = {{0.0, 0.0, -2.0}, {0.5, 0.3, -1.0}, {-0.3, -0.2, -0.8}, {0.1, 0.0, -0.3}};
    const auto tangent = gaze::Scaling::Tangent;
    for (const auto &p : pts) {
        auto g = gaze::convert(p.x, p.y, p.z, 0.063, 45.0, false, {}, tangent);
        REQUIRE(g);
        const double z = std::fabs(p.z);
        CHECK((*g)[0] == doctest::Approx((p.x + h) / z).epsilon(1e-12));
        CHECK((*g)[1] == doctest::Approx((p.x - h) / z).epsilon(1e-12));
        CHECK((*g)[2] == doctest::Approx(p.y / z).epsilon(1e-12));
    }
    // Right and up are positive, and the VRCFT ToYaw = atan(x) gets exactly 30 degrees back.
    const double t30 = std::tan(30.0 * M_PI / 180);
    auto g = gaze::convert(t30 * 10, 0, -10, 0.0, 45.0, false, {}, tangent);
    REQUIRE(g);
    CHECK(std::atan((*g)[0]) * 180 / M_PI == doctest::Approx(30.0).epsilon(1e-9));
    auto up = gaze::convert(0, 1, -2, 0.063, 45.0, false, {}, tangent);
    REQUIRE(up);
    CHECK((*up)[2] > 0);
    // The angle has a limit at max_deg, and tan(45 deg) = 1.
    auto far = gaze::convert(10, 0, -1, 0.0, 45.0, false, {}, tangent);
    REQUIRE(far);
    CHECK((*far)[0] == doctest::Approx(1.0));
}

TEST_CASE("per-eye gaze directions convert like frameeyeosc's gaze_angles()") {
    const gaze::Offsets none;
    // Straight ahead (-z forward) is the centre.
    auto c = gaze::from_direction(0, 0, -1, 45, false, none);
    REQUIRE(c);
    CHECK((*c)[0] == doctest::Approx(0));
    CHECK((*c)[1] == doctest::Approx(0));
    // The gaze_angles() function multiplies atan2(x, -z) and atan2(y, -z) by 4/pi, so 45 deg is 1.
    auto gaze_angles = [](double x, double y, double z) {
        const double k = 4 / M_PI;
        return std::array<double, 2>{std::clamp(std::atan2(x, -z) * k, -1.0, 1.0),
                                     std::clamp(std::atan2(y, -z) * k, -1.0, 1.0)};
    };
    for (const auto &v : {std::array<double, 3>{0.5, 0.17, -0.85}, {-0.2, -0.3, -0.9}, {0.9, 0.6, -0.3},
                          {-0.05, 0.4, -0.7}}) {
        CAPTURE(v[0]);
        auto d = gaze::from_direction(v[0], v[1], v[2], 45, false, none);
        REQUIRE(d);
        const auto want = gaze_angles(v[0], v[1], v[2]);
        CHECK((*d)[0] == doctest::Approx(want[0]).epsilon(1e-12));
        CHECK((*d)[1] == doctest::Approx(want[1]).epsilon(1e-12));
    }
    // Pitch is only the vertical angle, so 10 deg up stays 10 deg for all yaw values.
    const double up10 = std::tan(10 * M_PI / 180);
    auto side = gaze::from_direction(0.8, up10, -1, 45, false, none);
    CHECK((*side)[1] == doctest::Approx(10.0 / 45));
    // The tangent scaling gives the tangents of those angles, x/z and y/z.
    auto t = gaze::from_direction(0.5, 0.2, -1, 45, false, none, gaze::Scaling::Tangent);
    CHECK((*t)[0] == doctest::Approx(0.5));
    CHECK((*t)[1] == doctest::Approx(0.2));
    // Past the limit, the value is clamped to +-45 deg, which is +-1 for both scalings.
    auto far = gaze::from_direction(-1, 0, -0.1, 45, false, none);
    REQUIRE(far);
    CHECK((*far)[0] == doctest::Approx(-1.0));
    auto far_tan = gaze::from_direction(-1, 0, -0.1, 45, false, none, gaze::Scaling::Tangent);
    CHECK((*far_tan)[0] == doctest::Approx(-1.0));
    // The flip_y option and a yaw offset apply as in convert().
    auto f = gaze::from_direction(0, 0.2, -1, 45, true, none);
    CHECK((*f)[1] < 0);
    gaze::Offsets yaw10;
    yaw10.yaw_deg = 10;
    auto o = gaze::from_direction(0, 0, -1, 45, false, yaw10);
    CHECK((*o)[0] == doctest::Approx(10.0 / 45));
    CHECK_FALSE(gaze::from_direction(0, 0, 0, 45, false, none));
}

TEST_CASE("gaze smoothing is off at level 0, steadies a jump at a level, and starts again on a reset") {
    gaze::Smoother sm({{1.5, 0.3, 1.0}, {0.8, 0.1, 1.0}});
    CHECK(sm.levels() == 2);
    const float rest[4] = {0, 0, 0, 0}, jump[4] = {0.5f, -0.5f, 0.5f, -0.5f};
    float out[4];
    sm.apply(jump, out, 0, 0);   // Off passes the values through.
    CHECK(out[0] == 0.5f);

    const uint64_t step = 11'111'111;   // 90 Hz.
    sm.apply(rest, out, 1, step);       // The first sample of a level does not change.
    CHECK(out[0] == 0.0f);
    sm.apply(jump, out, 1, 2 * step);
    CHECK(out[0] > 0.0f);
    CHECK(out[0] < 0.5f);
    CHECK(out[1] < 0.0f);
    CHECK(out[1] > -0.5f);
    const float low = out[0];

    sm.apply(rest, out, 2, 3 * step);   // A new level starts again from the sample.
    CHECK(out[0] == 0.0f);
    sm.apply(jump, out, 2, 4 * step);
    CHECK(out[0] < low);                // The higher level smooths more.

    sm.reset();
    sm.apply(jump, out, 2, 5 * step);
    CHECK(out[0] == 0.5f);
    sm.apply(rest, out, 3, 6 * step);   // A level without a preset is off.
    CHECK(out[0] == 0.0f);
}


TEST_CASE("parallel convergence: both eyes share one direction, never crossed") {
    // Eyes that disagree (one 10 deg left, one 10 deg right of straight ahead) average to straight ahead.
    const double a = 10.0 * M_PI / 180;
    std::array<float, 3> l{float(std::sin(a)), 0.f, float(-std::cos(a))}, r{float(-std::sin(a)), 0.f, float(-std::cos(a))};
    auto g = gaze::parallel(l, r, 45.0, false, {});
    REQUIRE(g);
    CHECK((*g)[0] == doctest::Approx(0.0).epsilon(1e-6));
    CHECK((*g)[1] == doctest::Approx(0.0).epsilon(1e-6));
    // Vector length does not weight an eye: a long left vector gives the same answer.
    std::array<float, 3> l5{l[0] * 5, l[1] * 5, l[2] * 5};
    auto g5 = gaze::parallel(l5, r, 45.0, false, {});
    REQUIRE(g5);
    CHECK((*g5)[0] == doctest::Approx((*g)[0]).epsilon(1e-6));
    // Agreeing eyes 20 deg right give 20/45 in angle scaling, right positive.
    const double b = 20.0 * M_PI / 180;
    std::array<float, 3> e{float(std::sin(b)), 0.f, float(-std::cos(b))};
    auto gr = gaze::parallel(e, e, 45.0, false, {});
    REQUIRE(gr);
    CHECK((*gr)[0] == doctest::Approx(20.0 / 45.0).epsilon(1e-6));
    // A zero vector is invalid.
    CHECK_FALSE(gaze::parallel({0, 0, 0}, r, 45.0, false, {}));
}

TEST_CASE("parallel from a near fixation point does not converge, unlike convert()") {
    // A fixation point 0.3 m ahead and slightly right: convert() turns the eyes far toward each other.
    auto conv = gaze::convert(0.1, 0.0, -0.3, 0.063, 45.0, false, {});
    REQUIRE(conv);
    CHECK((*conv)[0] - (*conv)[1] > 0.2);   // About 11 deg of crossing.
    auto par = gaze::from_direction(0.1, 0.0, -0.3, 45.0, false, {});
    REQUIRE(par);
    CHECK((*par)[0] < (*conv)[0]);
    CHECK((*par)[0] > (*conv)[1]);   // The shared direction lies between the two crossed eyes.
}

TEST_CASE("prediction: off passes values through, on leads a steady movement") {
    gaze::Predictor p({0.010, 0.020, 0.035}, 0.25);
    REQUIRE(p.levels() == 3);
    float out[4];
    // A gaze moving at 1.0 per second on every channel, sampled every 10 ms.
    uint64_t t = 1'000'000'000ull;
    for (int k = 0; k < 10; k++, t += 10'000'000ull) {
        const float v = float(0.01 * k);
        const float in[4] = {v, v, v, v};
        p.apply(in, out, 0, t);
        for (float o : out)
            CHECK(o == doctest::Approx(v));   // Level 0 is off.
    }
    // Level 2 (20 ms) leads by about 0.02 once the speed estimate settles.
    const float in[4] = {0.10f, 0.10f, 0.10f, 0.10f};
    p.apply(in, out, 2, t);
    CHECK(out[0] == doctest::Approx(0.10 + 0.02).epsilon(0.05));
}

TEST_CASE("prediction: a repeated sample keeps the speed, a still gaze is not moved, a jump is capped") {
    gaze::Predictor p({0.020}, 0.25);
    float out[4];
    uint64_t t = 1'000'000'000ull;
    const float a[4] = {0, 0, 0, 0};
    p.apply(a, out, 1, t);
    for (float o : out)
        CHECK(o == 0.0f);   // No speed on the first sample.
    // Still gaze: no movement means no prediction.
    for (int k = 0; k < 5; k++)
        p.apply(a, out, 1, t += 10'000'000ull);
    CHECK(out[0] == doctest::Approx(0.0));
    // A huge jump in 10 ms is capped at max_delta.
    const float b[4] = {0.9f, 0.9f, 0.9f, 0.9f};
    p.apply(b, out, 1, t += 10'000'000ull);
    CHECK(out[0] <= 0.9f + 0.25f + 1e-6f);
    // After a reset there is no speed again.
    p.reset();
    p.apply(b, out, 1, t += 10'000'000ull);
    CHECK(out[0] == doctest::Approx(0.9));
}
