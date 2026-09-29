#pragma once

#include "F16Airframe.h"

namespace missilesim::flight
{
    // What the pilot (or the mouse-aim instructor) asks the fly-by-wire for.
    struct PilotCommand
    {
        float pitchRate = 0.0f; // body pitch rate, rad/s, bounded by the AoA and g limiters
        float rollRate = 0.0f;  // stability-axis roll rate, rad/s
        float sideslip = 0.0f;  // rad
    };

    struct FlightControlStatus
    {
        float pitchRateCommand = 0.0f; // after the limiters, rad/s
        float rollRateLimit = 0.0f;    // rad/s
        bool alphaLimited = false;     // the AoA limiter is holding the pull
        bool loadLimited = false;      // the g limiter is holding the pull or push
        bool spinPrevention = false;
    };

    // F-16 fly-by-wire limits as NASA TP-1538 appendix A describes them: an
    // angle-of-attack limiter (~25 deg in 1 g) and +9 g, roll-rate command up
    // to 308 deg/s scheduled down with dynamic pressure and AoA (the report's
    // control system B), pilot rudder faded out between 20 and 30 deg AoA, and
    // a spin-prevention mode above 29 deg.
    //
    // The pitch axis takes a pitch-rate command, which is what a point-the-nose
    // controller needs; the AoA and g limits bound that rate through the AoA
    // kinematics. (The real F-16 is g-command at speed and blends toward
    // pitch-rate command at low speed. Driving g from nose error was measured
    // to oscillate: the nose moves mostly through AoA, several degrees per g
    // at modest dynamic pressure, so that loop gain is far above one.)
    //
    // The report's gains and block diagrams are not reproduced; the laws are
    // realised by nonlinear dynamic inversion of the airframe's own tables,
    // which gives uniform handling across the envelope without a gain schedule.
    class FlightControlSystem
    {
    public:
        static constexpr float kMaxNormalLoad = 9.0f;   // +9 g, published F-16C limit
        static constexpr float kMinNormalLoad = -3.0f;  // negative g limit
        static constexpr float kAlphaLimitDeg = 25.0f;  // TP-1538: ~25 deg in 1 g flight
        static constexpr float kMaxRollRateDeg = 308.0f;
        static constexpr float kMaxSideslipDeg = 6.0f;

        Surfaces update(const F16Airframe &airframe, const PilotCommand &command, float gravity);

        const FlightControlStatus &status() const { return m_status; }

        // TP-1538 control system B: 308 deg/s, less 0.0115 deg/s per Pa below
        // 10,500 Pa and 4 deg/s per degree of AoA above 15 deg, floor 80 deg/s.
        static float rollRateLimit(float dynamicPressure, float alpha);

    private:
        FlightControlStatus m_status;
    };
}
