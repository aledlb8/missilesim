#pragma once

namespace missilesim::sim
{
    // Turns variable rendered-frame time into a whole number of fixed
    // simulation steps ("Fix Your Timestep", Fiedler).
    //
    // Backlog policy, which is what keeps time scaling honest:
    //  - A single frame contributes at most kMaxFrameSeconds of wall time; a
    //    longer hitch (debugger, window drag) is not replayed in one burst.
    //  - Up to kMaxStepsPerFrame steps run per frame, enough for the 10x time
    //    scale at 15 frames per second with the default 0.01 s step.
    //  - Simulation time still owed beyond kMaxBacklogSeconds after stepping
    //    is dropped and counted (droppedSeconds): the simulation then runs
    //    slower than requested instead of spiralling into ever longer frames.
    // Because every step is the same size, any frame pacing that is not
    // backlog-limited produces the same sequence of steps and therefore the
    // same simulation.
    class FixedStepClock
    {
    public:
        static constexpr float kMaxFrameSeconds = 0.1f;
        static constexpr int kMaxStepsPerFrame = 64;
        static constexpr float kMaxBacklogSeconds = 0.25f;

        explicit FixedStepClock(float stepSeconds = 0.01f);

        void setStep(float stepSeconds);
        float step() const { return m_step; }

        // Adds one frame of wall time at the given time scale and returns the
        // number of fixed steps to run now.
        int advance(float frameSeconds, float timeScale);

        // Leftover fraction of a step (0..1), for render interpolation.
        float alpha() const;

        void reset();
        double droppedSeconds() const { return m_droppedSeconds; }

    private:
        float m_step = 0.01f;
        double m_accumulator = 0.0;
        double m_droppedSeconds = 0.0;
    };
}
