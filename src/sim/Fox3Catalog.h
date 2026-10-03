#pragma once

// Named Fox 3 cards from the opened study. A card stores printed mass, diameter,
// fin type, and guidance or motor class. Thrust, burn, impulse, seeker band,
// gimbal, datalink rate, and lethal radius stay empty when a page did not print
// them. Brochure kilometres are claims. They are not fly-out numbers.
//
// flyout() is a separate, labeled record. "reference" is the declared archetype
// the harness already flies. A refused card does not borrow a neighbour's motor.

#include <cstdint>

namespace missilesim::fox3
{
    enum class Motor : std::uint8_t
    {
        ReferenceSolid,
        BoostSustain,
        SingleSolid,
        DualPulse,
        DualMode,
        DuctedRocket,
        Unpublished
    };

    enum class Support : std::uint8_t
    {
        DatalinkThenActive,
        SemiActiveThenActive,
        Unresolved,
        NotOpened
    };

    enum class Link : std::uint8_t
    {
        NotStated,
        Uplink,
        TwoWay,
        TwoWayClaim
    };

    enum class Shell : std::uint8_t
    {
        ReferenceExact,
        ReferenceShell,
        SpecificThrust,
        Refused
    };

    enum class LaunchRefusal : std::uint8_t
    {
        None = 0,
        SemiActiveMidcourse,
        DuctedRocket,
        SeekerUnresolved,
        SecondarySource,
        NoPerformanceCard
    };

    struct Spec
    {
        const char *id = "";
        const char *displayName = "";
        const char *family = "";

        float massKg = 0.0f;
        bool massPublished = false;
        const char *massNote = "";

        float lengthM = 0.0f;
        bool lengthPublished = false;
        const char *lengthNote = "";

        float diameterM = 0.0f;
        bool diameterPublished = false;
        const char *diameterNote = "";

        float spanM = 0.0f;
        bool spanPublished = false;

        const char *finNote = "";
        Motor motor = Motor::Unpublished;
        Support support = Support::DatalinkThenActive;
        Link link = Link::NotStated;
        bool homeOnJam = false;

        // Brochure or export sentence. Never read as a range the integrator flies.
        const char *rangeClaim = "";
        // A capture range only where the page also printed the radar cross section.
        const char *captureNote = "";
        const char *card = "";

        Shell shell = Shell::Refused;
        LaunchRefusal refusal = LaunchRefusal::None;
        const char *flyoutNote = "";
    };

    // Declared archetype. Same numbers the radar round used before this catalog.
    struct Flyout
    {
        float massKg = 90.0f;
        float dragCoefficient = 0.35f;
        float areaM2 = 0.02f;
        float thrustN = 14000.0f;
        float fuelKg = 24.0f;
        float fuelPerS = 4.0f;
    };

    const Spec *catalog();
    int catalogCount();
    const Spec *find(const char *id);

    Flyout referenceFlyout();
    // Refused cards and cards with no printed shell return the archetype.
    // Callers that launch must still honour LaunchRefusal and must not treat
    // that fallback as the named round's motor.
    Flyout flyout(const Spec &spec);

    const char *motorLabel(Motor motor);
    const char *supportLabel(Support support);
    const char *linkLabel(Link link);
    const char *shellLabel(Shell shell);
    const char *refusalLabel(LaunchRefusal refusal);

    int runFox3CatalogChecks();
}
