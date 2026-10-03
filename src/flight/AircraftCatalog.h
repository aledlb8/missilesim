#pragma once

// Which flight card the player is flying. Exterior assets use the same id
// under models/fighters/<id>.obj; the renderer resolves them per object.
// The F-16 card is the NASA TP-1538 model in src/flight. Every other card is a
// point mass in CardAirframe, fed only by the numbers below. A faint UI row is
// not a published figure. Stand-ins live in the integrator, not in these cells.
// Brochure weights and thrust are the named rows in modeling_notes/flight.
// A cell the brochures leave empty may be filled from the same War Thunder
// vehicle. The card text says War Thunder. A different variant is not used.

namespace missilesim::flight
{
    enum class CardStatus
    {
        FullTable,
        BrochureOnly,
        Insufficient
    };

    // Substitutes CardAirframe may apply when a cell is empty. They are not a
    // drag polar, not an Oswald factor, and not the F-16 tables.
    constexpr float kStandInMassKg = 19000.0f;
    constexpr float kStandInSeaLevelMps = 460.0f;
    constexpr float kStandInPositiveG = 9.0f;
    constexpr float kStandInNegativeG = -3.0f;
    constexpr float kStandInCd0 = 0.020f;
    constexpr float kStandInThrustToWeight = 1.0f;

    struct AircraftCard
    {
        const char *id = "";
        const char *displayName = "";
        const char *family = "";
        CardStatus status = CardStatus::Insufficient;
        bool flyable = false;
        // True only for the NASA TP-1538 airframe. Point-mass cards leave this false.
        bool tableModel = false;

        bool massPublished = false;
        float massKg = 0.0f;
        const char *massText = "Not published";

        // The mass the point-mass model flies, built the way the F-16's
        // combat mass is: empty + half the internal fuel + two short-range
        // missiles + the pilot. A published brochure mass (above) is often a
        // maximum or an empty weight and is not flown. 0 falls back to massKg.
        float flyingMassKg = 0.0f;
        const char *flyingMassText = "";

        bool militaryPublished = false;
        float militaryThrustN = 0.0f;
        const char *militaryText = "Not published";

        bool maxThrustPublished = false;
        float maxThrustN = 0.0f;
        const char *maxText = "Not published";

        bool areaPublished = false;
        float wingAreaM2 = 0.0f;
        const char *areaText = "Not published";

        bool positiveGPublished = false;
        float positiveG = 0.0f;
        bool negativeGPublished = false;
        float negativeG = 0.0f;

        // speedEquality means seaLevelSpeedMps is a speed the airplane is
        // stated to reach, so drag may be matched to it. An inequality is
        // published text and is not that speed.
        bool speedPublished = false;
        bool speedEquality = false;
        float seaLevelSpeedMps = 0.0f;
        const char *speedText = "Not published";

        const char *note = "";
    };

    const AircraftCard *aircraftCatalog();
    int aircraftCatalogCount();
    const AircraftCard *findAircraft(const char *id);
    const char *defaultAircraftId();
    const char *cardStatusLabel(CardStatus status);
}
