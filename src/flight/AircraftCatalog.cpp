#include "AircraftCatalog.h"

#include <cstring>

namespace missilesim::flight
{
namespace
{
    constexpr float kPound = 0.45359237f;
    constexpr float kGravity = 9.80665f;
    constexpr float kLbf = kPound * kGravity;
    constexpr float kKnot = 1852.0f / 3600.0f;
    constexpr float kFoot2 = 0.09290304f;

    AircraftCard f16()
    {
        AircraftCard card{};
        card.id = "f-16c-block-50";
        card.displayName = "F-16C Block 50";
        card.family = "United States";
        card.status = CardStatus::FullTable;
        card.flyable = true;
        card.tableModel = true;
        card.massPublished = true;
        card.massKg = 22972.0f * kPound;
        card.massText = "22,972 lb combat";
        card.militaryPublished = true;
        card.militaryText = "14,807.7 lbf sea-level static";
        card.maxThrustPublished = true;
        card.maxText = "24,655.2 lbf sea-level static";
        card.areaPublished = true;
        card.wingAreaM2 = 27.87f;
        card.areaText = "27.87 m2";
        card.positiveGPublished = true;
        card.positiveG = 9.0f;
        card.negativeGPublished = true;
        card.negativeG = -3.0f;
        card.speedPublished = true;
        card.speedText = "1,555 km/h IAS limit, War Thunder";
        card.note = "NASA TP-1538 wing and moments, Block 50 combat mass, F110 thrust scale on the F100 deck. "
                    "The static thrust line is the deck at sea level, not a constant. Fuel is not burned. "
                    "War Thunder's F-16CM card limits indicated airspeed to 1,555 km/h. The flying load stays +9 / -3.";
        return card;
    }

    AircraftCard f15()
    {
        AircraftCard card{};
        card.id = "f-15c";
        card.displayName = "F-15C";
        card.family = "United States";
        card.status = CardStatus::BrochureOnly;
        card.flyable = true;
        card.massPublished = true;
        card.massKg = 31700.0f * kPound;
        card.massText = "31,700 lb";
        card.militaryPublished = true;
        card.militaryThrustN = 2.0f * 6070.0f * kGravity;
        card.militaryText = "6,070 kgf each, War Thunder";
        card.maxThrustN = 2.0f * 23770.0f * kLbf;
        card.maxThrustPublished = true;
        card.maxText = "23,770 lbf each, with afterburner";
        card.areaPublished = true;
        card.wingAreaM2 = 19690.0f / 349.0f;
        card.areaText = "56.42 m2, War Thunder";
        card.positiveGPublished = true;
        card.positiveG = 12.0f;
        card.negativeGPublished = true;
        card.negativeG = -4.0f;
        card.speedPublished = true;
        card.speedEquality = true;
        card.seaLevelSpeedMps = 1629.0f / 3.6f;
        card.speedText = "1,629 km/h IAS limit, War Thunder";
        card.note = "USAF fact-sheet weight 31,700 lb. Museum F100-PW-220, 23,770 lb with afterburner, each engine. "
                    "War Thunder F-15C MSIP II, same F100-PW-220: stationary military 6,070 kgf each, wing 56.42 m2 from 349 kg/m2 at full internal fuel, load about -4 / +12, indicated-airspeed limit 1,629 km/h. "
                    "The stat card's 2,506 km/h is at 10,668 m and is not the sea-level drag speed.";
        return card;
    }

    AircraftCard fa18()
    {
        AircraftCard card{};
        card.id = "fa-18e";
        card.displayName = "F/A-18E";
        card.family = "United States";
        card.status = CardStatus::BrochureOnly;
        card.flyable = true;
        card.massPublished = true;
        card.massKg = 66000.0f * kPound;
        card.massText = "66,000 lb maximum takeoff";
        card.militaryPublished = true;
        card.militaryThrustN = 2.0f * 6030.0f * kGravity;
        card.militaryText = "6,030 kgf each, War Thunder";
        card.maxThrustN = 2.0f * 22000.0f * kLbf;
        card.maxThrustPublished = true;
        card.maxText = "22,000 lbf static each";
        card.areaPublished = true;
        card.wingAreaM2 = 500.0f * kFoot2;
        card.areaText = "500 ft2 approximate";
        card.positiveGPublished = true;
        card.positiveG = 11.0f;
        card.negativeGPublished = true;
        card.negativeG = -5.0f;
        card.speedPublished = true;
        card.speedEquality = true;
        card.seaLevelSpeedMps = 1497.0f / 3.6f;
        card.speedText = "1,497 km/h IAS limit, War Thunder";
        card.note = "Maximum takeoff 66,000 lb, not a combat weight. NAVAIR 22,000 lb static per engine. "
                    "Afterburner is not stated on that row, so it stays the ceiling. Wing area is the Naval Aviation News 500 ft2, approximate. "
                    "War Thunder's F/A-18E, fully modified, military 6,030 kgf each. Its afterburner figure is lower than the NAVAIR row, so it is not used. "
                    "Load is about -5 / +11 and the indicated-airspeed limit is 1,497 km/h. Its wing loading matches this area.";
        return card;
    }

    AircraftCard f22()
    {
        AircraftCard card{};
        card.id = "f-22a";
        card.displayName = "F-22A";
        card.family = "United States";
        card.status = CardStatus::BrochureOnly;
        card.flyable = true;
        card.massPublished = true;
        card.massKg = 83500.0f * kPound;
        card.massText = "83,500 lb maximum takeoff";
        card.maxThrustN = 2.0f * 35000.0f * kLbf;
        card.maxText = "35,000 lb class each";
        card.areaPublished = true;
        card.wingAreaM2 = 78.04f;
        card.areaText = "78.04 m2";
        card.note = "Maximum takeoff 83,500 lb. The 35,000 lb figure is a class, used here as the ceiling, not a measured rating. "
                    "Wing area is Lockheed 78.04 m2. Parasite drag is a stand-in. War Thunder has no F-22A.";
        return card;
    }

    AircraftCard f35()
    {
        AircraftCard card{};
        card.id = "f-35a";
        card.displayName = "F-35A";
        card.family = "United States";
        card.status = CardStatus::BrochureOnly;
        card.flyable = true;
        card.massPublished = true;
        card.massKg = 29300.0f * kPound;
        card.massText = "29,300 lb empty";
        card.militaryPublished = true;
        card.militaryThrustN = 25000.0f * kLbf;
        card.militaryText = "25,000 lbf";
        card.maxThrustPublished = true;
        card.maxThrustN = 40000.0f * kLbf;
        card.maxText = "40,000 lbf";
        card.areaPublished = true;
        card.wingAreaM2 = 460.0f * kFoot2;
        card.areaText = "460 ft2";
        card.positiveGPublished = true;
        card.positiveG = 9.0f;
        card.note = "Empty 29,300 lb. Fuel is not added, so the thrust-to-weight is high. "
                    "Product-card military 25,000 lb and maximum 40,000 lb, one engine. Wing 460 ft2. Positive load 9.0. Parasite drag is a stand-in. War Thunder has no F-35A.";
        return card;
    }

    AircraftCard gripen()
    {
        AircraftCard card{};
        card.id = "gripen-e";
        card.displayName = "JAS 39E Gripen";
        card.family = "Europe";
        card.status = CardStatus::BrochureOnly;
        card.flyable = true;
        card.massPublished = true;
        card.massKg = 16500.0f;
        card.massText = "16,500 kg maximum takeoff";
        card.militaryPublished = true;
        card.militaryThrustN = 6060.0f * kGravity;
        card.militaryText = "6,060 kgf, War Thunder";
        card.maxThrustPublished = true;
        card.maxThrustN = 98000.0f;
        card.maxText = "98 kN";
        card.areaPublished = true;
        card.wingAreaM2 = (7890.0f + 3400.0f) / 376.0f;
        card.areaText = "30.03 m2, War Thunder";
        card.positiveGPublished = true;
        card.positiveG = 9.0f;
        card.negativeGPublished = true;
        card.negativeG = -3.0f;
        card.speedPublished = true;
        card.speedEquality = true;
        card.seaLevelSpeedMps = 1543.0f / 3.6f;
        card.speedText = "above 1,400 km/h; 1,543 km/h IAS, War Thunder";
        card.note = "Maximum takeoff 16,500 kg. Saab 98 kN. Afterburner is not stated, so 98 kN stays the ceiling. Load is -3 / +9. "
                    "Sea-level speed is only published as above 1,400 km/h. War Thunder's JAS39E adds military thrust 6,060 kgf, a 30.03 m2 wing from 376 kg/m2 at full internal fuel, and an indicated-airspeed limit of 1,543 km/h, which is above that floor.";
        return card;
    }

    AircraftCard typhoon()
    {
        AircraftCard card{};
        card.id = "typhoon";
        card.displayName = "Eurofighter Typhoon";
        card.family = "Europe";
        card.status = CardStatus::BrochureOnly;
        card.flyable = true;
        card.massPublished = true;
        card.massKg = 11340.0f + 4650.0f;
        card.massText = "15,990 kg War Thunder, with fuel";
        card.militaryPublished = true;
        card.militaryThrustN = 2.0f * 60000.0f;
        card.militaryText = "60 kN each";
        card.maxThrustPublished = true;
        card.maxThrustN = 2.0f * 90000.0f;
        card.maxText = "90 kN each";
        card.areaPublished = true;
        card.wingAreaM2 = 51.2f;
        card.areaText = "51.2 m2";
        card.positiveGPublished = true;
        card.positiveG = 9.0f;
        card.negativeGPublished = true;
        card.negativeG = -3.0f;
        card.speedPublished = true;
        card.speedEquality = true;
        card.seaLevelSpeedMps = 1530.0f / 3.6f;
        card.speedText = "1,530 km/h at sea level";
        card.note = "The brochure's 11,000 kg is basic mass empty, not a flying weight. The model uses War Thunder's Typhoon FGR.4 base weight plus its 4.65 t of internal fuel. "
                    "Bundeswehr 60 kN dry and 90 kN reheat, each engine. Wing 51.2 m2. Sea level 1,530 km/h sets the drag. Load is +9 / -3.";
        return card;
    }

    AircraftCard rafale()
    {
        AircraftCard card{};
        card.id = "rafale-c";
        card.displayName = "Rafale C";
        card.family = "Europe";
        card.status = CardStatus::BrochureOnly;
        card.flyable = true;
        card.massPublished = true;
        card.massKg = 24500.0f;
        card.massText = "24.5 t maximum";
        card.militaryPublished = true;
        card.militaryThrustN = 2.0f * 4900.0f * kGravity;
        card.militaryText = "4,900 kgf each, War Thunder";
        card.maxThrustPublished = true;
        card.maxThrustN = 2.0f * 7500.0f * kGravity;
        card.maxText = "2 x 7.5 t";
        card.areaPublished = true;
        card.wingAreaM2 = (9420.0f + 4700.0f) / 310.0f;
        card.areaText = "45.55 m2, War Thunder";
        card.positiveGPublished = true;
        card.positiveG = 9.0f;
        card.negativeGPublished = true;
        card.negativeG = -3.2f;
        card.speedPublished = true;
        card.speedEquality = true;
        card.seaLevelSpeedMps = 750.0f * kKnot;
        card.speedText = "750 kt low altitude";
        card.note = "Maximum 24.5 t. Current card 2 x 7.5 t stays the ceiling. Load is -3.2 / +9. "
                    "Drag uses the DGA low-altitude 750 kt. War Thunder's Rafale C F3 supplies the wing, 45.55 m2 from 310 kg/m2 at full internal fuel, "
                    "and dry thrust 4,900 kgf each. Its afterburner figure is not substituted for the 7.5 t card.";
        return card;
    }

    AircraftCard mig29()
    {
        AircraftCard card{};
        card.id = "mig-29-9-13";
        card.displayName = "MiG-29 9.13";
        card.family = "Russia";
        card.status = CardStatus::BrochureOnly;
        card.flyable = true;
        card.massPublished = true;
        card.massKg = 15600.0f;
        card.massText = "15,600 kg normal takeoff";
        card.militaryPublished = true;
        card.militaryThrustN = 2.0f * 5040.0f * kGravity;
        card.militaryText = "5,040 kgf each";
        card.maxThrustPublished = true;
        card.maxThrustN = 2.0f * 8300.0f * kGravity;
        card.maxText = "8,300 kgf each";
        card.areaPublished = true;
        card.wingAreaM2 = 38.06f;
        card.areaText = "38.06 m2";
        card.positiveGPublished = true;
        card.positiveG = 9.0f;
        card.negativeGPublished = true;
        card.negativeG = -5.0f;
        card.speedPublished = true;
        card.speedEquality = true;
        card.seaLevelSpeedMps = 1500.0f / 3.6f;
        card.speedText = "1,500 km/h near the ground";
        card.note = "Museum normal takeoff 15,600 kg, a secondary table. Klimov sea-level static 5,040 kgf and 8,300 kgf per engine. "
                    "Museum wing 38.06 m2. Speed near the ground 1,500 km/h. Operational g is 9. War Thunder's MiG-29 (9-13) supplies the missing negative load, about -5 g.";
        return card;
    }

    AircraftCard su27()
    {
        AircraftCard card{};
        card.id = "su-27s";
        card.displayName = "Su-27S";
        card.family = "Russia";
        card.status = CardStatus::BrochureOnly;
        card.flyable = true;
        card.massPublished = true;
        card.massKg = 23430.0f;
        card.massText = "23,430 kg normal takeoff";
        card.militaryPublished = true;
        card.militaryThrustN = 2.0f * 7670.0f * kGravity;
        card.militaryText = "7,670 kgf each";
        card.maxThrustPublished = true;
        card.maxThrustN = 2.0f * 12500.0f * kGravity;
        card.maxText = "12,500 kgf each";
        card.areaPublished = true;
        card.wingAreaM2 = (16420.0f + 9400.0f) / 417.0f;
        card.areaText = "61.92 m2, War Thunder";
        card.positiveGPublished = true;
        card.positiveG = 9.0f;
        card.negativeGPublished = true;
        card.negativeG = -4.0f;
        card.speedPublished = true;
        card.speedEquality = true;
        card.seaLevelSpeedMps = 1400.0f / 3.6f;
        card.speedText = "1,400 km/h";
        card.note = "Sukhoi normal takeoff 23,430 kg with two R-27R1, two R-73E, and 5,270 kg of fuel. "
                    "Full power 7,670 kgf and afterburner 12,500 kgf, each. Sea level 1,400 km/h without stores. Operational g is +9. "
                    "War Thunder's Su-27 supplies the wing, 61.92 m2 from 417 kg/m2 at full internal fuel, and a negative load of about -4 g.";
        return card;
    }

    AircraftCard su35()
    {
        AircraftCard card{};
        card.id = "su-35s";
        card.displayName = "Su-35S";
        card.family = "Russia";
        card.status = CardStatus::BrochureOnly;
        card.flyable = true;
        card.massPublished = true;
        card.massKg = 25300.0f;
        card.massText = "25,300 kg normal takeoff";
        card.militaryPublished = true;
        card.militaryThrustN = 2.0f * 8800.0f * kGravity;
        card.militaryText = "8,800 kgf each";
        card.maxThrustPublished = true;
        card.maxThrustN = 2.0f * 14500.0f * kGravity;
        card.maxText = "14,500 kgf each";
        card.positiveGPublished = true;
        card.positiveG = 9.0f;
        card.speedPublished = true;
        card.speedEquality = true;
        card.seaLevelSpeedMps = 1400.0f / 3.6f;
        card.speedText = "1,400 km/h at 200 m";
        card.note = "KnAAPO normal takeoff 25,300 kg with two RVV-AE and two R-73E. "
                    "Static 8,800 kgf and 14,500 kgf full afterburner, each. No wing area. 1,400 km/h at 200 m. Operational g is 9. "
                    "War Thunder has no Su-35S, so the wing and the negative load stay stand-ins.";
        return card;
    }

    AircraftCard su57()
    {
        AircraftCard card{};
        card.id = "su-57";
        card.displayName = "Su-57";
        card.family = "Russia";
        card.status = CardStatus::BrochureOnly;
        card.flyable = true;
        card.massPublished = true;
        card.massKg = 26700.0f;
        card.massText = "26,700 kg normal takeoff";
        card.speedPublished = true;
        card.speedEquality = true;
        card.seaLevelSpeedMps = 1350.0f / 3.6f;
        card.speedText = "1,350 km/h";
        card.note = "Export normal takeoff 26,700 kg. Low altitude 1,350 km/h. Thrust, wing area, and g are not published. "
                    "Thrust-to-weight of 1 at that mass is a stand-in. Drag uses the published speed. War Thunder has no Su-57.";
        return card;
    }

    AircraftCard j20()
    {
        AircraftCard card{};
        card.id = "j-20a";
        card.displayName = "J-20A";
        card.family = "China";
        card.status = CardStatus::Insufficient;
        card.flyable = true;
        card.note = "No published mass, thrust, area, load, or speed. A labeled stand-in flies so the choice is not the F-16. "
                    "None of those stand-in numbers is a J-20 figure. War Thunder has no J-20A.";
        return card;
    }

    const AircraftCard kCatalog[] = {
        f16(), f15(), fa18(), f22(), f35(), gripen(), typhoon(), rafale(), mig29(), su27(), su35(), su57(), j20(),
    };
}

const AircraftCard *aircraftCatalog()
{
    return kCatalog;
}

int aircraftCatalogCount()
{
    return static_cast<int>(sizeof(kCatalog) / sizeof(kCatalog[0]));
}

const AircraftCard *findAircraft(const char *id)
{
    if (id == nullptr)
    {
        return nullptr;
    }
    for (const AircraftCard &card : kCatalog)
    {
        if (std::strcmp(card.id, id) == 0)
        {
            return &card;
        }
    }
    return nullptr;
}

const char *defaultAircraftId()
{
    return kCatalog[0].id;
}

const char *cardStatusLabel(CardStatus status)
{
    switch (status)
    {
    case CardStatus::FullTable:
        return "Full table";
    case CardStatus::BrochureOnly:
        return "Brochure";
    case CardStatus::Insufficient:
        return "Insufficient";
    }
    return "";
}
}
