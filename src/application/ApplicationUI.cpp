// Control panel (Tab): one docked panel with the engagement actions and every
// simulation parameter, grouped into tabs. Uses the ui:: widgets throughout.
#include "Application.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>

#include "flight/AircraftCatalog.h"
#include "objects/Fighter.h"
#include "objects/Missile.h"
#include "objects/Target.h"
#include "physics/Atmosphere.h"
#include "physics/PhysicsEngine.h"
#include "rendering/Renderer.h"
#include "sim/Fox2Catalog.h"
#include "ui/Theme.h"
#include "ui/Widgets.h"

#include <cstring>

namespace ui = missilesim::ui;

namespace
{
    // Three-component input laid out like the other rows.
    bool vectorRow(const char *label, float values[3], const char *hint)
    {
        ImGui::PushID(label);
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const float rowHeight = ui::px(32.0f);
        const float controlWidth = std::floor(width * 0.54f);
        ImDrawList *drawList = ImGui::GetWindowDrawList();
        const float size = ui::px(ui::type::body);
        drawList->AddText(ui::fonts().medium, size, ImVec2(origin.x, origin.y + (rowHeight - size) * 0.5f),
                          ui::toU32(ui::color::text, 0.9f), label);
        if (hint != nullptr && ImGui::IsMouseHoveringRect(origin, ImVec2(origin.x + width - controlWidth, origin.y + rowHeight)))
        {
            ImGui::SetTooltip("%s", hint);
        }

        ImGui::SetCursorScreenPos(ImVec2(origin.x + width - controlWidth, origin.y + (rowHeight - ImGui::GetFrameHeight()) * 0.5f));
        ImGui::SetNextItemWidth(controlWidth);
        ImGui::PushFont(ui::fonts().mono, ui::type::body - 1.5f);
        const bool changed = ImGui::InputFloat3("##value", values, "%.0f");
        ImGui::PopFont();
        ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + rowHeight));
        ImGui::Dummy(ImVec2(0.0f, 0.0f));
        ImGui::PopID();
        return changed;
    }

    void note(const char *text)
    {
        ImGui::PushTextWrapPos(0.0f);
        ImGui::PushFont(ui::fonts().body, ui::type::body - 1.5f);
        ImGui::PushStyleColor(ImGuiCol_Text, ui::color::textFaint);
        ImGui::TextUnformatted(text);
        ImGui::PopStyleColor();
        ImGui::PopFont();
        ImGui::PopTextWrapPos();
    }

    void fox2Readout(const char *label, const char *value, bool published)
    {
        ui::readoutRow(label, value, published ? nullptr : &ui::color::textFaint);
    }

    void formatMeasure(char *buffer, size_t size, const char *format, float value)
    {
        std::snprintf(buffer, size, format, value);
    }

    void drawFox2Card(const missilesim::fox2::Spec &spec)
    {
        char line[96];
        formatMeasure(line, sizeof(line), "%.1f kg", spec.massKg);
        fox2Readout("Launch mass", line, true);
        formatMeasure(line, sizeof(line), "%.2f m", spec.lengthM);
        fox2Readout("Length", line, true);
        formatMeasure(line, sizeof(line), "%.3f m", spec.diameterM);
        fox2Readout("Diameter", line, !spec.diameterIsAssumption);
        if (spec.spanM > 0.0f)
        {
            formatMeasure(line, sizeof(line), "%.2f m", spec.spanM);
            fox2Readout("Span", line, true);
        }

        fox2Readout("Motor", missilesim::fox2::motorConfidenceLabel(spec.motor),
                    spec.motor != missilesim::fox2::MotorConfidence::UnpublishedStandIn);
        const bool motorPublished = spec.motor != missilesim::fox2::MotorConfidence::UnpublishedStandIn;
        formatMeasure(line, sizeof(line), "%.0f N", spec.resolvedThrustN);
        fox2Readout(motorPublished ? "Thrust" : "Thrust stand-in", line, motorPublished);
        formatMeasure(line, sizeof(line), "%.2f s", spec.resolvedBurnS);
        fox2Readout(motorPublished ? "Burn" : "Burn stand-in", line, motorPublished);
        if (spec.propellantKg > 0.0f)
        {
            formatMeasure(line, sizeof(line), "%.1f kg", spec.propellantKg);
            fox2Readout("Propellant in mass", line, true);
        }

        formatMeasure(line, sizeof(line), "%.1f g", spec.resolvedBurnG);
        fox2Readout(spec.hasTvc ? "Burn load" : "Structural load", line, spec.structuralGPublished);
        formatMeasure(line, sizeof(line), "%.1f g", spec.resolvedCoastG);
        fox2Readout("Coast load", line, spec.structuralGPublished);
        formatMeasure(line, sizeof(line), "%.2f", spec.resolvedCnMax);
        fox2Readout(spec.aeroGIsShapeCoefficient ? "CN shape coeff." : "CN max", line, spec.cnOverride > 0.0f);

        fox2Readout("Aspect", missilesim::fox2::aspectLabel(spec.aspect), true);
        if (spec.gimbalDeg > 0.0f)
        {
            formatMeasure(line, sizeof(line), "%.0f°", spec.gimbalDeg);
            fox2Readout("Gimbal", line, spec.gimbalPublished);
        }
        if (spec.ifovDeg > 0.0f)
        {
            formatMeasure(line, sizeof(line), "%.1f°", spec.ifovDeg);
            fox2Readout("Instantaneous field", line, spec.ifovPublished);
        }
        else
        {
            fox2Readout("Instantaneous field", "Not published", false);
        }
        if (spec.cueDeg > 0.0f)
        {
            formatMeasure(line, sizeof(line), "%.0f°", spec.cueDeg);
            fox2Readout("Cue", line, spec.cuePublished);
        }
        if (spec.trackRatePublished)
        {
            formatMeasure(line, sizeof(line), "%.1f°/s", spec.trackRateDegPerS);
            fox2Readout("Track rate", line, true);
        }
        else
        {
            fox2Readout("Track rate", "Gimbal is the stop", false);
        }

        const bool irccmKnown = spec.irccm == missilesim::fox2::IrccmKind::None || spec.irccmCircuitPublished;
        fox2Readout("IRCCM", missilesim::fox2::irccmLabel(spec.irccm), irccmKnown);
        const char *homing = "Lock before launch";
        if (spec.homing == missilesim::fox2::LaunchHoming::LockAfterLaunch)
        {
            homing = spec.rearHemisphereDesignation ? "After launch, full sphere" : "Lock after launch";
        }
        fox2Readout("Homing", homing, true);
        if (spec.hasTvc)
        {
            std::snprintf(line, sizeof(line), "±%.0f°", spec.vaneDeg);
            fox2Readout("Thrust vector", line, spec.vanePublished);
        }
        else
        {
            fox2Readout("Thrust vector", "None", true);
        }
        if (spec.armDistanceM > 0.0f)
        {
            formatMeasure(line, sizeof(line), "%.0f m", spec.armDistanceM);
            fox2Readout("Arm distance", line, spec.armDistancePublished);
        }
        if (spec.armTimeS > 0.0f)
        {
            formatMeasure(line, sizeof(line), "%.1f s", spec.armTimeS);
            fox2Readout("Arm time", line, spec.armTimePublished);
        }
        if (spec.armAfterBurnoutS > 0.0f)
        {
            formatMeasure(line, sizeof(line), "%.2f s", spec.armAfterBurnoutS);
            fox2Readout("Arm after burnout", line, false);
        }
        if (spec.proximityM > 0.0f)
        {
            formatMeasure(line, sizeof(line), "%.1f m", spec.proximityM);
            fox2Readout("Proximity", line, spec.proximityPublished);
        }
        if (spec.inhibitS > 0.0f)
        {
            formatMeasure(line, sizeof(line), "%.2f s", spec.inhibitS);
            fox2Readout("Guidance inhibit", line, spec.inhibitPublished);
        }

        ImGui::Dummy(ImVec2(0.0f, ui::px(8.0f)));
        if (spec.motorNote != nullptr && spec.motorNote[0] != '\0')
        {
            note(spec.motorNote);
            ImGui::Dummy(ImVec2(0.0f, ui::px(6.0f)));
        }
        if (spec.irccmNote != nullptr && spec.irccmNote[0] != '\0')
        {
            note(spec.irccmNote);
            ImGui::Dummy(ImVec2(0.0f, ui::px(6.0f)));
        }
        if (spec.card != nullptr && spec.card[0] != '\0')
        {
            note(spec.card);
        }
    }

    void drawAircraftCard(const missilesim::flight::AircraftCard &card)
    {
        char line[96];
        fox2Readout("Status", missilesim::flight::cardStatusLabel(card.status),
                    card.status != missilesim::flight::CardStatus::Insufficient);
        fox2Readout("Model", card.tableModel ? "Six degree of freedom" : "Point mass", true);
        if (card.massKg > 0.0f)
        {
            fox2Readout("Mass", card.massText, card.massPublished);
        }
        else
        {
            formatMeasure(line, sizeof(line), "%.0f kg stand-in", missilesim::flight::kStandInMassKg);
            fox2Readout("Mass", line, false);
        }
        const bool thrustStandIn = !card.tableModel && card.maxThrustN <= 0.0f && card.militaryThrustN <= 0.0f;
        if (thrustStandIn)
        {
            fox2Readout("Thrust", "T/W = 1 stand-in", false);
        }
        else
        {
            fox2Readout("Military thrust", card.militaryText, card.militaryPublished);
            fox2Readout("Maximum thrust", card.maxText, card.maxThrustPublished);
        }
        fox2Readout("Wing area", card.areaText, card.areaPublished);
        if (card.positiveGPublished)
        {
            formatMeasure(line, sizeof(line), "%.1f g", card.positiveG);
            fox2Readout("Positive load", line, true);
        }
        else
        {
            fox2Readout("Positive load", "9 stand-in", false);
        }
        if (card.negativeGPublished)
        {
            formatMeasure(line, sizeof(line), "%.1f g", card.negativeG);
            fox2Readout("Negative load", line, true);
        }
        else
        {
            fox2Readout("Negative load", "-3 stand-in", false);
        }
        fox2Readout("Sea-level speed", card.speedText, card.speedPublished);
        if (card.tableModel)
        {
            fox2Readout("Drag", "NASA TP-1538 tables", true);
        }
        else if (card.speedEquality)
        {
            fox2Readout("Drag", "Matched to the sea-level speed", true);
        }
        else if (card.areaPublished)
        {
            fox2Readout("Drag", "Cd0 0.020 stand-in", false);
        }
        else
        {
            formatMeasure(line, sizeof(line), "%.0f km/h stand-in", missilesim::flight::kStandInSeaLevelMps * 3.6f);
            fox2Readout("Drag", line, false);
        }
        if (card.note != nullptr && card.note[0] != '\0')
        {
            ImGui::Dummy(ImVec2(0.0f, ui::px(8.0f)));
            note(card.note);
        }
    }

    const char *aiStateName(TargetAIState state)
    {
        switch (state)
        {
        case TargetAIState::PATROL:
            return "Patrol";
        case TargetAIState::REPOSITION:
            return "Reposition";
        case TargetAIState::DEFENSIVE:
            return "Defensive";
        case TargetAIState::RECOVERING:
            return "Recover";
        default:
            return "Unknown";
        }
    }
}

void Application::setupUI()
{
    if (!m_world || !m_renderer)
    {
        return;
    }

    // Rounds already in the air keep what they were launched with.
    auto applyLiveMissileConfig = [&]()
    {
        m_world->setCustomRoundSpec(customRoundSpec(), true);
    };

    auto applyLiveTargetAIConfig = [&]()
    {
        m_world->setTargetAIConfig(m_targetAIConfig);
        m_world->applyTargetAIConfigToAll();
    };

    // ---- Panel frame ------------------------------------------------------------
    const ImGuiViewport *viewport = ImGui::GetMainViewport();
    const float edge = ui::px(12.0f);
    const float panelWidth = ui::px(kControlPanelWidth);
    // Slide in each time the panel opens (it is not drawn while hidden, so a
    // gap in frames means it was just reopened).
    const int frame = ImGui::GetFrameCount();
    if (frame - m_controlPanelLastFrame > 1)
    {
        m_controlPanelAppear = 0.0f;
    }
    m_controlPanelLastFrame = frame;
    m_controlPanelAppear += (1.0f - m_controlPanelAppear) * (1.0f - std::exp(-14.0f * ImGui::GetIO().DeltaTime));
    const float appear = m_controlPanelAppear;
    ImGui::SetNextWindowPos(ImVec2(viewport->Pos.x + viewport->Size.x - edge - panelWidth + (1.0f - appear) * ui::px(28.0f),
                                   viewport->Pos.y + edge));
    ImGui::SetNextWindowSize(ImVec2(panelWidth, viewport->Size.y - edge * 2.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, appear);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, ui::px(10.0f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(ui::px(22.0f), ui::px(20.0f)));
    const bool open = ImGui::Begin("##control-panel", nullptr,
                                   ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                       ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollWithMouse);
    ImGui::PopStyleVar(2);
    if (!open)
    {
        ImGui::End();
        ImGui::PopStyleVar();
        return;
    }

    ImDrawList *drawList = ImGui::GetWindowDrawList();

    // Header: title + hide hint.
    {
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        ui::drawTracked(drawList, ui::fonts().display, ui::px(21.0f), origin, ui::toU32(ui::color::text), "CONTROLS", 0.12f);
        const char *hide = "Hide";
        const float hideWidth = ui::fonts().body->CalcTextSizeA(ui::px(14.0f), FLT_MAX, 0.0f, hide).x;
        const float hideX = origin.x + width - hideWidth;
        drawList->AddText(ui::fonts().body, ui::px(14.0f), ImVec2(hideX, origin.y + ui::px(3.0f)), ui::toU32(ui::color::textMuted), hide);
        const float capWidth = ui::fonts().monoMedium->CalcTextSizeA(ui::px(12.5f), FLT_MAX, 0.0f, "Tab").x + ui::px(14.0f);
        ui::drawKeycap(drawList, ImVec2(hideX - capWidth - ui::px(8.0f), origin.y), "Tab");
        ImGui::Dummy(ImVec2(width, ui::px(34.0f)));
    }

    // Primary actions.
    {
        const float spacing = ImGui::GetStyle().ItemSpacing.x;
        const float buttonWidth = (ImGui::GetContentRegionAvail().x - spacing * 2.0f) / 3.0f;
        const missilesim::sim::LaunchBlock block = m_world->launchClearance();
        const char *blockedLabel = "UNAVAILABLE";
        switch (block)
        {
        case missilesim::sim::LaunchBlock::NoRound:
            blockedLabel = "EMPTY";
            break;
        case missilesim::sim::LaunchBlock::Reloading:
            blockedLabel = "RELOADING";
            break;
        case missilesim::sim::LaunchBlock::SeekerCaged:
            blockedLabel = "CAGED";
            break;
        case missilesim::sim::LaunchBlock::NoDesignation:
            blockedLabel = "NO TARGET";
            break;
        case missilesim::sim::LaunchBlock::NeedsInfraredLock:
            blockedLabel = "NO IR LOCK";
            break;
        default:
            break;
        }
        if (block != missilesim::sim::LaunchBlock::None)
        {
            ui::button(blockedLabel, ui::ButtonStyle::Ghost, buttonWidth);
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("%s", missilesim::sim::launchBlockMessage(block));
            }
        }
        else if (ui::button("LAUNCH", ui::ButtonStyle::Primary, buttonWidth))
        {
            launchMissile();
        }
        ImGui::SameLine();
        if (ui::button("REARM", ui::ButtonStyle::Secondary, buttonWidth))
        {
            rearm();
        }
        ImGui::SameLine();
        if (ui::button(m_isPaused ? "RESUME" : "PAUSE", ui::ButtonStyle::Secondary, buttonWidth))
        {
            m_isPaused = !m_isPaused;
        }
    }

    // Tab bar.
    {
        ImGui::Dummy(ImVec2(0.0f, ui::px(6.0f)));
        const char *const tabs[] = {"MISSILE", "TARGETS", "WORLD", "TELEMETRY"};
        const ImVec2 origin = ImGui::GetCursorScreenPos();
        const float width = ImGui::GetContentRegionAvail().x;
        const float tabHeight = ui::px(38.0f);
        const float size = ui::px(14.0f);
        const float tracking = 0.14f;
        float x = origin.x;
        for (int i = 0; i < static_cast<int>(std::size(tabs)); ++i)
        {
            const float labelWidth = ui::measureTracked(ui::fonts().display, size, tabs[i], tracking).x;
            ImGui::SetCursorScreenPos(ImVec2(x, origin.y));
            ImGui::PushID(i);
            if (ImGui::InvisibleButton("##tab", ImVec2(labelWidth + ui::px(4.0f), tabHeight)))
            {
                m_controlPanelTab = i;
            }
            const bool hovered = ImGui::IsItemHovered();
            ImGui::PopID();
            if (hovered)
            {
                ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
            }
            const bool selected = m_controlPanelTab == i;
            const ImVec4 colour = selected ? ui::color::text : (hovered ? ui::withAlpha(ui::color::text, 0.8f) : ui::color::textFaint);
            ui::drawTracked(drawList, ui::fonts().display, size, ImVec2(x, origin.y + (tabHeight - size) * 0.5f), ui::toU32(colour), tabs[i], tracking);
            if (selected)
            {
                drawList->AddRectFilled(ImVec2(x, origin.y + tabHeight - ui::px(2.0f)), ImVec2(x + labelWidth - size * tracking, origin.y + tabHeight),
                                        ui::toU32(ui::color::accent));
            }
            x += labelWidth + ui::px(22.0f);
        }
        drawList->AddLine(ImVec2(origin.x, origin.y + tabHeight), ImVec2(origin.x + width, origin.y + tabHeight), ui::toU32(ui::color::hairline));
        ImGui::SetCursorScreenPos(ImVec2(origin.x, origin.y + tabHeight + ui::px(4.0f)));
        ImGui::Dummy(ImVec2(0.0f, 0.0f));
    }

    // Scrolling body; rows sit tighter than the default item spacing.
    ImGui::BeginChild("##control-panel-body", ImVec2(0.0f, 0.0f), ImGuiChildFlags_None, ImGuiWindowFlags_NoBackground);
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(ImGui::GetStyle().ItemSpacing.x, ui::px(2.0f)));

    switch (m_controlPanelTab)
    {
    case 0: // Missile
    {
        ui::sectionLabel("ROLE");
        int roleIndex = m_playerRole == PlayerRole::Fighter ? 1 : 0;
        const char *const roles[] = {"SAM", "FIGHTER"};
        if (ui::segmentedRow("Player", &roleIndex, roles, 2,
                             "SAM fires the custom cold-launch round. Fighter carries a catalog Fox 2 on the wingtip rails."))
        {
            setPlayerRole(roleIndex == 1 ? PlayerRole::Fighter : PlayerRole::Sam);
        }

        if (m_playerRole == PlayerRole::Fighter)
        {
            note("Two wingtip rounds, fired right rail first; both can be in the air at once. Each leaves at the fighter's speed: no vertical hop, no booster multiplier, no terrain avoidance. S pulls and W pushes. A and D command roll rate, so a tap sets the bank and holding the key keeps rolling. Q and E rudder, Shift and Ctrl throttle, X afterburner, G rearm, F launch, R uncage.");

            ui::sectionLabel("AIRCRAFT");
            note("The picture stays models/jet.obj. Choosing a name replaces the whole flight model. A faint row is not a published number.");
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(1.0f, 1.0f, 1.0f, 0.03f));
            ImGui::BeginChild("##aircraft-catalog", ImVec2(0.0f, ui::px(280.0f)), ImGuiChildFlags_Borders);
            const char *const aircraftFamilies[] = {"United States", "Europe", "Russia", "China"};
            const missilesim::flight::AircraftCard *cards = missilesim::flight::aircraftCatalog();
            const int cardCount = missilesim::flight::aircraftCatalogCount();
            auto drawAircraft = [&](const missilesim::flight::AircraftCard &card)
            {
                ImGui::PushID(card.id);
                const bool selected = m_aircraftId == card.id;
                if (ImGui::Selectable(card.displayName, selected, ImGuiSelectableFlags_None, ImVec2(0.0f, ui::px(26.0f))))
                {
                    selectAircraft(card.id);
                }
                ImGui::PopID();
            };
            for (const char *family : aircraftFamilies)
            {
                bool any = false;
                for (int cardIndex = 0; cardIndex < cardCount; ++cardIndex)
                {
                    if (cards[cardIndex].family != nullptr && std::strcmp(cards[cardIndex].family, family) == 0)
                    {
                        if (!any)
                        {
                            ui::sectionLabel(family);
                            any = true;
                        }
                        drawAircraft(cards[cardIndex]);
                    }
                }
            }
            ImGui::EndChild();
            ImGui::PopStyleColor();
            const missilesim::flight::AircraftCard *selectedAircraft = missilesim::flight::findAircraft(m_aircraftId.c_str());
            if (selectedAircraft != nullptr)
            {
                ui::sectionLabel(selectedAircraft->displayName);
                drawAircraftCard(*selectedAircraft);
            }
            if (!m_world->shots().empty())
            {
                ImGui::Dummy(ImVec2(0.0f, ui::px(6.0f)));
                note("Rounds already in the air keep their seeker. This choice reloads the rails that are still loaded.");
            }

            ui::sectionLabel("FOX 2");
            ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(1.0f, 1.0f, 1.0f, 0.03f));
            ImGui::BeginChild("##fox2-catalog", ImVec2(0.0f, ui::px(280.0f)), ImGuiChildFlags_Borders);
            const char *const families[] = {
                "Sidewinder", "Russia", "France", "Germany", "United Kingdom",
                "Israel", "China", "Japan", "South Africa"};
            const int roundCount = missilesim::fox2::catalogCount();
            const missilesim::fox2::Spec *rounds = missilesim::fox2::catalog();
            auto drawRound = [&](const missilesim::fox2::Spec &round)
            {
                ImGui::PushID(round.id);
                const bool selected = m_fox2Id == round.id;
                if (ImGui::Selectable(round.displayName, selected, ImGuiSelectableFlags_None, ImVec2(0.0f, ui::px(26.0f))))
                {
                    selectFox2(round.id);
                }
                ImGui::PopID();
            };
            for (const char *family : families)
            {
                bool any = false;
                for (int roundIndex = 0; roundIndex < roundCount; ++roundIndex)
                {
                    if (rounds[roundIndex].family != nullptr && std::strcmp(rounds[roundIndex].family, family) == 0)
                    {
                        if (!any)
                        {
                            ui::sectionLabel(family);
                            any = true;
                        }
                        drawRound(rounds[roundIndex]);
                    }
                }
            }
            for (int roundIndex = 0; roundIndex < roundCount; ++roundIndex)
            {
                bool listed = false;
                for (const char *family : families)
                {
                    if (rounds[roundIndex].family != nullptr && std::strcmp(rounds[roundIndex].family, family) == 0)
                    {
                        listed = true;
                        break;
                    }
                }
                if (!listed)
                {
                    if (rounds[roundIndex].family != nullptr)
                    {
                        ui::sectionLabel(rounds[roundIndex].family);
                    }
                    drawRound(rounds[roundIndex]);
                }
            }
            ImGui::EndChild();
            ImGui::PopStyleColor();

            const missilesim::fox2::Spec *selected = missilesim::fox2::find(m_fox2Id.c_str());
            if (selected != nullptr)
            {
                ui::sectionLabel(selected->displayName);
                drawFox2Card(*selected);
            }
            break;
        }

        note("Custom cold-launch round. A vertical hop, a four-times boost, and terrain avoidance stay on this missile. The cell reloads two seconds after each launch, so several rounds can be in the air. The catalog rounds are the fighter's.");
        ImGui::Dummy(ImVec2(0.0f, ui::px(6.0f)));

        ui::sectionLabel("LAUNCHER");
        vectorRow("Launch position", m_initialPosition, "Where the round sits before launch (x, y, z metres).");
        vectorRow("Launch velocity", m_initialVelocity, "Initial velocity given at launch (x, y, z m/s).");

        ui::sectionLabel("AIRFRAME");
        ui::sliderRow("Dry mass", &m_mass, 10.0f, 1000.0f, "%.0f kg", "Mass without propellant.");
        ui::sliderRow("Drag coefficient", &m_dragCoefficient, 0.01f, 1.0f, "%.3f");
        ui::sliderRow("Cross-section", &m_crossSectionalArea, 0.01f, 1.0f, "%.3f m\xC2\xB2", "Frontal area used for drag.");
        ui::sliderRow("Lift coefficient", &m_liftCoefficient, 0.0f, 1.0f, "%.3f");

        ui::sectionLabel("PROPULSION");
        ui::sliderRow("Thrust", &m_missileThrust, 1000.0f, 50000.0f, "%.0f N", "Sustainer thrust after the boost phase.");
        ui::sliderRow("Propellant", &m_missileFuel, 10.0f, 1000.0f, "%.0f kg");
        ui::sliderRow("Burn rate", &m_missileFuelConsumptionRate, 0.1f, 10.0f, "%.2f kg/s");

        ui::sectionLabel("GUIDANCE");
        ui::toggleRow("Guidance", &m_guidanceEnabled, "Proportional navigation toward the seeker's target.");
        ui::sliderRow("Lead aggressiveness", &m_navigationGain, 1.0f, 4.0f, "%.2f",
                      "Navigation gain: how hard the missile leads a crossing target.");
        ui::sliderRow("Steering force", &m_maxSteeringForce, 1000.0f, 50000.0f, "%.0f N", "Maximum lateral force guidance may command.");
        ui::sliderRow("Seeker field of view", &m_trackingAngle, 5.0f, 180.0f, "%.0f\xC2\xB0", "Half-angle the seeker can see off the nose.");
        ui::sliderRow("Proximity fuse", &m_proximityFuseRadius, 0.0f, 75.0f, "%.0f m", "Detonation distance from the target.");
        ui::sliderRow("Flare rejection", &m_countermeasureResistance, 0.0f, 1.0f, "%.2f",
                      "IRCCM: how well the seeker ignores flares (1 = immune).");

        ui::sectionLabel("TERRAIN");
        ui::toggleRow("Terrain avoidance", &m_terrainAvoidanceEnabled);
        ui::sliderRow("Minimum clearance", &m_terrainClearance, 0.0f, 400.0f, "%.0f m");
        ui::sliderRow("Look-ahead", &m_terrainLookAheadTime, 0.5f, 12.0f, "%.1f s");

        ImGui::Dummy(ImVec2(0.0f, ui::px(12.0f)));
        note("Changes apply to the next round loaded. Apply now to reload the round in the cell; rounds in the air keep theirs.");
        ImGui::Dummy(ImVec2(0.0f, ui::px(6.0f)));
        if (ui::button("APPLY NOW", ui::ButtonStyle::Secondary, -1.0f))
        {
            applyLiveMissileConfig();
        }
        break;
    }
    case 1: // Targets
    {
        ui::sectionLabel("FORMATION");
        ui::sliderRowInt("Aircraft", &m_targetCount, 1, 20, "%d", "Number of fighters. Respawns on release.");
        if (ImGui::IsItemDeactivatedAfterEdit())
        {
            resetTargets();
        }
        ui::sliderRow("Stand-off distance", &m_targetAIConfig.preferredDistance, 300.0f, 20000.0f, "%.0f m",
                      "Average distance the fighters keep from the launcher. Respawns on release.");
        if (ImGui::IsItemDeactivatedAfterEdit())
        {
            resetTargets();
        }
        ui::sliderRow("Minimum speed", &m_targetAIConfig.minSpeed, 60.0f, 450.0f, "%.0f m/s");
        m_targetAIConfig.maxSpeed = std::max(m_targetAIConfig.maxSpeed, m_targetAIConfig.minSpeed + 10.0f);
        ui::sliderRow("Maximum speed", &m_targetAIConfig.maxSpeed, m_targetAIConfig.minSpeed + 10.0f, 600.0f, "%.0f m/s");

        ImGui::Dummy(ImVec2(0.0f, ui::px(10.0f)));
        const float spacing = ImGui::GetStyle().ItemSpacing.x;
        const float halfWidth = (ImGui::GetContentRegionAvail().x - spacing) * 0.5f;
        if (ui::button("APPLY SPEEDS", ui::ButtonStyle::Secondary, halfWidth))
        {
            applyLiveTargetAIConfig();
        }
        ImGui::SameLine();
        if (ui::button("RESPAWN", ui::ButtonStyle::Secondary, halfWidth))
        {
            resetTargets();
        }

        ui::sectionLabel("ROSTER");
        const Missile *focus = focusMissile();
        const Fighter *jet = fighter();
        const glm::vec3 missilePosition = focus != nullptr ? focus->getPosition()
                                                           : (jet != nullptr ? jet->getPosition() : glm::vec3(0.0f));
        if (ImGui::BeginTable("##roster", 5, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_RowBg | ImGuiTableFlags_PadOuterX))
        {
            ImGui::PushFont(ui::fonts().medium, ui::type::body - 2.0f);
            ImGui::TableSetupColumn("ID", ImGuiTableColumnFlags_WidthFixed, ui::px(30.0f));
            ImGui::TableSetupColumn("State");
            ImGui::TableSetupColumn("Alt");
            ImGui::TableSetupColumn("Range");
            ImGui::TableSetupColumn("Flares", ImGuiTableColumnFlags_WidthFixed, ui::px(44.0f));
            ImGui::PushStyleColor(ImGuiCol_Text, ui::color::textMuted);
            ImGui::TableHeadersRow();
            ImGui::PopStyleColor();
            ImGui::PopFont();

            ImGui::PushFont(ui::fonts().mono, ui::type::body - 2.5f);
            const std::vector<std::unique_ptr<Target>> &aircraft = targets();
            for (size_t i = 0; i < aircraft.size(); ++i)
            {
                const Target *target = aircraft[i].get();
                const bool active = target->isActive();
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::Text("T%zu", i + 1);
                ImGui::TableSetColumnIndex(1);
                ImGui::PushFont(ui::fonts().medium, ui::type::body - 2.0f);
                if (active)
                {
                    const bool defensive = target->getAIState() == TargetAIState::DEFENSIVE;
                    ImGui::TextColored(defensive ? ui::color::danger : ui::color::text, "%s", aiStateName(target->getAIState()));
                }
                else
                {
                    ImGui::TextColored(ui::color::textFaint, "Down");
                }
                ImGui::PopFont();
                ImGui::TableSetColumnIndex(2);
                active ? ImGui::Text("%.0f m", std::max(target->getPosition().y, 0.0f)) : ImGui::TextDisabled("\xE2\x80\x94");
                ImGui::TableSetColumnIndex(3);
                const float range = glm::distance(missilePosition, target->getPosition());
                active ? ImGui::Text(range < 1000.0f ? "%.0f m" : "%.2f km", range < 1000.0f ? range : range / 1000.0f)
                       : ImGui::TextDisabled("\xE2\x80\x94");
                ImGui::TableSetColumnIndex(4);
                active ? ImGui::Text("%d", target->getRemainingFlares()) : ImGui::TextDisabled("\xE2\x80\x94");
            }
            ImGui::PopFont();
            ImGui::EndTable();
        }
        break;
    }
    case 2: // World
    {
        ui::sectionLabel("SIMULATION");
        ui::sliderRow("Time scale", &m_simulationSpeed, 0.1f, 10.0f, "%.1f\xC3\x97", "Simulation speed relative to real time.");
        float gravity = physics()->getGravity();
        if (ui::sliderRow("Gravity", &gravity, 0.0f, 20.0f, "%.2f m/s\xC2\xB2"))
        {
            physics()->setGravity(gravity);
        }
        float airDensity = physics()->getAirDensity();
        if (ui::sliderRow("Sea-level air density", &airDensity, 0.0f, 2.0f, "%.3f kg/m\xC2\xB3"))
        {
            physics()->setAirDensity(airDensity);
        }
        if (ui::toggleRow("Ground collision", &m_groundEnabled))
        {
            physics()->setGroundEnabled(m_groundEnabled);
        }
        if (m_groundEnabled && ui::sliderRow("Ground bounce", &m_groundRestitution, 0.0f, 1.0f, "%.2f", "Restitution of ground impacts."))
        {
            physics()->setGroundRestitution(m_groundRestitution);
        }

        ui::sectionLabel("OVERLAYS");
        ui::toggleRow("Predicted trajectory", &m_showTrajectory);
        ui::sliderRowInt("Trajectory detail", &m_trajectoryPoints, 10, 600, "%d");
        ui::sliderRow("Prediction horizon", &m_trajectoryTime, 0.5f, 60.0f, "%.1f s");
        ui::toggleRow("Target flight path", &m_showPredictedTargetPath);
        ui::toggleRow("Intercept point", &m_showInterceptPoint);
        ui::toggleRow("Target labels", &m_showTargetInfo);
        ui::toggleRow("Seeker x-ray", &m_seekerXrayEnabled, "Marks what the seeker is tracking, through terrain, once in flight.");
        bool guides = m_renderer->getWorldGuidesEnabled();
        if (ui::toggleRow("Airspace guides", &guides, "Range rings, the airspace boundary and corner beacons."))
        {
            m_renderer->setWorldGuidesEnabled(guides);
        }

        ui::sectionLabel("CAMERA");
        float cameraSpeed = m_renderer->getCameraSpeed();
        if (ui::sliderRow("Free camera speed", &cameraSpeed, 1.0f, 800.0f, "%.0f m/s"))
        {
            m_renderer->setCameraSpeed(cameraSpeed);
        }
        ImGui::Dummy(ImVec2(0.0f, ui::px(8.0f)));
        if (ui::button("FRAME ENGAGEMENT", ui::ButtonStyle::Secondary, -1.0f))
        {
            setCameraMode(CameraMode::FREE, true);
        }
        break;
    }
    default: // Telemetry
    {
        char buffer[96];
        const Missile *focus = focusMissile();
        const missilesim::sim::Shot *shot = followedShot();

        ui::sectionLabel("MISSION");
        ui::readoutRow("State", missionStateLabel());
        std::snprintf(buffer, sizeof(buffer), "%016llx", static_cast<unsigned long long>(m_world->seed()));
        ui::readoutRow("Scenario seed", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.1f s", m_world->time());
        ui::readoutRow("Simulation time", buffer);
        std::snprintf(buffer, sizeof(buffer), "%zu", m_world->shots().size());
        ui::readoutRow("Rounds in the air", buffer);
        if (focus == nullptr)
        {
            ui::readoutRow("Seeker", "No round");
            break;
        }

        const glm::vec3 position = focus->getPosition();
        const glm::vec3 velocity = focus->getVelocity();
        const glm::vec3 acceleration = focus->getAcceleration();
        const float speed = glm::length(velocity);
        const float altitude = std::max(position.y, 0.0f);
        const Atmosphere::State air = physics()->getAtmosphereState(altitude);
        const Target *tracked = getTrackedMissileTarget();
        ui::readoutRow("Seeker", getMissileSeekerStateLabel());
        if (tracked != nullptr)
        {
            std::snprintf(buffer, sizeof(buffer), "%.0f m", glm::distance(position, tracked->getPosition()));
            ui::readoutRow("Target range", buffer);
        }
        else
        {
            ui::readoutRow("Target range", "No lock");
        }
        if (shot != nullptr && shot->closestApproach >= 0.0f)
        {
            std::snprintf(buffer, sizeof(buffer), "%.1f m", shot->closestApproach);
            ui::readoutRow("Closest pass", buffer);
        }
        else
        {
            ui::readoutRow("Closest pass", "\xE2\x80\x94");
        }
        std::snprintf(buffer, sizeof(buffer), "%.1f s", shot != nullptr ? shot->flightTime : 0.0f);
        ui::readoutRow("Flight time", buffer);

        ui::sectionLabel("MISSILE");
        std::snprintf(buffer, sizeof(buffer), "%.0f, %.0f, %.0f", position.x, position.y, position.z);
        ui::readoutRow("Position (m)", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.0f, %.0f, %.0f", velocity.x, velocity.y, velocity.z);
        ui::readoutRow("Velocity (m/s)", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.1f m/s\xC2\xB2", glm::length(acceleration));
        ui::readoutRow("Acceleration", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.0f km/h", speed * 3.6f);
        ui::readoutRow("Speed", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.2f", air.speedOfSoundMetersPerSecond > 0.0f ? speed / air.speedOfSoundMetersPerSecond : 0.0f);
        ui::readoutRow("Mach", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.0f m", position.y - physics()->getGroundLevel());
        ui::readoutRow("Terrain clearance", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.1f kg", focus->getMass());
        ui::readoutRow("Mass", buffer);

        ui::sectionLabel("PROPULSION");
        const bool thrusting = focus->isThrustEnabled();
        const bool burnedOut = !thrusting && focus->getFuel() <= 0.0f;
        const ImVec4 motorColour = thrusting ? ui::color::accent : ui::color::textMuted;
        ui::readoutRow("Motor", thrusting ? "Burning" : (burnedOut ? "Burned out" : "Off"), &motorColour);
        std::snprintf(buffer, sizeof(buffer), "%.0f N  \xC2\xB7  %.0f%%", focus->getThrust(), focus->getThrottle() * 100.0f);
        ui::readoutRow("Thrust", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.1f kg", focus->getFuel());
        ui::readoutRow("Propellant", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.2f kg/s", focus->getFuelConsumptionRate());
        ui::readoutRow("Burn rate", buffer);

        ui::sectionLabel("ATMOSPHERE");
        std::snprintf(buffer, sizeof(buffer), "%.3f kg/m\xC2\xB3", air.densityKgPerCubicMeter);
        ui::readoutRow("Density", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.1f kPa", air.pressurePascals * 0.001f);
        ui::readoutRow("Pressure", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.1f \xC2\xB0""C", air.temperatureKelvin - 273.15f);
        ui::readoutRow("Temperature", buffer);
        std::snprintf(buffer, sizeof(buffer), "%.0f m/s", air.speedOfSoundMetersPerSecond);
        ui::readoutRow("Speed of sound", buffer);
        break;
    }
    }

    ImGui::Dummy(ImVec2(0.0f, ui::px(16.0f)));
    ImGui::PopStyleVar();
    ImGui::EndChild();
    ImGui::End();
    ImGui::PopStyleVar();
}
