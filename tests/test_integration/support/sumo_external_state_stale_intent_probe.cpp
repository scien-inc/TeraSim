/****************************************************************************/
// Test-only probe for Phase A lane completion with stale maneuver state.
/****************************************************************************/

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <libsumo/Helper.h>
#include <libsumo/Lane.h>
#include <libsumo/Simulation.h>
#include <libsumo/Vehicle.h>
#include <microsim/MSVehicle.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>
#include <utils/xml/SUMOXMLDefinitions.h>

namespace {

constexpr double NONZERO_TOLERANCE = 1e-6;
constexpr double FINAL_POS_LAT_TOLERANCE = 0.15;

void
require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::string
jsonString(const std::string& value) {
    std::string escaped;
    escaped.reserve(value.size() + 2);
    escaped.push_back('"');
    for (const char character : value) {
        if (character == '"' || character == '\\') {
            escaped.push_back('\\');
        }
        escaped.push_back(character);
    }
    escaped.push_back('"');
    return escaped;
}

void
writeNumber(const double value) {
    if (std::isfinite(value)) {
        std::cout << value;
    } else {
        std::cout << "null";
    }
}

MSVehicle&
getVehicle(const std::string& vehicleID) {
    MSVehicle* vehicle = dynamic_cast<MSVehicle*>(
        libsumo::Helper::getVehicle(vehicleID));
    if (vehicle == nullptr) {
        throw std::runtime_error("vehicle is not an MSVehicle: " + vehicleID);
    }
    return *vehicle;
}

double
distance2D(
    const libsumo::TraCIPosition& start,
    const libsumo::TraCIPosition& end
) {
    return std::hypot(end.x - start.x, end.y - start.y);
}

libsumo::TraCIPosition
positionAtDeclaredLaneOffset(
    const libsumo::TraCIPositionVector& shape,
    const double declaredLength,
    const double offset
) {
    require(shape.value.size() >= 2, "lane shape must contain two points");
    require(
        std::isfinite(declaredLength) && declaredLength > 0.,
        "lane length must be finite and positive"
    );

    double shapeLength = 0.;
    for (std::size_t index = 1; index < shape.value.size(); ++index) {
        shapeLength += distance2D(shape.value[index - 1], shape.value[index]);
    }
    const double laneFraction = std::min(
        1., std::max(0., offset / declaredLength));
    const double geometryOffset = laneFraction * shapeLength;
    double traversed = 0.;
    for (std::size_t index = 1; index < shape.value.size(); ++index) {
        const libsumo::TraCIPosition& start = shape.value[index - 1];
        const libsumo::TraCIPosition& end = shape.value[index];
        const double segmentLength = distance2D(start, end);
        if (traversed + segmentLength >= geometryOffset) {
            const double ratio = segmentLength > 0.
                                 ? (geometryOffset - traversed) / segmentLength
                                 : 0.;
            libsumo::TraCIPosition result;
            result.x = start.x + ratio * (end.x - start.x);
            result.y = start.y + ratio * (end.y - start.y);
            return result;
        }
        traversed += segmentLength;
    }
    return shape.value.back();
}

double
navigationAngle(
    const libsumo::TraCIPosition& current,
    const libsumo::TraCIPosition& next
) {
    double angle = std::atan2(next.x - current.x, next.y - current.y)
                   * 180. / M_PI;
    if (angle < 0.) {
        angle += 360.;
    }
    return angle;
}

std::string
laneChangeBitIntent(const int leftState, const int rightState) {
    const int directionalState = leftState | rightState;
    const bool wantsLeft = (directionalState & LCA_LEFT) != 0;
    const bool wantsRight = (directionalState & LCA_RIGHT) != 0;
    if (!wantsLeft && !wantsRight) {
        return "none";
    }
    if (wantsLeft && wantsRight) {
        return "both";
    }
    return wantsLeft ? "left" : "right";
}

std::pair<int, int>
laneChangeStates(const std::string& vehicleID) {
    return {
        libsumo::Vehicle::getLaneChangeState(vehicleID, 1).second,
        libsumo::Vehicle::getLaneChangeState(vehicleID, -1).second,
    };
}

void
applyImmediatePose(
    const std::string& vehicleID,
    const libsumo::TraCIPosition& position,
    const double angle,
    const bool strictLaneHint
) {
    libsumo::Vehicle::moveToXYImmediate(
        vehicleID,
        "edge_426",
        0,
        position.x,
        position.y,
        angle,
        1,
        10.,
        strictLaneHint
    );
    libsumo::Vehicle::setSpeed(vehicleID, -1.);
    libsumo::Vehicle::setPreviousSpeed(vehicleID, 5., 0.);
}

std::vector<std::string>
simulationOptions(
    const std::string& networkPath,
    const std::string& routesPath
) {
    return {
        "sumo",
        "--net-file", networkPath,
        "--route-files", routesPath,
        "--step-length", "0.05",
        "--lateral-resolution", "0.2",
        "--no-step-log", "true",
        "--duration-log.disable", "true",
        "--no-warnings", "true",
        "--seed", "42",
    };
}

}  // namespace

int
main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr
            << "usage: sumo_external_state_stale_intent_probe "
            << "NET ROUTES VEHICLE_ID STRICT\n";
        return 2;
    }

    const std::string networkPath = argv[1];
    const std::string routesPath = argv[2];
    const std::string vehicleID = argv[3];
    const bool strictLaneHint = std::string(argv[4]) == "true";

    try {
        libsumo::Simulation::start(
            simulationOptions(networkPath, routesPath));
        libsumo::Simulation::step();
        require(
            libsumo::Vehicle::getLaneID(vehicleID) == "edge_426_0",
            "vehicle did not depart on edge_426_0"
        );
        const std::vector<std::string> originalRoute =
            libsumo::Vehicle::getRoute(vehicleID);

        const libsumo::TraCIPositionVector lane0Shape =
            libsumo::Lane::getShape("edge_426_0");
        const libsumo::TraCIPositionVector lane1Shape =
            libsumo::Lane::getShape("edge_426_1");
        const double lane0Length = libsumo::Lane::getLength("edge_426_0");
        const double lane1Length = libsumo::Lane::getLength("edge_426_1");

        // Let SUMO create the lane-change maneuver naturally from a TraCI
        // request. The probe never writes maneuverDist itself.
        libsumo::Vehicle::changeLane(vehicleID, 1, 10.);
        libsumo::Simulation::executeMove();
        MSVehicle& vehicle = getVehicle(vehicleID);
        MSAbstractLaneChangeModel& laneChangeModel =
            vehicle.getLaneChangeModel();
        const double authorizedManeuverDistance =
            laneChangeModel.getManeuverDist();
        const std::pair<int, int> authorizedStates =
            laneChangeStates(vehicleID);
        require(
            std::isfinite(authorizedManeuverDistance)
            && std::abs(authorizedManeuverDistance) > NONZERO_TOLERANCE,
            "SUMO did not create a finite nonzero maneuver distance"
        );
        require(
            (authorizedStates.first & LCA_LEFT) != 0
            && (authorizedStates.first & LCA_TRACI) != 0,
            "SUMO did not authorize lane 0 to lane 1"
        );
        require(
            libsumo::Vehicle::getLaneID(vehicleID) == "edge_426_0",
            "SUMO changed primary lane while authorizing the maneuver"
        );

        // Complete one Phase A sample, then one Phase B cycle. This makes the
        // target-center update below consecutive with external-state history.
        const double historyOffset =
            libsumo::Vehicle::getLanePosition(vehicleID);
        const libsumo::TraCIPosition historyLane0 =
            positionAtDeclaredLaneOffset(
                lane0Shape, lane0Length, historyOffset);
        const libsumo::TraCIPosition historyLane1 =
            positionAtDeclaredLaneOffset(
                lane1Shape, lane1Length, historyOffset);
        const libsumo::TraCIPosition historyLane0Next =
            positionAtDeclaredLaneOffset(
                lane0Shape, lane0Length, historyOffset + 0.1);
        libsumo::TraCIPosition historyPosition;
        historyPosition.x = historyLane0.x
                            + 0.1 * (historyLane1.x - historyLane0.x);
        historyPosition.y = historyLane0.y
                            + 0.1 * (historyLane1.y - historyLane0.y);
        const double historyAngle =
            navigationAngle(historyLane0, historyLane0Next);
        applyImmediatePose(
            vehicleID, historyPosition, historyAngle, strictLaneHint);
        const double historyPhaseATime = libsumo::Simulation::getTime();
        const std::string historyPhaseALane =
            libsumo::Vehicle::getLaneID(vehicleID);
        require(
            historyPhaseALane == "edge_426_0",
            "history Phase A changed primary lane"
        );
        require(
            libsumo::Vehicle::getRoute(vehicleID) == originalRoute,
            "history Phase A changed route"
        );
        libsumo::Simulation::step();
        const std::string historyPhaseBLane =
            libsumo::Vehicle::getLaneID(vehicleID);
        require(
            historyPhaseBLane == "edge_426_0",
            "history Phase B completed the maneuver too early"
        );
        require(
            libsumo::Vehicle::getRoute(vehicleID) == originalRoute,
            "history Phase B changed route"
        );

        // Erase every source of current lane-change intent without touching
        // the naturally-created finite maneuver distance or its geometry.
        const double maneuverDistanceBeforeClear =
            laneChangeModel.getManeuverDist();
        require(
            std::isfinite(maneuverDistanceBeforeClear)
            && std::abs(maneuverDistanceBeforeClear) > NONZERO_TOLERANCE,
            "maneuver distance vanished before stale-intent setup"
        );
        vehicle.getInfluencer().setLaneTimeLine(
            std::vector<std::pair<SUMOTime, int> >());
        vehicle.getInfluencer().resetLatDist();
        laneChangeModel.setOwnState(LCA_NONE);
        for (const int direction : {-1, 0, 1}) {
            laneChangeModel.getCanceledState(direction) = LCA_NONE;
            laneChangeModel.saveLCState(direction, LCA_NONE, LCA_NONE);
        }
        const double maneuverDistanceAfterClear =
            laneChangeModel.getManeuverDist();
        const int staleOwnState = laneChangeModel.getOwnState();
        const std::pair<int, int> staleSavedLeft =
            laneChangeModel.getSavedState(1);
        const std::pair<int, int> staleSavedRight =
            laneChangeModel.getSavedState(-1);
        const std::pair<int, int> staleStates =
            laneChangeStates(vehicleID);
        require(staleOwnState == LCA_NONE, "LCM own state was not cleared");
        require(
            vehicle.getInfluencer().getLatDist() == 0.,
            "Influencer sublane intent was not cleared"
        );
        require(
            staleSavedLeft == std::make_pair(
                (int)LCA_NONE, (int)LCA_NONE)
            && staleSavedRight == std::make_pair(
                (int)LCA_NONE, (int)LCA_NONE),
            "LCM saved left/right states were not cleared"
        );
        require(
            laneChangeBitIntent(staleStates.first, staleStates.second) == "none",
            "left/right lane-change intent was not cleared"
        );
        require(
            std::isfinite(maneuverDistanceAfterClear)
            && std::abs(maneuverDistanceAfterClear) > NONZERO_TOLERANCE
            && maneuverDistanceAfterClear == maneuverDistanceBeforeClear,
            "clearing intent modified the natural maneuver distance"
        );

        const double targetOffset =
            libsumo::Vehicle::getLanePosition(vehicleID);
        const libsumo::TraCIPosition targetCenter =
            positionAtDeclaredLaneOffset(
                lane1Shape, lane1Length, targetOffset);
        const libsumo::TraCIPosition targetLane0 =
            positionAtDeclaredLaneOffset(
                lane0Shape, lane0Length, targetOffset);
        const libsumo::TraCIPosition targetLane0Next =
            positionAtDeclaredLaneOffset(
                lane0Shape, lane0Length, targetOffset + 0.1);
        const double targetAngle =
            navigationAngle(targetLane0, targetLane0Next);

        const double phaseATime = libsumo::Simulation::getTime();
        applyImmediatePose(
            vehicleID, targetCenter, targetAngle, strictLaneHint);
        const std::string phaseALane =
            libsumo::Vehicle::getLaneID(vehicleID);
        const double phaseAPosLat =
            libsumo::Vehicle::getLateralLanePosition(vehicleID);
        const std::pair<int, int> phaseAStates =
            laneChangeStates(vehicleID);
        const bool phaseARouteUnchanged =
            libsumo::Vehicle::getRoute(vehicleID) == originalRoute;
        require(phaseALane == "edge_426_0", "Phase A changed primary lane");
        require(phaseARouteUnchanged, "Phase A changed route");
        require(
            phaseAStates == std::make_pair(
                (int)LCA_NONE, (int)LCA_NONE),
            "Phase A reintroduced left/right lane-change intent"
        );

        libsumo::Simulation::step();
        const double phaseBTime = libsumo::Simulation::getTime();
        const std::string phaseBLane =
            libsumo::Vehicle::getLaneID(vehicleID);
        const double phaseBPosLat =
            libsumo::Vehicle::getLateralLanePosition(vehicleID);
        const bool phaseBRouteUnchanged =
            libsumo::Vehicle::getRoute(vehicleID) == originalRoute;
        require(phaseBLane == "edge_426_1", "Phase B did not select target lane");
        require(phaseBRouteUnchanged, "Phase B changed route");
        require(
            std::isfinite(phaseBPosLat)
            && std::abs(phaseBPosLat) <= FINAL_POS_LAT_TOLERANCE,
            "Phase B target-relative posLat is not near zero"
        );

        std::cout << std::setprecision(17)
                  << "RESULT_JSON={\"vehicle_id\":"
                  << jsonString(vehicleID)
                  << ",\"strict_lane_hint\":"
                  << (strictLaneHint ? "true" : "false")
                  << ",\"authorized_maneuver_distance\":";
        writeNumber(authorizedManeuverDistance);
        std::cout << ",\"history_phase_a_time\":";
        writeNumber(historyPhaseATime);
        std::cout << ",\"history_phase_a_lane\":"
                  << jsonString(historyPhaseALane)
                  << ",\"history_phase_b_lane\":"
                  << jsonString(historyPhaseBLane)
                  << ",\"maneuver_distance_before_clear\":";
        writeNumber(maneuverDistanceBeforeClear);
        std::cout << ",\"maneuver_distance_after_clear\":";
        writeNumber(maneuverDistanceAfterClear);
        std::cout << ",\"stale_own_state\":" << staleOwnState
                  << ",\"stale_left_state_without_traci\":"
                  << staleSavedLeft.first
                  << ",\"stale_left_state\":" << staleStates.first
                  << ",\"stale_right_state_without_traci\":"
                  << staleSavedRight.first
                  << ",\"stale_right_state\":" << staleStates.second
                  << ",\"stale_lca_bit_intent\":"
                  << jsonString(
                         laneChangeBitIntent(
                             staleStates.first, staleStates.second))
                  << ",\"phase_a_time\":";
        writeNumber(phaseATime);
        std::cout << ",\"phase_a_lane\":" << jsonString(phaseALane)
                  << ",\"phase_a_pos_lat\":";
        writeNumber(phaseAPosLat);
        std::cout << ",\"phase_a_lca_bit_intent\":"
                  << jsonString(
                         laneChangeBitIntent(
                             phaseAStates.first, phaseAStates.second))
                  << ",\"phase_a_route_unchanged\":"
                  << (phaseARouteUnchanged ? "true" : "false")
                  << ",\"phase_b_time\":";
        writeNumber(phaseBTime);
        std::cout << ",\"phase_b_lane\":" << jsonString(phaseBLane)
                  << ",\"phase_b_pos_lat\":";
        writeNumber(phaseBPosLat);
        std::cout << ",\"phase_b_route_unchanged\":"
                  << (phaseBRouteUnchanged ? "true" : "false")
                  << "}\n";

        libsumo::Simulation::close();
        return 0;
    } catch (const std::exception& error) {
        if (libsumo::Simulation::isLoaded()) {
            libsumo::Simulation::close("stale-intent probe failure");
        }
        std::cerr << "stale-intent probe failed: " << error.what() << "\n";
        return 1;
    }
}
