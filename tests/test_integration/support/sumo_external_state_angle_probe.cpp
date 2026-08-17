/****************************************************************************/
// Test-only probe for moveToXYImmediate / lane-change angle-state divergence.
/****************************************************************************/

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

namespace {

constexpr double STEP_LENGTH = 0.05;
constexpr int MAX_SCAN_STEPS = 5000;

struct Snapshot {
    std::string primaryLane;
    std::string targetLane;
    std::string shadowLane;
    double speed;
    double lateralSpeed;
    double reportedLateralSpeed;
    double lateralPosition;
    double lanePosition;
    int laneIndex;
    double laneAngle;
    double angle;
    double angleOffset;
    double completion;
    int direction;
    bool changing;
    double maneuverDistance;
    double previousManeuverDistance;
    int ownState;
    int leftState;
    int rightState;
};

struct Candidate {
    std::string vehicleID;
    Snapshot state;
    libsumo::TraCIPosition position;
    double simulationTime;
};

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

std::string
laneID(const MSLane* lane) {
    return lane == nullptr ? "" : lane->getID();
}

std::string
laneChangeBitIntent(const int leftState, const int rightState) {
    const int lcaLeft = 1 << 1;
    const int lcaRight = 1 << 2;
    const bool wantsLeft = (leftState & lcaLeft) != 0;
    const bool wantsRight = (rightState & lcaRight) != 0;
    if (wantsLeft == wantsRight) {
        return "none";
    }
    return wantsLeft ? "left" : "right";
}

Snapshot
snapshot(const std::string& vehicleID) {
    MSBaseVehicle* baseVehicle = libsumo::Helper::getVehicle(vehicleID);
    MSVehicle* vehicle = dynamic_cast<MSVehicle*>(baseVehicle);
    if (vehicle == nullptr) {
        throw std::runtime_error("vehicle is not an MSVehicle: " + vehicleID);
    }
    const MSAbstractLaneChangeModel& laneChangeModel =
        vehicle->getLaneChangeModel();
    const std::pair<int, int> leftState =
        libsumo::Vehicle::getLaneChangeState(vehicleID, 1);
    const std::pair<int, int> rightState =
        libsumo::Vehicle::getLaneChangeState(vehicleID, -1);
    const std::string primaryLane = libsumo::Vehicle::getLaneID(vehicleID);
    const double lanePosition =
        libsumo::Vehicle::getLanePosition(vehicleID);
    return {
        primaryLane,
        laneID(laneChangeModel.getTargetLane()),
        laneID(laneChangeModel.getShadowLane()),
        libsumo::Vehicle::getSpeed(vehicleID),
        laneChangeModel.getSpeedLat(),
        libsumo::Vehicle::getLateralSpeed(vehicleID),
        libsumo::Vehicle::getLateralLanePosition(vehicleID),
        lanePosition,
        libsumo::Vehicle::getLaneIndex(vehicleID),
        libsumo::Lane::getAngle(primaryLane, lanePosition),
        libsumo::Vehicle::getAngle(vehicleID),
        laneChangeModel.getAngleOffset(),
        laneChangeModel.getLaneChangeCompletion(),
        laneChangeModel.getLaneChangeDirection(),
        laneChangeModel.isChangingLanes(),
        laneChangeModel.getManeuverDist(),
        laneChangeModel.getPreviousManeuverDist(),
        laneChangeModel.getOwnState(),
        leftState.second,
        rightState.second,
    };
}

void
writeSnapshot(const Snapshot& value) {
    const double radiansToDegrees = 180.0 / M_PI;
    std::cout
        << "{\"primary_lane\":" << jsonString(value.primaryLane)
        << ",\"target_lane\":" << jsonString(value.targetLane)
        << ",\"shadow_lane\":" << jsonString(value.shadowLane)
        << ",\"lca_bit_intent\":"
        << jsonString(laneChangeBitIntent(value.leftState, value.rightState))
        << ",\"speed\":";
    writeNumber(value.speed);
    std::cout << ",\"lateral_speed\":";
    writeNumber(value.lateralSpeed);
    std::cout << ",\"reported_lateral_speed\":";
    writeNumber(value.reportedLateralSpeed);
    std::cout << ",\"lateral_position\":";
    writeNumber(value.lateralPosition);
    std::cout << ",\"lane_position\":";
    writeNumber(value.lanePosition);
    std::cout << ",\"lane_index\":" << value.laneIndex;
    std::cout << ",\"lane_angle\":";
    writeNumber(value.laneAngle);
    std::cout << ",\"angle\":";
    writeNumber(value.angle);
    std::cout << ",\"angle_minus_lane_angle\":";
    writeNumber(std::remainder(value.angle - value.laneAngle, 360.0));
    std::cout << ",\"angle_offset_radians\":";
    writeNumber(value.angleOffset);
    std::cout << ",\"angle_offset_degrees\":";
    writeNumber(value.angleOffset * radiansToDegrees);
    std::cout << ",\"completion\":";
    writeNumber(value.completion);
    std::cout
        << ",\"direction\":" << value.direction
        << ",\"changing\":" << (value.changing ? "true" : "false")
        << ",\"maneuver_distance\":";
    writeNumber(value.maneuverDistance);
    std::cout << ",\"previous_maneuver_distance\":";
    writeNumber(value.previousManeuverDistance);
    std::cout
        << ",\"own_state\":" << value.ownState
        << ",\"left_state\":" << value.leftState
        << ",\"right_state\":" << value.rightState
        << "}";
}

double
angleDifference(const double left, const double right) {
    return std::abs(std::remainder(left - right, 360.0));
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
        "--lateral-resolution", "0.5",
        "--no-step-log", "true",
        "--duration-log.disable", "true",
        "--no-warnings", "true",
        "--seed", "42",
    };
}

Candidate
findCandidate(const double lateralSign) {
    for (int step = 0; step < MAX_SCAN_STEPS; ++step) {
        libsumo::Simulation::step();
        for (const std::string& vehicleID : libsumo::Vehicle::getIDList()) {
            const Snapshot state = snapshot(vehicleID);
            if (
                state.speed <= 0.2
                && lateralSign * state.lateralSpeed >= 0.5
                && laneChangeBitIntent(state.leftState, state.rightState) == "none"
                && !state.primaryLane.empty()
                && state.primaryLane.front() != ':'
            ) {
                return {
                    vehicleID,
                    state,
                    libsumo::Vehicle::getPosition(vehicleID, true),
                    libsumo::Simulation::getTime(),
                };
            }
        }
    }
    throw std::runtime_error(
        "no low-speed lateral-motion candidate with service intent none"
    );
}

}  // namespace

int
main(int argc, char** argv) {
    if (argc != 6) {
        std::cerr
            << "usage: sumo_external_state_angle_probe NET ROUTES "
            << "LATERAL_SIGN STRICT CYCLES\n";
        return 2;
    }

    const std::string networkPath = argv[1];
    const std::string routesPath = argv[2];
    const double lateralSign = std::stod(argv[3]);
    const bool strictLaneHint = std::string(argv[4]) == "true";
    const int cycles = std::stoi(argv[5]);
    if (lateralSign == 0.0 || cycles <= 0) {
        std::cerr << "LATERAL_SIGN and CYCLES must be non-zero\n";
        return 2;
    }

    try {
        libsumo::Simulation::start(
            simulationOptions(networkPath, routesPath)
        );
        const Candidate pureCandidate = findCandidate(lateralSign);

        std::vector<Snapshot> pureStates;
        std::vector<double> pureTimes;
        pureStates.reserve(cycles);
        pureTimes.reserve(cycles);
        for (int cycle = 0; cycle < cycles; ++cycle) {
            libsumo::Simulation::step();
            pureTimes.push_back(libsumo::Simulation::getTime());
            pureStates.push_back(snapshot(pureCandidate.vehicleID));
        }
        libsumo::Simulation::close();

        // Start from scratch instead of saveState/loadState. SUMO does not
        // serialize all lane-change transient fields, including angleOffset,
        // so a reload would make the comparator start from a different state.
        libsumo::Simulation::start(
            simulationOptions(networkPath, routesPath)
        );
        const Candidate feedbackCandidate = findCandidate(lateralSign);

        std::cout << std::setprecision(17);
        std::cout
            << "{\"record_type\":\"candidate\",\"pure_vehicle_id\":"
            << jsonString(pureCandidate.vehicleID)
            << ",\"feedback_vehicle_id\":"
            << jsonString(feedbackCandidate.vehicleID)
            << ",\"strict_lane_hint\":"
            << (strictLaneHint ? "true" : "false")
            << ",\"requested_lateral_sign\":";
        writeNumber(lateralSign);
        std::cout << ",\"pure_simulation_time\":";
        writeNumber(pureCandidate.simulationTime);
        std::cout << ",\"feedback_simulation_time\":";
        writeNumber(feedbackCandidate.simulationTime);
        std::cout << ",\"pure_position\":{\"x\":";
        writeNumber(pureCandidate.position.x);
        std::cout << ",\"y\":";
        writeNumber(pureCandidate.position.y);
        std::cout << ",\"z\":";
        writeNumber(pureCandidate.position.z);
        std::cout << "},\"feedback_position\":{\"x\":";
        writeNumber(feedbackCandidate.position.x);
        std::cout << ",\"y\":";
        writeNumber(feedbackCandidate.position.y);
        std::cout << ",\"z\":";
        writeNumber(feedbackCandidate.position.z);
        std::cout << "},\"pure_state\":";
        writeSnapshot(pureCandidate.state);
        std::cout << ",\"feedback_state\":";
        writeSnapshot(feedbackCandidate.state);
        std::cout << "}\n";

        const libsumo::TraCIPosition frozenPosition =
            feedbackCandidate.position;
        const std::string frozenLane =
            feedbackCandidate.state.primaryLane;
        const double frozenLanePosition =
            feedbackCandidate.state.lanePosition;
        const double requestedAngle =
            libsumo::Lane::getAngle(frozenLane, frozenLanePosition);

        for (int cycle = 0; cycle < cycles; ++cycle) {
            const double phaseATime = libsumo::Simulation::getTime();
            const Snapshot prePhaseA =
                snapshot(feedbackCandidate.vehicleID);
            const std::string currentRoad =
                libsumo::Vehicle::getRoadID(feedbackCandidate.vehicleID);
            const int currentLaneIndex =
                libsumo::Vehicle::getLaneIndex(feedbackCandidate.vehicleID);

            libsumo::Vehicle::moveToXYImmediate(
                feedbackCandidate.vehicleID,
                currentRoad,
                currentLaneIndex,
                frozenPosition.x,
                frozenPosition.y,
                requestedAngle,
                1,
                10.0,
                strictLaneHint
            );
            libsumo::Vehicle::setSpeed(
                feedbackCandidate.vehicleID, -1.0
            );
            libsumo::Vehicle::setPreviousSpeed(
                feedbackCandidate.vehicleID, 0.0, 0.0
            );

            const libsumo::TraCIPosition observedPosition =
                libsumo::Vehicle::getPosition(
                    feedbackCandidate.vehicleID, true
                );
            const Snapshot phaseA =
                snapshot(feedbackCandidate.vehicleID);
            libsumo::Simulation::step();
            const Snapshot phaseB =
                snapshot(feedbackCandidate.vehicleID);
            const Snapshot& pure = pureStates.at(cycle);

            std::cout
                << "{\"record_type\":\"cycle\",\"cycle\":" << cycle
                << ",\"vehicle_id\":"
                << jsonString(feedbackCandidate.vehicleID)
                << ",\"strict_lane_hint\":"
                << (strictLaneHint ? "true" : "false")
                << ",\"requested_lateral_sign\":";
            writeNumber(lateralSign);
            std::cout << ",\"phase_a_time\":";
            writeNumber(phaseATime);
            std::cout << ",\"phase_b_time\":";
            writeNumber(libsumo::Simulation::getTime());
            std::cout << ",\"pure_phase_b_time\":";
            writeNumber(pureTimes.at(cycle));
            std::cout << ",\"phase_a_requested_x\":";
            writeNumber(frozenPosition.x);
            std::cout << ",\"phase_a_requested_y\":";
            writeNumber(frozenPosition.y);
            std::cout << ",\"phase_a_observed_x\":";
            writeNumber(observedPosition.x);
            std::cout << ",\"phase_a_observed_y\":";
            writeNumber(observedPosition.y);
            std::cout << ",\"phase_a_requested_angle\":";
            writeNumber(requestedAngle);
            std::cout << ",\"phase_a_observed_angle\":";
            writeNumber(phaseA.angle);
            std::cout << ",\"phase_b_angle\":";
            writeNumber(phaseB.angle);
            std::cout << ",\"pure_phase_b_angle\":";
            writeNumber(pure.angle);
            std::cout << ",\"phase_a_b_angle_delta\":";
            writeNumber(angleDifference(phaseA.angle, phaseB.angle));
            std::cout << ",\"phase_b_pure_angle_delta\":";
            writeNumber(angleDifference(phaseB.angle, pure.angle));
            std::cout << ",\"pre_phase_a\":";
            writeSnapshot(prePhaseA);
            std::cout << ",\"phase_a\":";
            writeSnapshot(phaseA);
            std::cout << ",\"phase_b\":";
            writeSnapshot(phaseB);
            std::cout << ",\"pure\":";
            writeSnapshot(pure);
            std::cout << "}\n";
        }

        libsumo::Simulation::close();
        return 0;
    } catch (const std::exception& error) {
        if (libsumo::Simulation::isLoaded()) {
            libsumo::Simulation::close("angle probe failure");
        }
        std::cerr << "angle probe failed: " << error.what() << "\n";
        return 1;
    }
}
