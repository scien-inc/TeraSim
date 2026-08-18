/****************************************************************************/
// Test-only split probe for field-trigger leakage and an independent right run.
/****************************************************************************/

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <libsumo/Helper.h>
#include <libsumo/Lane.h>
#include <libsumo/Simulation.h>
#include <libsumo/Vehicle.h>
#include <microsim/MSLane.h>
#include <microsim/MSVehicle.h>
#include <microsim/lcmodels/MSAbstractLaneChangeModel.h>

namespace {

constexpr double DEFAULT_FROZEN_YAW = 321.4971618652344;
constexpr double LEFT_TARGET_FRACTION = 1.0;
constexpr int FIELD_LEFT_STEPS = 65;
constexpr double DEFAULT_RIGHT_A1_FRACTION = -0.0866;
constexpr double DEFAULT_RIGHT_A2_FRACTION = 0.1;
constexpr double RIGHT_OUTSIDE_FRACTION = -0.46;
constexpr int RIGHT_OUTSIDE_FROZEN_CYCLES = 60;
constexpr double RIGHT_TARGET_FRACTION = 1.0;

struct Snapshot {
    std::string primaryLane;
    std::string lcmTargetLane;
    std::string shadowLane;
    double positionX;
    double positionY;
    double primaryPosLat;
    double sourcePosLat;
    double targetPosLat;
    double speedLat;
    double reportedSpeedLat;
    double maneuverDistance;
    double previousManeuverDistance;
    double angle;
    double angleOffset;
    int ownState;
    int leftState;
    int rightState;
};

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

std::string
laneID(const MSLane* lane) {
    return lane == nullptr ? "" : lane->getID();
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

libsumo::TraCIPosition
fractionalLanePosition(
    const libsumo::TraCIPositionVector& sourceShape,
    const double sourceLength,
    const libsumo::TraCIPositionVector& targetShape,
    const double targetLength,
    const double laneOffset,
    const double targetFraction
) {
    const libsumo::TraCIPosition source =
        positionAtDeclaredLaneOffset(sourceShape, sourceLength, laneOffset);
    const libsumo::TraCIPosition target =
        positionAtDeclaredLaneOffset(targetShape, targetLength, laneOffset);
    libsumo::TraCIPosition result;
    result.x = source.x + targetFraction * (target.x - source.x);
    result.y = source.y + targetFraction * (target.y - source.y);
    return result;
}

double
navigationAngle(
    const libsumo::TraCIPosition& current,
    const libsumo::TraCIPosition& next
) {
    double angle = std::atan2(
        next.x - current.x,
        next.y - current.y
    ) * 180. / M_PI;
    return angle < 0. ? angle + 360. : angle;
}

double
relativePosLat(MSVehicle& vehicle, const std::string& laneIDValue) {
    const MSLane* lane = MSLane::dictionary(laneIDValue);
    if (lane == nullptr) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double geometrySign = vehicle.getLateralGeometrySign();
    if (!std::isfinite(geometrySign) || geometrySign == 0.) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const Position coordinates =
        lane->getShape().transformToVectorCoordinates(vehicle.getPosition());
    if (coordinates == Position::INVALID
            || !std::isfinite(coordinates.y())) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    return coordinates.y() / geometrySign;
}

Snapshot
snapshot(
    const std::string& vehicleID,
    const std::string& sourceLane,
    const std::string& targetLane
) {
    MSVehicle& vehicle = getVehicle(vehicleID);
    const MSAbstractLaneChangeModel& laneChangeModel =
        vehicle.getLaneChangeModel();
    const std::pair<int, int> leftState =
        libsumo::Vehicle::getLaneChangeState(vehicleID, 1);
    const std::pair<int, int> rightState =
        libsumo::Vehicle::getLaneChangeState(vehicleID, -1);
    const Position& position = vehicle.getPosition();
    return {
        libsumo::Vehicle::getLaneID(vehicleID),
        laneID(laneChangeModel.getTargetLane()),
        laneID(laneChangeModel.getShadowLane()),
        position.x(),
        position.y(),
        libsumo::Vehicle::getLateralLanePosition(vehicleID),
        relativePosLat(vehicle, sourceLane),
        relativePosLat(vehicle, targetLane),
        laneChangeModel.getSpeedLat(),
        libsumo::Vehicle::getLateralSpeed(vehicleID),
        laneChangeModel.getManeuverDist(),
        laneChangeModel.getPreviousManeuverDist(),
        libsumo::Vehicle::getAngle(vehicleID),
        laneChangeModel.getAngleOffset(),
        laneChangeModel.getOwnState(),
        leftState.second,
        rightState.second,
    };
}

void
writeSnapshot(
    const std::string& scenario,
    const std::string& stage,
    const int cycle,
    const std::string& vehicleID,
    const bool strictLaneHint,
    const std::string& sourceLane,
    const std::string& targetLane,
    const std::vector<std::string>& originalRoute
) {
    const Snapshot value = snapshot(vehicleID, sourceLane, targetLane);
    const int maneuverSign = value.maneuverDistance > 0.
                             ? 1
                             : value.maneuverDistance < 0. ? -1 : 0;
    std::cout << std::setprecision(17)
              << "{\"record_type\":\"snapshot\",\"scenario\":"
              << jsonString(scenario)
              << ",\"stage\":"
              << jsonString(stage)
              << ",\"cycle\":" << cycle
              << ",\"simulation_time\":";
    writeNumber(libsumo::Simulation::getTime());
    std::cout << ",\"vehicle_id\":" << jsonString(vehicleID)
              << ",\"strict_lane_hint\":"
              << (strictLaneHint ? "true" : "false")
              << ",\"primary_lane\":" << jsonString(value.primaryLane)
              << ",\"lcm_target_lane\":" << jsonString(value.lcmTargetLane)
              << ",\"shadow_lane\":" << jsonString(value.shadowLane)
              << ",\"source_reference_lane\":" << jsonString(sourceLane)
              << ",\"target_reference_lane\":" << jsonString(targetLane)
              << ",\"position_x\":";
    writeNumber(value.positionX);
    std::cout << ",\"position_y\":";
    writeNumber(value.positionY);
    std::cout << ",\"primary_pos_lat\":";
    writeNumber(value.primaryPosLat);
    std::cout << ",\"source_pos_lat\":";
    writeNumber(value.sourcePosLat);
    std::cout << ",\"target_pos_lat\":";
    writeNumber(value.targetPosLat);
    std::cout << ",\"speed_lat\":";
    writeNumber(value.speedLat);
    std::cout << ",\"reported_speed_lat\":";
    writeNumber(value.reportedSpeedLat);
    std::cout << ",\"maneuver_distance\":";
    writeNumber(value.maneuverDistance);
    std::cout << ",\"previous_maneuver_distance\":";
    writeNumber(value.previousManeuverDistance);
    std::cout << ",\"maneuver_sign\":" << maneuverSign
              << ",\"angle\":";
    writeNumber(value.angle);
    std::cout << ",\"angle_offset_radians\":";
    writeNumber(value.angleOffset);
    std::cout << ",\"angle_offset_degrees\":";
    writeNumber(value.angleOffset * 180. / M_PI);
    std::cout << ",\"own_state\":" << value.ownState
              << ",\"left_state\":" << value.leftState
              << ",\"right_state\":" << value.rightState
              << ",\"route_unchanged\":"
              << (libsumo::Vehicle::getRoute(vehicleID) == originalRoute
                  ? "true" : "false")
              << "}\n";
}

void
applyImmediatePose(
    const std::string& vehicleID,
    const libsumo::TraCIPosition& position,
    const double angle,
    const bool strictLaneHint,
    const int laneIndex
) {
    libsumo::Vehicle::moveToXYImmediate(
        vehicleID,
        "edge_426",
        laneIndex,
        position.x,
        position.y,
        angle,
        1,
        10.,
        strictLaneHint
    );
    libsumo::Vehicle::setSpeed(vehicleID, -1.);
    libsumo::Vehicle::setPreviousSpeed(vehicleID, 0., 0.);
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
    if (argc < 7 || argc > 10) {
        std::cerr
            << "usage: sumo_external_state_right_reversal_probe "
            << "NET LEFT_ROUTES RIGHT_ROUTES VEHICLE_ID STRICT WAIT_CYCLES "
            << "[FROZEN_YAW [RIGHT_A1_FRACTION [RIGHT_A2_FRACTION]]]\n";
        return 2;
    }

    const std::string networkPath = argv[1];
    const std::string leftRoutesPath = argv[2];
    const std::string rightRoutesPath = argv[3];
    const std::string vehicleID = argv[4];
    const bool strictLaneHint = std::string(argv[5]) == "true";
    const int waitCycles = std::stoi(argv[6]);
    const double frozenYaw = argc >= 8
                             ? std::stod(argv[7])
                             : DEFAULT_FROZEN_YAW;
    const double rightA1Fraction = argc >= 9
                                   ? std::stod(argv[8])
                                   : DEFAULT_RIGHT_A1_FRACTION;
    const double rightA2Fraction = argc >= 10
                                   ? std::stod(argv[9])
                                   : DEFAULT_RIGHT_A2_FRACTION;

    if (waitCycles < 1 || waitCycles > 4) {
        std::cerr << "WAIT_CYCLES must be between 1 and 4\n";
        return 2;
    }

    try {
        libsumo::Simulation::start(
            simulationOptions(networkPath, leftRoutesPath));
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
        const double laneOffset =
            libsumo::Vehicle::getLanePosition(vehicleID);

        // Reproduce the field trigger without injecting a discontinuous yaw or
        // a one-cycle center-to-center jump. On the current image, 1 / 65 is
        // the coarsest uniform fraction that leaves a standard right target
        // after three feedback cycles; 1 / 64 leaves the target unset.
        libsumo::Vehicle::changeLane(vehicleID, 1, 10.);
        libsumo::Simulation::executeMove();
        writeSnapshot(
            "field_trigger",
            "left_authorize",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_0",
            "edge_426_1",
            originalRoute
        );

        const libsumo::TraCIPosition lane0Here =
            positionAtDeclaredLaneOffset(
                lane0Shape, lane0Length, laneOffset);
        const libsumo::TraCIPosition lane0Next =
            positionAtDeclaredLaneOffset(
                lane0Shape, lane0Length, laneOffset + 0.1);
        const double leftYaw = navigationAngle(lane0Here, lane0Next);
        for (int fractionStep = 1;
                fractionStep <= FIELD_LEFT_STEPS;
                ++fractionStep) {
            const double fraction = fractionStep
                                    / static_cast<double>(FIELD_LEFT_STEPS);
            const libsumo::TraCIPosition leftPose = fractionalLanePosition(
                lane0Shape,
                lane0Length,
                lane1Shape,
                lane1Length,
                laneOffset,
                fraction
            );
            const int currentLaneIndex =
                libsumo::Vehicle::getLaneIndex(vehicleID);
            applyImmediatePose(
                vehicleID,
                leftPose,
                leftYaw,
                strictLaneHint,
                currentLaneIndex
            );
            writeSnapshot(
                "field_trigger",
                "left_gradual_phase_a",
                fractionStep - 1,
                vehicleID,
                strictLaneHint,
                "edge_426_0",
                "edge_426_1",
                originalRoute
            );
            libsumo::Simulation::step();
            writeSnapshot(
                "field_trigger",
                "left_gradual_phase_b",
                fractionStep - 1,
                vehicleID,
                strictLaneHint,
                "edge_426_0",
                "edge_426_1",
                originalRoute
            );
        }
        require(
            libsumo::Vehicle::getLaneID(vehicleID) == "edge_426_1",
            "field-trigger left completion did not reach edge_426_1"
        );

        const libsumo::TraCIPosition lane1Center =
            positionAtDeclaredLaneOffset(
                lane1Shape, lane1Length, laneOffset);
        const libsumo::TraCIPosition lane1Next =
            positionAtDeclaredLaneOffset(
                lane1Shape, lane1Length, laneOffset + 0.1);
        const double lane1Yaw = navigationAngle(lane1Center, lane1Next);
        for (int cycle = 0; cycle < waitCycles; ++cycle) {
            applyImmediatePose(
                vehicleID, lane1Center, lane1Yaw, strictLaneHint, 1);
            writeSnapshot(
                "field_trigger",
                "wait_phase_a",
                cycle,
                vehicleID,
                strictLaneHint,
                "edge_426_1",
                "edge_426_0",
                originalRoute
            );
            libsumo::Simulation::step();
            writeSnapshot(
                "field_trigger",
                "wait_phase_b",
                cycle,
                vehicleID,
                strictLaneHint,
                "edge_426_1",
                "edge_426_0",
                originalRoute
            );
        }

        // Inject the field failure state after a completed maneuver: no
        // intent/target/shadow and maneuverDist == 0, but stale speedLat remains.
        // Phase A must synchronize this to the frozen external pose without
        // inventing a lane-change decision.
        getVehicle(vehicleID).getLaneChangeModel().setSpeedLat(1.);
        for (int cycle = 0; cycle < 3; ++cycle) {
            writeSnapshot(
                "field_trigger",
                "zero_stale_pre_phase_a",
                cycle,
                vehicleID,
                strictLaneHint,
                "edge_426_1",
                "edge_426_0",
                originalRoute
            );
            applyImmediatePose(
                vehicleID, lane1Center, lane1Yaw, strictLaneHint, 1);
            writeSnapshot(
                "field_trigger",
                "zero_stale_phase_a",
                cycle,
                vehicleID,
                strictLaneHint,
                "edge_426_1",
                "edge_426_0",
                originalRoute
            );
            libsumo::Simulation::step();
            writeSnapshot(
                "field_trigger",
                "zero_stale_phase_b",
                cycle,
                vehicleID,
                strictLaneHint,
                "edge_426_1",
                "edge_426_0",
                originalRoute
            );
        }

        libsumo::Vehicle::changeLane(vehicleID, 0, 10.);
        libsumo::Simulation::executeMove();
        writeSnapshot(
            "field_trigger",
            "right_authorize",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_1",
            "edge_426_0",
            originalRoute
        );
        libsumo::Simulation::step();
        writeSnapshot(
            "field_trigger",
            "right_authorize_phase_b",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_1",
            "edge_426_0",
            originalRoute
        );
        const Snapshot fieldRightAuthorization = snapshot(
            vehicleID, "edge_426_1", "edge_426_0");

        const bool leftRouteUnchanged =
            libsumo::Vehicle::getRoute(vehicleID) == originalRoute;
        libsumo::Simulation::close();

        // Start from a fresh lane-1 departure so the right-request snapshots
        // cannot inherit any lane-change or angle state from the left run.
        libsumo::Simulation::start(
            simulationOptions(networkPath, rightRoutesPath));
        libsumo::Simulation::step();
        require(
            libsumo::Vehicle::getLaneID(vehicleID) == "edge_426_1",
            "fresh-right vehicle did not depart on edge_426_1"
        );
        const std::vector<std::string> rightOriginalRoute =
            libsumo::Vehicle::getRoute(vehicleID);
        const double rightLaneOffset =
            libsumo::Vehicle::getLanePosition(vehicleID);

        libsumo::Vehicle::changeLane(vehicleID, 0, 10.);
        libsumo::Simulation::executeMove();
        writeSnapshot(
            "fresh_right",
            "right_authorize",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_1",
            "edge_426_0",
            rightOriginalRoute
        );
        const Snapshot rightAuthorization = snapshot(
            vehicleID, "edge_426_1", "edge_426_0");
        require(
            rightAuthorization.maneuverDistance < 0.,
            "fresh right authorization did not create a negative maneuver"
        );
        require(
            rightAuthorization.lcmTargetLane == "edge_426_0",
            "fresh right authorization did not target edge_426_0"
        );

        const libsumo::TraCIPosition rightA1Pose = fractionalLanePosition(
            lane1Shape,
            lane1Length,
            lane0Shape,
            lane0Length,
            rightLaneOffset,
            rightA1Fraction
        );
        applyImmediatePose(
            vehicleID, rightA1Pose, frozenYaw, strictLaneHint, 1);
        writeSnapshot(
            "fresh_right",
            "right_a1",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_1",
            "edge_426_0",
            rightOriginalRoute
        );
        libsumo::Simulation::step();
        writeSnapshot(
            "fresh_right",
            "right_b1",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_1",
            "edge_426_0",
            rightOriginalRoute
        );

        const libsumo::TraCIPosition rightA2Pose = fractionalLanePosition(
            lane1Shape,
            lane1Length,
            lane0Shape,
            lane0Length,
            rightLaneOffset,
            rightA2Fraction
        );
        applyImmediatePose(
            vehicleID, rightA2Pose, frozenYaw, strictLaneHint, 1);
        writeSnapshot(
            "fresh_right",
            "right_a2",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_1",
            "edge_426_0",
            rightOriginalRoute
        );
        libsumo::Simulation::step();
        writeSnapshot(
            "fresh_right",
            "right_b2",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_1",
            "edge_426_0",
            rightOriginalRoute
        );

        applyImmediatePose(
            vehicleID, rightA2Pose, frozenYaw, strictLaneHint, 1);
        writeSnapshot(
            "fresh_right",
            "right_frozen_phase_a",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_1",
            "edge_426_0",
            rightOriginalRoute
        );
        libsumo::Simulation::step();
        writeSnapshot(
            "fresh_right",
            "right_frozen_phase_b",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_1",
            "edge_426_0",
            rightOriginalRoute
        );

        // Cross just beyond the source-lane half-width, but remain closest to
        // lane 1 under the standard hint. Repeating this exact physical pose
        // exposes virtual maneuver consumption and A/B angle accumulation.
        const libsumo::TraCIPosition rightOutsidePose =
            fractionalLanePosition(
                lane1Shape,
                lane1Length,
                lane0Shape,
                lane0Length,
                rightLaneOffset,
                RIGHT_OUTSIDE_FRACTION
            );
        for (int cycle = 0;
                cycle < RIGHT_OUTSIDE_FROZEN_CYCLES;
                ++cycle) {
            applyImmediatePose(
                vehicleID,
                rightOutsidePose,
                frozenYaw,
                strictLaneHint,
                1
            );
            writeSnapshot(
                "fresh_right",
                "right_outside_phase_a",
                cycle,
                vehicleID,
                strictLaneHint,
                "edge_426_1",
                "edge_426_0",
                rightOriginalRoute
            );
            libsumo::Simulation::step();
            writeSnapshot(
                "fresh_right",
                "right_outside_phase_b",
                cycle,
                vehicleID,
                strictLaneHint,
                "edge_426_1",
                "edge_426_0",
                rightOriginalRoute
            );
        }

        const libsumo::TraCIPosition rightTargetPose =
            fractionalLanePosition(
                lane1Shape,
                lane1Length,
                lane0Shape,
                lane0Length,
                rightLaneOffset,
                RIGHT_TARGET_FRACTION
            );
        applyImmediatePose(
            vehicleID, rightTargetPose, frozenYaw, strictLaneHint, 1);
        writeSnapshot(
            "fresh_right",
            "right_target_phase_a",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_1",
            "edge_426_0",
            rightOriginalRoute
        );
        libsumo::Simulation::step();
        writeSnapshot(
            "fresh_right",
            "right_target_phase_b",
            0,
            vehicleID,
            strictLaneHint,
            "edge_426_1",
            "edge_426_0",
            rightOriginalRoute
        );
        const Snapshot rightFinal = snapshot(
            vehicleID, "edge_426_1", "edge_426_0");
        const bool rightRouteUnchanged =
            libsumo::Vehicle::getRoute(vehicleID) == rightOriginalRoute;

        std::cout << std::setprecision(17)
                  << "RESULT_JSON={\"vehicle_id\":"
                  << jsonString(vehicleID)
                  << ",\"strict_lane_hint\":"
                  << (strictLaneHint ? "true" : "false")
                  << ",\"wait_cycles\":" << waitCycles
                  << ",\"frozen_yaw\":";
        writeNumber(frozenYaw);
        std::cout << ",\"field_left_steps\":" << FIELD_LEFT_STEPS
                  << ",\"field_left_step_fraction\":";
        writeNumber(1. / static_cast<double>(FIELD_LEFT_STEPS));
        std::cout << ",\"left_target_fraction\":";
        writeNumber(LEFT_TARGET_FRACTION);
        std::cout << ",\"right_a1_fraction\":";
        writeNumber(rightA1Fraction);
        std::cout << ",\"right_a2_fraction\":";
        writeNumber(rightA2Fraction);
        std::cout << ",\"right_outside_fraction\":";
        writeNumber(RIGHT_OUTSIDE_FRACTION);
        std::cout << ",\"outside_cycles\":"
                  << RIGHT_OUTSIDE_FROZEN_CYCLES
                  << ",\"right_target_fraction\":";
        writeNumber(RIGHT_TARGET_FRACTION);
        std::cout << ",\"field_right_primary_lane\":"
                  << jsonString(fieldRightAuthorization.primaryLane)
                  << ",\"field_right_lcm_target_lane\":"
                  << jsonString(fieldRightAuthorization.lcmTargetLane)
                  << ",\"field_right_maneuver_distance\":";
        writeNumber(fieldRightAuthorization.maneuverDistance);
        std::cout << ",\"left_route_unchanged\":"
                  << (leftRouteUnchanged ? "true" : "false")
                  << ",\"right_route_unchanged\":"
                  << (rightRouteUnchanged ? "true" : "false")
                  << ",\"route_unchanged\":"
                  << (rightRouteUnchanged ? "true" : "false")
                  << ",\"right_final_primary_lane\":"
                  << jsonString(rightFinal.primaryLane)
                  << ",\"right_final_lcm_target_lane\":"
                  << jsonString(rightFinal.lcmTargetLane)
                  << ",\"right_final_target_pos_lat\":";
        writeNumber(rightFinal.targetPosLat);
        std::cout << ",\"right_final_maneuver_distance\":";
        writeNumber(rightFinal.maneuverDistance);
        std::cout << ",\"right_final_angle\":";
        writeNumber(rightFinal.angle);
        std::cout
                  << "}\n";

        libsumo::Simulation::close();
        return 0;
    } catch (const std::exception& error) {
        if (libsumo::Simulation::isLoaded()) {
            libsumo::Simulation::close("right-reversal probe failure");
        }
        std::cerr << "right-reversal probe failed: " << error.what() << "\n";
        return 1;
    }
}
