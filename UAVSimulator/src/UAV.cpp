#include "UAV.h"
#include "Utils.h"
#include "utils.cpp"
#include <cmath>
#include <limits>
#include <fstream> //read and write to file
#include <iomanip> //design output
#include <iostream>

constexpr double PI = 3.14159265358979323846; // constant number
constexpr double DEG2RAD = PI / 180.0;
constexpr double RAD2DEG = 180.0 / PI;

struct Point {
    double x, y;
};

/**
 * @brief Constructor for the UAV class. Initializes UAV with position, velocity, heading, and radius.
 * 
 * @param id UAV identifier.
 * @param x Initial x-coordinate.
 * @param y Initial y-coordinate.
 * @param v0 Initial speed.
 * @param azimuthDeg Initial heading angle in degrees.
 * @param radius Minimum turning radius.
 */

UAV::UAV(int id, double x, double y, double v0, double azimuthDeg, double radius)
    : id(id), x(x), y(y), v(v0), azimuth(azimuthDeg), r_min(radius),
      hasTarget(false), state(UAVState::FLYING),
      circleCenterX(0), circleCenterY(0), circleAngle(0) {}

/**
 * @brief Sets a new target destination for the UAV.
 * 
 * @param tx Target x-coordinate.
 * @param ty Target y-coordinate.
 */

void UAV::setTarget(double tx, double ty) {
    targetX = tx;
    targetY = ty;
    hasTarget = true;
    state = UAVState::FLYING;
}

/**
 * @brief Checks if the UAV has reached its current target.
 * 
 * @return true if the UAV is within r_min distance from the target, false otherwise.
 */

bool UAV::hasReachedTarget() const {
    if (!hasTarget) return false;
    double dx = x - targetX;
    double dy = y - targetY;
    double dist = std::sqrt(dx * dx + dy * dy);
    return dist < r_min;
}
 
/**
 * @brief Normalizes an angle in degrees to the range [0, 360).
 * 
 * This function takes any angle value in degrees and wraps it around
 * so that the result is always within the interval [0, 360) degrees.
 * 
 * @param angle The input angle in degrees (can be any real number).
 * @return The normalized angle in degrees, guaranteed to be >= 0 and < 360.
 */
// double normalizeAngleDeg(double angle) {
//     while (angle < 0) angle += 360.0;
//     while (angle >= 360.0) angle -= 360.0;
//     return angle;
// }

/**
 * @brief Updates the UAV's state based on its current mode and target.
 * 
 * When the UAV is in FLYING state and has a target, this function samples 20 points 
 * on the circumference of a circle with radius r_min around the target point. 
 * It selects the "best" point to approach based on minimizing heading adjustments.
 * 
 * Once the UAV reaches this selected point, it switches to CIRCLING state and
 * circles around the target point.
 * 
 * @param dt Time step to update by (in seconds).
 */
const double epsilon = 1e-6;

double normalize_angle(double degrees) {
    double result = fmod(degrees, 360.0);
    if (result < 0) result += 360.0;  // negative drgrees
    return result;
}


double to_radians(double degrees) {
    return degrees * PI / 180.0;
}

// sin on degrees
double sin_deg(double degrees) {
    return std::sin(to_radians(degrees));
}

// cos on degrees
double cos_deg(double degrees) {
    return std::cos(to_radians(degrees));
}

// tan on degrees
double tan_deg(double degrees) {
    return std::tan(to_radians(degrees));
}

/**
 * @brief Updates the UAV's position and orientation based on motion along a Dubins path.
 *
 * The function generates a smooth trajectory from the UAV's current position to a target position (targetX, targetY),
 * while obeying a minimum turning radius constraint (r_min). The path consists of:
 *   1. An initial circular arc (turn-in) to align the UAV's azimuth with a tangent leading to the goal.
 *   2. A straight-line segment tangent to both start and goal circles.
 *   3. A final circular arc around the target to smoothly enter the goal direction.
 *
 * The motion is updated based on a small time step `dt`, and assumes constant velocity `v`.
 * 
 * @param dt Time step for this update in seconds.
 */
void UAV::update(double dt) {
    // Step 1: Compute center of initial turning circle
    std::pair<double, double> circleCenter = computeCircleCenter(r_min, targetX, targetY);
    double cx1 = circleCenter.first;
    double cy1 = circleCenter.second;
    if (std::abs(cy1) < 1e-10) cy1 = 0.0;

    // Step 2: Compute external tangents between the two circles (start and goal)
    std::vector<TangentPoint> tangents = computeCommonTangents(cx1, cy1, targetX, targetY, r_min);

    // Step 3: Choose appropriate tangent and extract points
    std::pair<double, double> circleToLinearPoint = (chosenPoint(tangents, targetX, targetY)).first;
    std::pair<double, double> linearToCirclePoint = (chosenPoint(tangents, targetX, targetY)).second;

    // Step 4: Calculate the slope and angle of the straight-line segment
    double lineSlope = (circleToLinearPoint.second - linearToCirclePoint.second) /
                       (circleToLinearPoint.first - linearToCirclePoint.first);
    double advancedLength = v * dt;
    double lineAngle = std::atan(lineSlope); // in radians

    // Convert angle to degrees
    double angleDeg = lineAngle * 180.0 / PI;
    if (targetY < cy1) {
        angleDeg = -angleDeg;
    }

    // Step 5: Determine direction of initial circular turn (CW or CCW)
    int isClockwiseStart;
    std::string str = getRotationDirection(circleToLinearPoint.first, circleToLinearPoint.second, cx1, cy1, angleDeg);
    if (str == "Counter-Clockwise (CCW)") {
        isClockwiseStart = 1;
    } else if (str == "Clockwise (CW)") {
        isClockwiseStart = -1;
    }

    // Step 6: Projected position if continuing in straight line
    double xProjected = x + advancedLength * cos_deg(lineAngle);
    double yProjected = y + advancedLength * sin_deg(lineAngle);

    // Step 7: Movement phase handling (initial arc, straight line, final arc)
    static int flag;

    // --- Phase 1: Initial turning arc to align azimuth ---
    if (std::abs(x) < std::abs(circleToLinearPoint.first) && flag != 1) {
        flag = 0;

        double arcLength = v * dt;
        double delta_theta_rad = arcLength / r_min;
        double dTheta = delta_theta_rad * 180.0 / PI;
        double newTheta = azimuth + isClockwiseStart * dTheta;

        this->x = x + arcLength * cos_deg(newTheta);
        this->y = y + arcLength * sin_deg(newTheta);
        newTheta = normalize_angle(newTheta);
        this->azimuth = newTheta;
    }

    // --- Phase 2: Straight line movement between the arcs ---
    else if (std::abs(x) > std::abs(circleToLinearPoint.first) &&
             std::abs(x) <= std::abs(linearToCirclePoint.first) && flag != 1) {
        flag = 0;
        this->x = x + advancedLength * cos_deg(azimuth);
        this->y = y + advancedLength * sin_deg(azimuth);
    }

    // --- Phase 3: Final turning arc around the target ---
    else {
        flag = 1;
        double arcLength = v * dt;
        double delta_theta_rad = arcLength / r_min;
        double dTheta = delta_theta_rad * 180.0 / PI;

        // The final circle is always assumed to turn clockwise
        double newTheta = azimuth - dTheta;
        newTheta = normalize_angle(newTheta);
        this->azimuth = newTheta;

        this->x = x + arcLength * cos_deg(azimuth);
        this->y = y + arcLength * sin_deg(azimuth);
    }
}


/**
 * @brief Appends the UAV's current state (time, position, azimuth) to a file.
 * 
 * @param time Current simulation time.
 * @param filepath Path to the output file.
 */

void UAV::writeStateToFile(double time, const std::string& filepath) const {
    std::ofstream file(filepath, std::ios::app);
    file << std::fixed << std::setprecision(2)
         << time << " " << x << " " << y << " " << azimuth << "\n";
}

/**
 * @brief Retrieves the UAV's unique identifier.
 * 
 * @return UAV ID as an integer.
 */

int UAV::getId() const {
    return id;
}