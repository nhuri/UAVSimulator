#include "Utils.h"
#include <cmath>
#include <string>
#include "Simulation.h"
#include <iostream>
#include "Command.h"
/**
 * @brief Normalizes an angle to the range [0, 360).
 *
 * @param angle Angle in degrees
 * @return Normalized angle in [0, 360)
 */
double normalizeAngleDeg(double angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

/**
 * @brief Determines whether a vehicle is rotating clockwise or counter-clockwise around a circle.
 *
 * @param x        Current x-position of the vehicle
 * @param y        Current y-position of the vehicle
 * @param cx       x-coordinate of the circle center
 * @param cy       y-coordinate of the circle center
 * @param azimuth  Current azimuth (heading) of the vehicle, in degrees
 * @return         "Counter-Clockwise (CCW)" or "Clockwise (CW)"
 */
std::string getRotationDirection(double x, double y, double cx, double cy, double azimuth) {
    // Radius vector from center to vehicle
    double rx = x - cx;
    double ry = y - cy;

    // Angle of the radius vector relative to X-axis
    double radiusAngle = std::atan2(ry, rx) * 180.0 / 3.14159265358979323846;
    radiusAngle = normalizeAngleDeg(radiusAngle);

    // Difference between azimuth and radius angle
    double diff = azimuth - radiusAngle;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;

    if (diff > 0) {
        return "Counter-Clockwise (CCW)";
    } else {
        return "Clockwise (CW)";
    }
}

/**
 * @brief Computes all common tangents (external and internal) between two equal-radius circles.
 *
 * @param cx1     Center x-coordinate of the first circle
 * @param cy1     Center y-coordinate of the first circle
 * @param cx2     Center x-coordinate of the second circle
 * @param cy2     Center y-coordinate of the second circle
 * @param radius  Radius of both circles
 * @return        Vector of tangent lines, each represented by two points (one on each circle)
 */

// std::vector<TangentPoint> computeCommonTangents(double cx1, double cy1,
//                                                   double cx2, double cy2,
//                                                   double radius) {
//     std::vector<TangentPoint> tangents;
    
//     static bool alreadyPrinted = false;

//     double dx = cx2 - cx1;
//     double dy = cy2 - cy1;
//     double distSq = dx * dx + dy * dy;
//     double dist = std::sqrt(distSq);

//     if (dist < 1e-6) {
//         // Circles are coincident — no valid tangents
//         return tangents;
//     }

//     double angleBetweenCenters = std::atan2(dy, dx);

//     // --- External tangents ---
//     // Angle offset for external tangents
//     double offsetExternal = std::acos(radius / dist);

//     for (int sign : {-1, 1}) {
//         double theta = angleBetweenCenters + sign * offsetExternal;

//         double x1 = cx1 + radius * std::sin(theta);
//         double y1 = cy1 - radius * std::cos(theta);

//         double x2 = cx2 + radius * std::sin(theta);
//         double y2 = cy2 - radius * std::cos(theta);

//         tangents.push_back({x1, y1, x2, y2});
//     }

//     // --- Internal tangents ---
//     // Internal tangents exist only if distance > 2 * radius
//     if (dist > 2 * radius) {
//         double offsetInternal = std::acos(2 * radius / dist);

//         for (int sign : {-1, 1}) {
//             double theta = angleBetweenCenters + sign * offsetInternal;

//             double x1 = cx1 + radius * std::sin(theta);
//             double y1 = cy1 - radius * std::cos(theta);

//             // The second point has opposite offset
//             double x2 = cx2 - radius * std::sin(theta);
//             double y2 = cy2 + radius * std::cos(theta);

//             tangents.push_back({x1, y1, x2, y2});
//         }
//     }
//     return tangents;
// }

std::vector<TangentPoint> computeCommonTangents(double cx1, double cy1,
                                               double cx2, double cy2,
                                               double radius) {
    std::vector<TangentPoint> tangents;

    double dx = cx2 - cx1;
    double dy = cy2 - cy1;
    double dist = std::sqrt(dx*dx + dy*dy);

    if (dist < 1e-6) {
        // Circles coincident - no tangents
        return tangents;
    }

    double angleBetweenCenters = std::atan2(dy, dx);

    // External tangents
    double offsetExternal = std::acos(radius / dist);

    for (int sign : {-1, 1}) {
        double theta = angleBetweenCenters + sign * offsetExternal;

        double x1 = cx1 + radius * std::cos(theta);
        double y1 = cy1 + radius * std::sin(theta);

        double x2 = cx2 + radius * std::cos(theta);
        double y2 = cy2 + radius * std::sin(theta);

        tangents.push_back({x1, y1, x2, y2});
    }

    // Internal tangents - only if dist > 2*radius
    if (dist > 2 * radius) {
        double offsetInternal = std::acos(2 * radius / dist);

        for (int sign : {-1, 1}) {
            double theta = angleBetweenCenters + sign * offsetInternal;

            double x1 = cx1 + radius * std::cos(theta);
            double y1 = cy1 + radius * std::sin(theta);

            double x2 = cx2 - radius * std::cos(theta);
            double y2 = cy2 - radius * std::sin(theta);

            tangents.push_back({x1, y1, x2, y2});
        }
    }

    return tangents;
}
 
    double startX;
    double startY;
    double azimuth0;

// Computes the center of the turn circle based on the turning direction,
// which is determined internally using the initial UAV state from SimParams.ini.
std::pair<double, double> computeCircleCenter(double radius, double targetX, double targetY) {
    const double pi = 3.14159265358979323846;

    Simulation sim;
    sim.loadParams("input/SimParams.ini");

    double startX = sim.getX0();
    double startY = sim.getY0();
    double azimuth0 = sim.getAzimuth0();

    double azimuthRad = azimuth0 * pi / 180.0;
    double perpendicularRad;

    // Determine turning direction: Clockwise or Counter-clockwise
    if (getRotationDirection(startX, startY, targetX, targetY, azimuth0) == "Clockwise (CW)") {
        perpendicularRad = azimuthRad - pi / 2;
    } else {
        perpendicularRad = azimuthRad + pi / 2;
    }

    double dx = radius * std::cos(perpendicularRad);
    double dy = radius * std::sin(perpendicularRad);

    double center_x = startX + dx;
    double center_y = startY + dy;

    return {center_x, center_y};
}


    std::pair<std::pair<double, double>, std::pair<double, double>> chosenPoint(
    const std::vector<TangentPoint>& tangents, double cx2, double cy2)
{
    for (const auto& t : tangents) {
        std::string rotation = getRotationDirection(startX, startY, cx2, cy2, azimuth0);

        // Option 1: max x1 and max x2
        if (rotation == "Clockwise (CW)" && startY > cy2) {
            const auto& best = *std::max_element(tangents.begin(), tangents.end(), [](const TangentPoint& a, const TangentPoint& b) {
                return (a.x1 + a.x2) < (b.x1 + b.x2);
            });
            return { {best.x1, best.y1}, {best.x2, best.y2} };
        }

        // Option 2: min x1 and max x2
        if (rotation == "Counter-Clockwise (CCW)" && startY > cy2) {
            const auto& best = *std::max_element(tangents.begin(), tangents.end(), [](const TangentPoint& a, const TangentPoint& b) {
                return (a.x2 - a.x1) < (b.x2 - b.x1);
            });
            return { {best.x1, best.y1}, {best.x2, best.y2} };
        }

        // Option 3: min x1 and min x2
        if (rotation == "Clockwise (CW)" && startY <= cy2) {
            const auto& best = *std::min_element(tangents.begin(), tangents.end(), [](const TangentPoint& a, const TangentPoint& b) {
                return (a.x1 + a.x2) < (b.x1 + b.x2);
            });
            return { {best.x1, best.y1}, {best.x2, best.y2} };
        }

        // Option 4: max x1 and min x2
        if (rotation == "Counter-Clockwise (CCW)" && startY <= cy2) {
            const auto& best = *std::max_element(tangents.begin(), tangents.end(), [](const TangentPoint& a, const TangentPoint& b) {
                return (a.x1 - a.x2) < (b.x1 - b.x2);
            });
            return { {best.x1, best.y1}, {best.x2, best.y2} };
        }
    }

    // Default fallback
    return { {tangents[0].x1, tangents[0].y1}, {tangents[0].x2, tangents[0].y2} };
}

