#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <algorithm>


/**
 * @brief Struct representing a tangent point pair between two circles.
 */
struct TangentPoint {
    double x1, y1; // Point on first circle
    double x2, y2; // Point on second circle
};

/**
 * @brief Computes the internal or external common tangents between two circles of equal radius.
 * 
 * @param cx1 X-coordinate of the first circle's center.
 * @param cy1 Y-coordinate of the first circle's center.
 * @param cx2 X-coordinate of the second circle's center.
 * @param cy2 Y-coordinate of the second circle's center.
 * @param radius Radius of both circles (assumed equal).
 * @param internal True to compute internal tangents, false for external.
 * @return std::vector<TangentPoint> containing up to 2 tangents.
 */
std::vector<TangentPoint> computeCommonTangents(double cx1, double cy1, 
                                                double cx2, double cy2, 
                                                double radius);

std::string getRotationDirection(double x, double y, double cx, double cy, double azimuth);  


std::pair<std::pair<double, double>, std::pair<double, double>> chosenPoint(
    const std::vector<TangentPoint>& tangents, double cx2, double cy2);

// Compute circle center from a point on the circle, radius, and azimuth (in degrees)
std::pair<double, double> computeCircleCenter(double radius);

#endif // UTILS_H
