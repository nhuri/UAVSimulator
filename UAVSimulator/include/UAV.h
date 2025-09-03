#pragma once

#include <string>

/**
 * @brief Enumeration representing the UAV flight state.
 */
enum class UAVState {
    FLYING,     ///< UAV is flying towards a target.
    CIRCLING    ///< UAV is circling around the target.
};

/**
 * @brief Class representing a single UAV (Unmanned Aerial Vehicle).
 */
class UAV {
public:
    /**
     * @brief Constructor to initialize a UAV instance.
     * 
     * @param id UAV ID (unique number).
     * @param x Initial X coordinate.
     * @param y Initial Y coordinate.
     * @param v0 Constant flight speed [m/s].
     * @param azimuthDeg Initial heading angle in degrees (0 = X axis).
     * @param radius Minimum turning radius [m].
     */
    UAV(int id, double x, double y, double v0, double azimuthDeg, double radius);

    /**
     * @brief Update UAV position and orientation for a simulation step.
     * 
     * @param dt Time step in seconds.
     */
    void update(double dt);

    /**
     * @brief Assign a new target point to the UAV.
     * 
     * @param x Target X coordinate.
     * @param y Target Y coordinate.
     */
    void setTarget(double x, double y);

    /**
     * @brief Check whether UAV has reached its current target.
     * 
     * @return true if within minimum turning radius of the target.
     * @return false otherwise.
     */
    bool hasReachedTarget() const;

    /**
     * @brief Write current UAV state to output file (append).
     * 
     * @param time Current simulation time.
     * @param filepath Output file path.
     */
    void writeStateToFile(double time, const std::string& filepath) const;

    double normalizeAngleDeg(double angle);

    /**
     * @brief Get the UAV's unique ID.
     * 
     * @return int UAV ID.
     */
    int getId() const;

private:
    int id;             ///< UAV identifier
    double x, y;        ///< Current position
    double v;           ///< Constant speed
    double azimuth;     ///< Heading angle [degrees]
    double r_min;       ///< Minimum turning radius

    double targetX, targetY; ///< Target coordinates
    bool hasTarget;          ///< Flag: target assigned or not

    UAVState state;     ///< Current UAV state (FLYING or CIRCLING)

    // For circling
    double circleCenterX;
    double circleCenterY;
    double circleAngle; ///< Current angle around the circle [radians]
};
