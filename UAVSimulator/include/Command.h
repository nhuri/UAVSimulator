#pragma once
#include <vector>
#include <string>

struct Command { //struct representing a single command.
    double time;
    int uavId;
    double targetX;
    double targetY;
/**
 * @brief Constructor for the Command struct.
 * @param time     Time when the command should be executed.
 * @param uavId    ID of the UAV the command is assigned to.
 * @param targetX  X coordinate of the target location.
 * @param targetY  Y coordinate of the target location.
 */
    Command(double time, int uavId, double targetX, double targetY)
        : time(time), uavId(uavId), targetX(targetX), targetY(targetY) {}
};
