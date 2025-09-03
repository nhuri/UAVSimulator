#pragma once

#include <vector>
#include <string>
#include "UAV.h"
#include "Command.h"

class Simulation {
public:
    Simulation();
    double getX0() const { return x0; }
    double getY0() const { return y0; }
    double getAzimuth0() const { return azimuth0; }

    void loadParams(const std::string& filename);
    void loadCommands(const std::string& filename);
    void run();

private:
    double dt;
    double timeLimit;
    double r_min;
    double x0;
    double y0;
    double v0;
    double azimuth0;

    std::vector<UAV> uavs;
    std::vector<Command> commands;

    void applyCommands(double currentTime);
};
