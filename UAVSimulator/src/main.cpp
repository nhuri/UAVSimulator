#include "Simulation.h"
#include "utils.h"
#include <iostream>

/**
 * @brief Entry point for the UAV simulation program.
 * 
 * Loads simulation parameters and UAV commands from input files,
 * then runs the simulation and generates output files for each UAV.
 */

int main() {
    Simulation sim;

    sim.loadParams("input/SimParams.ini");
    sim.loadCommands("input/SimCmds.txt");
    sim.run();

//     std::vector<TangentPoint> tangents = computeCommonTangents(2, 0, 8, 8, 2);
//     std::pair<std::pair<double, double>, std::pair<double, double>> chosenPoint1 = chosenPoint(tangents, 270, 2, 0, 8, 8);
// for (const auto& t : tangents) {
//     std::cout << "Tangent from (" << t.x1 << ", " << t.y1 << ") to ("
//               << t.x2 << ", " << t.y2 << ")\n";
// }
    // std::cout << "The choosen linear from: (" << chosenPoint1.first.first << "," << chosenPoint1.first.second << ") to ("
    //           << chosenPoint1.second.first << "," << chosenPoint1.second.second << ")";
    return 0;
}
