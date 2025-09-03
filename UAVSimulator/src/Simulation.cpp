#include "Simulation.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <iomanip>

Simulation::Simulation()
    : dt(0.01), timeLimit(30.0), r_min(100.0), v0(30.0), x0(0.0), y0(0.0), azimuth0(0.0) {} // Default constructor initializing simulation parameters with default values

/**
 * @brief Load simulation parameters from a file and initialize UAVs.
 * 
 * This function reads a parameters file where each line contains a key-value pair
 * separated by '='. It extracts simulation parameters such as time step, number of UAVs,
 * initial positions, velocities, azimuth, and simulation time limit. After loading,
 * it creates and initializes the specified number of UAVs with the loaded parameters.
 * 
 * @param filename The path to the parameters file.
 */

void Simulation::loadParams(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) { // // Print an error message if the file wasn't opened successfully.
        std::cerr << "Failed to open params file: " << filename << std::endl;
        return;
    }

    std::string line;
    int n_uav = 0;
    double x0 = 0.0, y0 = 0.0, z0 = 0.0;

while (std::getline(file, line)) { // Read each line from the parameter file
    std::istringstream iss(line);  // Create a stream to parse the current line
    std::string key;
    
    if (std::getline(iss, key, '=')) { // Extract the key (before '=')
        std::string valueStr;
        
        if (std::getline(iss, valueStr)) { // Extract the value (after '=')
            double value = std::stod(valueStr); // Convert the value string to a double
            
            // Match the key to known parameters and assign the value
            if (key.find("Dt") != std::string::npos)
                dt = value; // Time step of the simulation

            else if (key.find("N_uav") != std::string::npos)
                n_uav = static_cast<int>(value); // Number of UAVs

            else if (key.find("R") != std::string::npos)
                r_min = value; // Minimum turning radius for UAVs

            else if (key.find("X0") != std::string::npos)
                x0 = value; // Initial X position for all UAVs

            else if (key.find("Y0") != std::string::npos)
                y0 = value; // Initial Y position for all UAVs

            else if (key.find("Z0") != std::string::npos)
                z0 = value; // Initial Z (altitude), unused in 2D simulation

            else if (key.find("V0") != std::string::npos)
                v0 = value; // Initial velocity for all UAVs

            else if (key.find("Az") != std::string::npos)
                azimuth0 = value; // Initial heading (azimuth) in degrees

            else if (key.find("TimeLim") != std::string::npos)
                timeLimit = value; // Total duration of the simulation
        }
    }
}


    for (int i = 0; i < n_uav; ++i) {
        uavs.emplace_back(i, x0, y0, v0, azimuth0, r_min); // // Construct a new UAV and add it to the UAV list
    }
}

/**
 * @brief Load UAV commands from a file and sort them by time.
 * 
 * This function reads command data from a file, where each command consists of a time,
 * UAV ID, and target coordinates (x, y). It appends each command to the internal commands
 * list and sorts the list by the command time in ascending order.
 * 
 * @param filename The path to the commands file.
 */

void Simulation::loadCommands(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Failed to open commands file: " << filename << std::endl;
        return;
    }

    double time, x, y;
    int id;

    while (file >> time >> id >> x >> y) {
        commands.push_back({time, id, x, y});
    } // Read commands from file until no more data, and add each command to the commands list.

    std::sort(commands.begin(), commands.end(),
              [](const Command& a, const Command& b) { return a.time < b.time; });
}

/**
 * @brief Apply all UAV commands scheduled up to the current simulation time.
 * 
 * This function iterates through the list of commands in chronological order and 
 * applies each command whose time is less than or equal to the current simulation time.
 * Each command updates the target destination of the specified UAV.
 * 
 * The function maintains a static index to avoid re-checking previously processed commands
 * in subsequent calls.
 * 
 * @param currentTime The current time in the simulation.
 */

void Simulation::applyCommands(double currentTime) {
    static size_t index = 0; // index is a pointer for the commands list.

    while (index < commands.size() && commands[index].time <= currentTime) {
        const Command& cmd = commands[index];
        if (cmd.uavId >= 0 && cmd.uavId < uavs.size()) {
            uavs[cmd.uavId].setTarget(cmd.targetX, cmd.targetY);
        } // Update the UAV's destination according to the current command.
        ++index;
    }
}

/**
 * @brief Run the main simulation loop.
 * 
 * This function advances the simulation in time steps of size `dt`, up to the configured time limit.
 * At each time step, it applies all relevant UAV commands, updates the state of each UAV,
 * and writes their current position and heading to their respective output files.
 */

void Simulation::run() {
    double currentTime = 0.0;

    while (currentTime <= timeLimit) {
        applyCommands(currentTime); // Apply all commands scheduled up to the current time.
        

        for (auto& uav : uavs) {
            std::string filename = "output/UAV" + std::to_string(uav.getId()) + ".txt";
            uav.writeStateToFile(currentTime, filename); // Append current state to the UAV's output file.
            uav.update(dt); // Update UAV position and heading based on current state and target.

         //   std::string filename = "output/UAV" + std::to_string(uav.getId()) + ".txt";
        //    uav.writeStateToFile(currentTime, filename); // Append current state to the UAV's output file.
        }
        currentTime += dt; // Advance simulation time.
    }
}