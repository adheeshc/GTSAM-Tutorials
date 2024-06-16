#pragma once

#ifndef READ_FILES_H
#define READ_FILES_H

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

void readOdometryFile(const std::string& filename, std::vector<gtsam::Pose2>& odometryReadings) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double x, y, theta;
        if (ss >> x >> y >> theta) {
            odometryReadings.emplace_back(x, y, theta);
        } else {
            std::cerr << "Error reading line: " << line << std::endl;
        }
    }
    file.close();
}

void readGPSFile(const std::string& filename, std::vector<std::pair<double, double>>& GPSReadings) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double x, y;
        if (ss >> x >> y) {
            GPSReadings.emplace_back(x, y);
        } else {
            std::cerr << "Error reading line: " << line << std::endl;
        }
    }
    file.close();
}

void readInitialEstimatesFile(const std::string& filename, std::vector<gtsam::Pose2>& initialEstimates) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double x, y, theta;
        if (ss >> x >> y >> theta) {
            initialEstimates.emplace_back(x, y, theta);
        } else {
            std::cerr << "Error reading line: " << line << std::endl;
        }
    }

    file.close();
}

#endif