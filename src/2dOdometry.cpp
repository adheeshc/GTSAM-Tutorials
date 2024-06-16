#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <cmath>
#include <iostream>
#include "readFiles.h"


/**
 * Example of a simple 2D localization example
 *  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 *  - The robot moves 2 meters each step
 *  - We have full odometry between poses
 */

using namespace gtsam;

int main(int argc, char** argv) {
    NonlinearFactorGraph graph;
    Pose2 priorMean(0.0, 0.0, 0.0);  // x,y,theta
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 1));
    graph.addPrior(1, priorMean, priorNoise);

    std::vector<Pose2> odometryReadings;
    readOdometryFile("../data/odometry.txt", odometryReadings);
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

    for(size_t i = 0; i<odometryReadings.size(); i++){
        graph.emplace_shared<BetweenFactor<Pose2>>(i+1, i+2, odometryReadings[i], odometryNoise);
    }
    graph.print("\nFactor Graph:\n");

    Values estimates;
    readInitialEstimatesFile("../data/estimates.txt", estimates);
    estimates.print("\nInitial Estimate:\n");

    Values result = LevenbergMarquardtOptimizer(graph, estimates).optimize();
    result.print("Final Result:\n");

    std::vector<double> resultVector;
    for (const auto& key_value : result) {
        Key key = key_value.key;
        try {
            Pose2 pose = result.at<Pose2>(key);
            resultVector.push_back(std::round(pose.x() * 100.0) / 100.0);
            resultVector.push_back(std::round(pose.y() * 100.0) / 100.0);
            resultVector.push_back(std::round(pose.theta() * 100.0) / 100.0);
        } catch (const gtsam::ValuesIncorrectType& e) {
            std::cerr << "Incorrect type for key " << key << ": " << e.what() << "\n";
        }
    }

    for (size_t i = 0; i < resultVector.size(); i += 3) {
        std::cout << "[ ";
        for (size_t j = i; j < i + 3 && j < resultVector.size(); ++j) {
            std::cout << resultVector[j] << " ";
        }
        std::cout << "]\n";
    }

    std::cout.precision(2);
    Marginals marginals(graph, result);
    std::cout << "x1 covariance:\n"
              << marginals.marginalCovariance(1) << std::endl;
    std::cout << "x2 covariance:\n"
              << marginals.marginalCovariance(2) << std::endl;
    std::cout << "x3 covariance:\n"
              << marginals.marginalCovariance(3) << std::endl;
    std::cout << "x4 covariance:\n"
              << marginals.marginalCovariance(4) << std::endl;

    return 0;
}