#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include "readFiles.h"

using namespace gtsam;

int main(int argc, char** argv) {
    NonlinearFactorGraph graph;
    Pose2 priorMean(0.0, 0.0, 0.0);  // x,y,theta
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 1));
    graph.addPrior(1, priorMean, priorNoise);

    // Add odometry readings
    std::vector<Pose2> odometryReadings;
    readOdometryFile("../data/odometry.txt", odometryReadings);
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    for (size_t i = 0; i < odometryReadings.size(); i++) {
        graph.emplace_shared<BetweenFactor<Pose2>>(i + 1, i + 2, odometryReadings[i], odometryNoise);
    }

    // Add Loop Closure constraint
    noiseModel::Diagonal::shared_ptr loopClosureNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    graph.emplace_shared<BetweenFactor<Pose2>>(odometryReadings.size() + 1, 2, Pose2(2, 0, M_PI_2), loopClosureNoise);

    graph.print("\nFactor Graph:\n");

    // Add initial estimates
    std::vector<Pose2> estimateReadings;
    readInitialEstimatesFile("../data/estimates.txt", estimateReadings);
    Values initialEstimate;
    for (size_t i = 0; i < estimateReadings.size(); i++) {
        initialEstimate.insert(i + 1, Pose2(estimateReadings[i]));
    }
    initialEstimate.print("\nInitial Estimate:\n");

    // Optimize using a Gauss-Newton nonlinear optimizer
    GaussNewtonParams parameters;
    parameters.relativeErrorTol = 1e-5;
    parameters.maxIterations = 100;
    GaussNewtonOptimizer optimizer(graph, initialEstimate, parameters);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");

    // PRINT OUTPUT
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

    // Calculate and print marginal covariances for all variables
    // std::cout.precision(2);
    // Marginals marginals(graph, result);
    // std::cout << "x1 covariance:\n"
    //           << marginals.marginalCovariance(1) << std::endl;
    // std::cout << "x2 covariance:\n"
    //           << marginals.marginalCovariance(2) << std::endl;
    // std::cout << "x3 covariance:\n"
    //           << marginals.marginalCovariance(3) << std::endl;
    // std::cout << "x4 covariance:\n"
    //           << marginals.marginalCovariance(4) << std::endl;
    // std::cout << "x5 covariance:\n"
    //           << marginals.marginalCovariance(5) << std::endl;

    std::cout << "Initial error: " << graph.error(initialEstimate) << std::endl;
    std::cout << "Error after optimization: " << graph.error(result) << std::endl;

    std::ofstream graphFile("../output/graph.dot");
    graph.saveGraph(graphFile, result);
    graphFile.close();

    return 0;
}