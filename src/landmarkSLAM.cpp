#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <fstream>
#include <iostream>
// #include "readFiles.h"

using namespace gtsam;

/**
 * A simple 2D planar slam example with landmarks
 *  - The robot and landmarks are on a 2 meter grid
 *  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 *  - The robot moves 2 meters each step
 *  - We have full odometry between poses
 *  - We have bearing and range information for measurements
 *  - Landmarks are 2 meters away from the robot trajectory
 */

int main(int argc, char** argv) {
    NonlinearFactorGraph graph;

    static Symbol x1('x', 1), x2('x', 2), x3('x', 3);
    static Symbol l1('l', 1), l2('l', 2);

    Pose2 priorMean(0.0, 0.0, 0.0);                                                                    // x,y,theta
    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.3, 0.3, 1));  // 30cm std on x,y, 0.1 rad on theta
    graph.addPrior(x1, priorMean, priorNoise);

    // Add odometry
    Pose2 odometry(2.0, 0.0, 0.0);
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));  // 20cm std on x,y, 0.1 rad on theta
    graph.emplace_shared<BetweenFactor<Pose2>>(x1, x2, odometry, odometryNoise);
    graph.emplace_shared<BetweenFactor<Pose2>>(x2, x3, odometry, odometryNoise);

    // Add Range-Bearing measurements
    noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.2));  // 0.1 rad std on bearing, 20cm on range
    Rot2 bearing11 = Rot2::fromDegrees(45), bearing21 = Rot2::fromDegrees(90), bearing32 = Rot2::fromDegrees(90);
    double range11 = std::sqrt(4.0 + 4.0), range21 = 2.0, range32 = 2.0;

    graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x1, l1, bearing11, range11, measurementNoise);
    graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x2, l1, bearing21, range21, measurementNoise);
    graph.emplace_shared<BearingRangeFactor<Pose2, Point2>>(x3, l2, bearing32, range32, measurementNoise);

    graph.print("Factor Graph:\n");

    Values initialEstimate;
    initialEstimate.insert(x1, Pose2(0.5, 0.0, 0.2));
    initialEstimate.insert(x2, Pose2(2.3, 0.1, -0.2));
    initialEstimate.insert(x3, Pose2(4.1, 0.1, 0.1));
    initialEstimate.insert(l1, Point2(1.8, 2.1));
    initialEstimate.insert(l2, Point2(4.1, 1.8));

    initialEstimate.print("Initial Estimate:\n");

    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
    Values result = optimizer.optimize();
    result.print("Final Result:\n");

    std::cout << "landmark 1: " << result.at<gtsam::Point2>(l1) << std::endl;
    std::cout << "landmark 2:  " << result.at<gtsam::Point2>(l2) << std::endl;

    std::vector<double> resultVector;
    for (const auto& key_value : result) {
        Key key = key_value.key;
        if (key == x1 || key == x2 || key == x3) {
            try {
                Pose2 pose = result.at<Pose2>(key);
                resultVector.push_back(std::round(pose.x() * 100.0) / 100.0);
                resultVector.push_back(std::round(pose.y() * 100.0) / 100.0);
                resultVector.push_back(std::round(pose.theta() * 100.0) / 100.0);
            } catch (const gtsam::ValuesIncorrectType& e) {
                std::cerr << "Incorrect type for key " << key << ": " << e.what() << "\n";
            }
        }
    }

    for (size_t i = 0; i < resultVector.size(); i += 3) {
        std::cout << "Pose x" << (i / 3) + 1 << ": " << "[ ";
        for (size_t j = i; j < i + 3 && j < resultVector.size(); ++j) {
            std::cout << resultVector[j] << " ";
        }
        std::cout << "]\n";
    }
    std::ofstream graphFile("../output/landmarkSLAM_graph.dot");
    if (!graphFile) {
        std::cerr << "Error opening file for writing: ../output/landmarkSLAM_graph.dot" << std::endl;
        return 1;
    }
    graph.saveGraph(graphFile, result);
    graphFile.close();

    int dotResult = std::system("dot -Tpng ../output/landmarkSLAM_graph.dot -o ../output/landmarkSLAM_graph.png");
    if (dotResult != 0) {
        std::cerr << "Error generating graph image using Graphviz" << std::endl;
        return 1;
    }

    return 0;
}