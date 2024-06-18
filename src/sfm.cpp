/**
 * A structure-from-motion example with landmarks, default function arguments give
 *  - The landmarks form a 10 meter cube
 *  - The robot rotates around the landmarks, always facing towards the cube
 * provides the “bundle adjustment” optimization given points, poses and camera params
 */

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <fstream>
#include <iostream>
#include <vector>
#include "SFMdata.h"

using namespace gtsam;

int main(int argc, char** argv) {
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0));  // camera calibration

    noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);
    vector<Point3> points = createPoints();
    vector<Pose3> poses = createPoses();

    NonlinearFactorGraph graph;

    // Add Prior
    noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
    graph.addPrior(Symbol('x', 0), poses[0], poseNoise);

    // Simulated measurements from each camera pose
    for (size_t i = 0; i < poses.size(); i++) {
        PinholeCamera<Cal3_S2> camera(poses[i], *K);
        for (size_t j = 0; j < points.size(); j++) {
            Point2 measurement = camera.project(points[j]);
            graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), K);
        }
    }

    // We add a prior on the position of the first landmark. This fixes the scale by indicating the distance between the first camera and the first landmark.
    // All other landmark positions are interpreted using this scale.
    noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
    graph.addPrior(Symbol('l', 0), points[0], pointNoise);

    // graph.print("\n FactorGraph: \n");

    Values initEstimates;
    for (size_t i = 0; i < poses.size(); i++) {
        auto pose = poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.1, 0.2)));
        initEstimates.insert(Symbol('x', i), pose);
    }
    for (size_t j = 0; j < points.size(); j++) {
        auto point = points[j] + Point3(-0.25, 0.2, 0.15);
        initEstimates.insert<Point3>(Symbol('l', j), point);
    }

    initEstimates.print("\nInitial Estimates: \n");

    Values result = DoglegOptimizer(graph, initEstimates).optimize();
    result.print("Final results: \n");

    std::cout << "Initial error: " << graph.error(initEstimates) << std::endl;
    std::cout << "Error after optimization: " << graph.error(result) << std::endl;

    std::ofstream graphFile("../output/sfm.dot");
    graph.saveGraph(graphFile, result);
    graphFile.close();

    int dotResult = std::system("dot -Tpng ../output/sfm.dot -o ../output/sfm.png");
    if (dotResult != 0) {
        std::cerr << "Error generating graph image using Graphviz" << std::endl;
        return 1;
    }

    return 0;
}