#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/GeneralSFMFactor.h>

#include "SFMdata.h"

#include <iostream>
#include <vector>

using namespace gtsam;

int main(int argc, char** argv) {
    std::vector<Pose3> poses = createPoses();
    std::vector<Point3> points = createPoints();

    NonlinearFactorGraph graph;

    auto poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
    graph.addPrior(Symbol('x', 0), poses[0], poseNoise);

    Cal3_S2 K(50.0, 50.0, 0.0, 50.0, 50.0);

    auto measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);

    for (size_t i = 0; i < poses.size(); i++) {
        for (size_t j = 0; j < points.size(); j++) {
            PinholeCamera<Cal3_S2> camera(poses[i], K);
            Point2 measurement = camera.project(points[j]);
            graph.emplace_shared<GeneralSFMFactor2<Cal3_S2>>(measurement, measurementNoise, Symbol('x', i), Symbol('l', j), Symbol('K', 0));
        }
    }

    auto pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
    graph.addPrior(Symbol('l', 0), points[0], pointNoise);

    auto calibNoise = noiseModel::Diagonal::Sigmas((Vector(5) << 500, 500, 0.1, 100, 100).finished());
    graph.addPrior(Symbol('K', 0), K, calibNoise);

    Values initialEstimate;
    initialEstimate.insert(Symbol('K', 0), Cal3_S2(60.0, 60.0, 0.0, 45.0, 45.0));
    for (size_t i = 0; i < poses.size(); i++) {
        initialEstimate.insert(Symbol('x', i), poses[i].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
    }

    for (size_t j = 0; j < points.size(); j++) {
        initialEstimate.insert<Point3>(Symbol('l', j), points[j] + Point3(-0.25, 0.20, 0.15));
    }

    Values result = DoglegOptimizer(graph, initialEstimate).optimize();

    std::cout << "Initial error: " << graph.error(initialEstimate) << std::endl;
    std::cout << "Error after optimization: " << graph.error(result) << std::endl;

    std::ofstream graphFile("../output/selfCalib.dot");
    graph.saveGraph(graphFile, result);
    graphFile.close();

    int dotResult = std::system("dot -Tpng ../output/selfCalib.dot -o ../output/selfCalib.png");
    if (dotResult != 0) {
        std::cerr << "Error generating graph image using Graphviz" << std::endl;
        return 1;
    }

    return 0;
}