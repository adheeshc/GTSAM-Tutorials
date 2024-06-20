#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <iostream>

// An example of gtsam for solving the camera resectioning problem

/*******************************************************************************
 * Camera: f = 1, Image: 100x100, center: 50, 50.0
 * Pose (ground truth): (Xw, -Yw, -Zw, [0,0,2.0]')
 * Known landmarks:
 *    3D Points: (10,10,0) (-10,10,0) (-10,-10,0) (10,-10,0)
 * Perfect measurements:
 *    2D Point:  (55,45)   (45,45)    (45,55)     (55,55)
 *******************************************************************************/

using namespace gtsam;
using symbol_shorthand::X;

class ResectioningFactor : public NoiseModelFactorN<Pose3> {
    typedef NoiseModelFactorN<Pose3> Base;

private:
    Cal3_S2::shared_ptr _K;  // camera intrinsic
    Point3 _p3;              // 3D point
    Point2 _p2;              // 2D measurement of point

public:
    ResectioningFactor(const SharedNoiseModel& model, const Key& key, const Cal3_S2::shared_ptr& calib, const Point2& p2, const Point3& p3)
        : Base(model, key), _K(calib), _p3(p3), _p2(p2) {}

    Vector evaluateError(const Pose3& pose, boost::optional<gtsam::Matrix&> H = boost::none) const override {
        PinholeCamera<Cal3_S2> camera(pose, *_K);
        return camera.project(_p3, H, boost::none, boost::none) - _p2;
    }
};

int main(int argc, char** argv) {
    Cal3_S2::shared_ptr calib(new Cal3_S2(1, 1, 0, 50, 50));  // camera intrinsic values

    // Create Graph
    NonlinearFactorGraph graph;

    // Add factors to graph
    noiseModel::Diagonal::shared_ptr measurementNoise = noiseModel::Diagonal::Sigmas(Vector2(0.5, 0.5));
    std::shared_ptr<ResectioningFactor> factor;
    graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib, Point2(55, 45), Point3(10, 10, 0));
    graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib, Point2(45, 45), Point3(-10, 10, 0));
    graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib, Point2(45, 55), Point3(-10, -10, 0));
    graph.emplace_shared<ResectioningFactor>(measurementNoise, X(1), calib, Point2(55, 55), Point3(10, -10, 0));

    // Create an initial estimate for camera pose
    Values initial;
    initial.insert(X(1), Pose3(Rot3(1, 0, 0, 0, -1, 0, 0, 0, -1), Point3(0, 0, 2)));

    // Optimize using LM
    Values result;
    result = LevenbergMarquardtOptimizer(graph, initial).optimize();

    result.print("Camera Calib values");

    // Save graph file
    std::ofstream graphFile("../output/cameraResectioning.dot");
    graph.saveGraph(graphFile, result);
    graphFile.close();

    int dotResult = std::system("dot -Tpng ../output/cameraResectioning.dot -o ../output/cameraResectioning.png");
    if (dotResult != 0) {
        std::cerr << "Error generating graph image using Graphviz" << std::endl;
        return 1;
    }

    std::cout << "Hello World!" << std::endl;
    return 0;
}