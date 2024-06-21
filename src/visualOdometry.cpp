/**
 * A 3D stereo visual odometry example
 *  - robot starts at origin
 *  -moves forward, taking periodic stereo measurements
 *  -takes stereo readings of many landmarks
 */

#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/utilities.h>
#include <gtsam/slam/StereoFactor.h>
#include <gtsam/slam/dataset.h>

#include <fstream>
#include <iostream>
#include <string>

using namespace gtsam;

int main(int argc, char** argv) {
    NonlinearFactorGraph graph;
    Values initialEstimate;
    const auto noise = noiseModel::Isotropic::Sigma(3, 1);

    std::string calib_file = "../data/VOcalib00.txt";
    std::string pose_file = "../data/VOcameraPoses00.txt";
    std::string factor_file = "../data/VOstereoFactors00.txt";

    // Load Calibration
    double fx, fy, s, cx, cy, b;  // focal lengths fx, fy, skew s, principal point cx, cy, baseline b
    std::ifstream calib(calib_file.c_str());
    std::cout << "Reading calib file: ";
    calib >> fx >> fy >> s >> cx >> cy >> b;
    const Cal3_S2Stereo::shared_ptr K(new Cal3_S2Stereo(fx, fy, s, cx, cy, b));
    std::cout << "Done!" << std::endl;

    // Load Camera Poses
    std::ifstream pose(pose_file.c_str());
    std::cout << "Reading camera poses file: ";
    int poseID;
    MatrixRowMajor m(4, 4);
    while (pose >> poseID) {
        for (int i = 0; i < 16; i++) {
            pose >> m.data()[i];
        }
        initialEstimate.insert(Symbol('x', poseID), Pose3(m));
    }
    std::cout << "Done!" << std::endl;

    // Load stereo measurement details
    std::ifstream stereoFactors(factor_file.c_str());
    std::cout << "Reading stereo measurements file: ";

    size_t x, l;       // camera and landmark keys
    double uL, uR, v;  // pixel coordinates uL, uR, v (same for left/right images due to rectification)
    double X, Y, Z;    // landmark coordinates X, Y, Z in camera frame, resulting from triangulation
    while (stereoFactors >> x >> l >> uL >> uR >> v >> X >> Y >> Z) {
        graph.emplace_shared<GenericStereoFactor<Pose3, Point3>>(StereoPoint2(uL, uR, v), noise, Symbol('x', x), Symbol('l', l), K);

        if (!initialEstimate.exists(Symbol('l', l))) {
            Pose3 cameraPose = initialEstimate.at<Pose3>(Symbol('x', x));
            Point3 worldPoint = cameraPose.transformFrom(Point3(X, Y, Z));
            initialEstimate.insert(Symbol('l', l), worldPoint);
        }
    }
    std::cout << "Done!" << std::endl;

    Pose3 firstPose = initialEstimate.at<Pose3>(Symbol('x', 1));
    graph.emplace_shared<NonlinearEquality<Pose3>>(Symbol('x', 1), firstPose);

    LevenbergMarquardtParams params;
    params.orderingType = Ordering::METIS;
    LevenbergMarquardtOptimizer optimizer(graph, initialEstimate, params);
    Values result = optimizer.optimize();

    std::cout << "Final result:" << std::endl;
    Values pose_values = utilities::allPose3s(result);
    // pose_values.print("Final camera poses:\n");

    std::cout << "Initial error: " << graph.error(initialEstimate) << std::endl;
    std::cout << "Error after optimization: " << graph.error(result) << std::endl;

    std::ofstream graphFile("../output/VO00.dot");
    graph.saveGraph(graphFile, result);
    graphFile.close();

    return 0;
}