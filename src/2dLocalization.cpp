#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

#include <cmath>
#include <iostream>
#include "readFiles.h"

using namespace gtsam;

/**
 * A simple 2D pose slam example with "GPS" measurements
 *  - The robot moves forward 2 meter each iteration
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have "GPS-like" measurements implemented with a custom factor
 */

class UnaryFactor : public NoiseModelFactor1<Pose2> {
private:
    double _mx, _my;

public:
    typedef boost::shared_ptr<UnaryFactor> shared_ptr;
    using OptionalMatrixType = Matrix*;

    UnaryFactor(Key j, double x, double y, const SharedNoiseModel& model)
        : NoiseModelFactor1<Pose2>(model, j), _mx(x), _my(y) {}

    ~UnaryFactor() override {}

    Vector evaluateError(const gtsam::Pose2& q, boost::optional<gtsam::Matrix&> H = boost::none) const override {
        // The error is then simply calculated as E(q) = h(q) - m:
        // error_x = q.x - mx
        // error_y = q.y - my
        // Node's orientation reflects in the Jacobian, in tangent space this is equal to the right-hand rule rotation matrix
        // H =  [ cos(q.theta)  -sin(q.theta) 0 ]
        //      [ sin(q.theta)   cos(q.theta) 0 ]

        const Rot2& R = q.rotation();
        if (H)
            (*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0.0, R.s(), R.c(), 0.0).finished();

        Vector error(2);
        error[0] = q.x() - _mx;
        error[1] = q.y() - _my;
        return error;
    }

    boost::shared_ptr<gtsam::NonlinearFactor> clone() const override {
        return boost::shared_ptr<gtsam::NonlinearFactor>(new UnaryFactor(*this));
    }
};

int main(int argc, char** argv) {
    NonlinearFactorGraph graph;

    // Add odometry readings
    noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
    std::vector<Pose2> odometryReadings;
    readOdometryFile("../data/odometry.txt", odometryReadings);
    for (size_t i = 0; i < odometryReadings.size(); i++) {
        graph.emplace_shared<BetweenFactor<Pose2>>(i + 1, i + 2, odometryReadings[i], odometryNoise);
    }

    // Add GPS measurements
    noiseModel::Diagonal::shared_ptr GPSNoise = noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1));  // 10cm std on x,y
    std::vector<std::pair<double, double>> GPSReadings;
    readGPSFile("../data/gps.txt", GPSReadings);
    for (size_t i = 0; i < GPSReadings.size(); i++) {
        graph.emplace_shared<UnaryFactor>(i + 1, GPSReadings[i].first, GPSReadings[i].second, GPSNoise);
    }
    graph.print("\nFactor Graph:\n");

    // Add initial estimates
    std::vector<Pose2> estimateReadings;
    readInitialEstimatesFile("../data/estimates.txt", estimateReadings);
    Values initialEstimate;
    for (size_t i = 0; i < estimateReadings.size(); i++) {
        initialEstimate.insert(i + 1, estimateReadings[i]);
    }
    initialEstimate.print("\nInitial Estimate:\n");

    // Optimize using Levenberg-Marquardt optimization
    Values result = LevenbergMarquardtOptimizer(graph, initialEstimate).optimize();
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
    std::cout.precision(2);
    Marginals marginals(graph, result);
    std::cout << "x1 covariance:\n"
              << marginals.marginalCovariance(1) << std::endl;
    std::cout << "x2 covariance:\n"
              << marginals.marginalCovariance(2) << std::endl;
    std::cout << "x3 covariance:\n"
              << marginals.marginalCovariance(3) << std::endl;
    return 0;
}
