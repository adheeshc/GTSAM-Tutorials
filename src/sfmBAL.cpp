/*
Solve a structure-from-motion example from a "Bundle Adjustment in the Large" BAL file
Compare the ordering performance for COLAMD vs METIS
using BAL dataset - problem-93-61203-pre.txt
*/

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sfm/SfmData.h>
#include <gtsam/slam/GeneralSFMFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/slam/dataset.h>
#include <fstream>
#include <iostream>
#include <vector>

using namespace gtsam;
using symbol_shorthand::C;
using symbol_shorthand::P;

int main(int argc, char** argv) {
    std::string filename = "../data/BAL/problem-16-22106-pre.txt";
    SfmData mydata = SfmData::FromBalFile(filename);
    std::cout << "read " << mydata.numberTracks() << " tracks on " << mydata.numberCameras() << " cameras" << std::endl;

    // Create a factor graph
    NonlinearFactorGraph graph;

    // Add measurements to factor graph
    noiseModel::Isotropic::shared_ptr measurementNoise = noiseModel::Isotropic::Sigma(2, 1.0);
    size_t j = 0;
    for (const SfmTrack& track : mydata.tracks) {
        for (const auto& [i, uv] : track.measurements) {
            graph.emplace_shared<GeneralSFMFactor<SfmCamera, Point3>>(uv, measurementNoise, C(i), P(j));
        }
        j++;
    }

    // Add Prior to factor graph
    noiseModel::Isotropic::shared_ptr cameraNoise = noiseModel::Isotropic::Sigma(9, 0.1);
    noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
    graph.addPrior(C(0), mydata.cameras[0], cameraNoise);
    graph.addPrior(P(0), mydata.tracks[0].p, pointNoise);

    // Create Initial Estimates
    Values initialEstimates;
    size_t i = 0;
    j = 0;
    for (const SfmCamera& camera : mydata.cameras) {
        initialEstimates.insert(C(i++), camera);
    }
    for (const SfmTrack& track : mydata.tracks) {
        initialEstimates.insert(P(j++), track.p);
    }

    // Optimize the graph and print results
    LevenbergMarquardtParams params_using_COLAMD, params_using_METIS;
    try {
        params_using_METIS.setVerbosity("ERROR");
        gttic_(METIS_ORDERING);
        params_using_METIS.ordering = Ordering::Create(Ordering::METIS, graph);
        gttoc_(METIS_ORDERING);

        params_using_COLAMD.setVerbosity("ERROR");
        gttic_(COLAMD_ORDERING);
        params_using_COLAMD.ordering = Ordering::Create(Ordering::COLAMD, graph);
        gttoc_(COLAMD_ORDERING);
    } catch (std::exception& e) {
        std::cout << e.what();
    }

    if (params_using_COLAMD.ordering == params_using_METIS.ordering) {
        std::cout << "COLAMD and METIS produce the same ordering. "
                  << "ERROR!!!" << std::endl;
    }

    Values result_METIS, result_COLAMD;
    try {
        gttic_(OPTIMIZE_WITH_METIS);
        LevenbergMarquardtOptimizer lm_METIS(graph, initialEstimates, params_using_METIS);
        result_METIS = lm_METIS.optimize();
        gttoc_(OPTIMIZE_WITH_METIS);

        gttic_(OPTIMIZE_WITH_COLAMD);
        LevenbergMarquardtOptimizer lm_COLAMD(graph, initialEstimates, params_using_COLAMD);
        result_COLAMD = lm_COLAMD.optimize();
        gttoc_(OPTIMIZE_WITH_COLAMD);
    } catch (std::exception& e) {
        std::cout << e.what();
    }

    // printing the result
    std::cout << "COLAMD final error: " << graph.error(result_COLAMD) << std::endl;
    std::cout << "METIS final error: " << graph.error(result_METIS) << std::endl;
    std::cout << std::endl;
    std::cout << "Time comparison by solving " << filename << " results:" << std::endl;
    std::cout << mydata.numberTracks() << " point tracks and " << mydata.numberCameras() << " cameras" << std::endl;
    tictoc_print_();

    return 0;
}