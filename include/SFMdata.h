#pragma once

#ifndef SFM_DATA_H
#define SFM_DATA_H

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholeCamera.h>

std::vector<gtsam::Point3> createPoints() {
    // Create the set of ground-truth landmarks
    std::vector<gtsam::Point3> points;
    points.push_back(gtsam::Point3(10.0, 10.0, 10.0));
    points.push_back(gtsam::Point3(-10.0, 10.0, 10.0));
    points.push_back(gtsam::Point3(-10.0, -10.0, 10.0));
    points.push_back(gtsam::Point3(10.0, -10.0, 10.0));
    points.push_back(gtsam::Point3(10.0, 10.0, -10.0));
    points.push_back(gtsam::Point3(-10.0, 10.0, -10.0));
    points.push_back(gtsam::Point3(-10.0, -10.0, -10.0));
    points.push_back(gtsam::Point3(10.0, -10.0, -10.0));

    return points;
}

std::vector<gtsam::Pose3> createPoses(
    const gtsam::Pose3& init = gtsam::Pose3(gtsam::Rot3::Ypr(M_PI / 2, 0, -M_PI / 2), gtsam::Point3(30, 0, 0)),
    const gtsam::Pose3& delta = gtsam::Pose3(gtsam::Rot3::Ypr(0, -M_PI / 4, 0), gtsam::Point3(sin(M_PI / 4) * 30, 0, 30 * (1 - sin(M_PI / 4)))),
    int steps = 8) {
    // Default values give a circular trajectory, radius 30 at pi/4 intervals, always facing the circle center
    std::vector<gtsam::Pose3> poses;
    int i = 1;
    poses.push_back(init);
    for (; i < steps; ++i) {
        poses.push_back(poses[i - 1].compose(delta));
    }

    return poses;
}

#endif