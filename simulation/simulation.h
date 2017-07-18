#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>
#include <sophus/se3.hpp>

struct Pose{
    Sophus::SE3d pose;
    std::vector<Eigen::Vector3d> projectPoints;
    Sophus::SE3d noisePose;
    std::vector<Eigen::Vector2i> noiseFeats;
};

class Simulation
{
public:
    Simulation();
    void run(Eigen::Matrix2d& featVar);
    std::vector<Pose> poses;
    std::vector<Eigen::Vector3d> points;
};

#endif // SIMULATION_H
