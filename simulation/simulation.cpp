#include "simulation.h"

#include "Eigen/Geometry"
#include "g2o/stuff/sampler.h"

Simulation::Simulation()
{

}


void Simulation::run(Eigen::Matrix2d &featVar)
{
    double r = 10.0;
    double b1[3] = {1.5, -0.3, 2.5}, b2[3] = {3.7, 0, 3}, b3[3] = {4.3, 2.4, 6}, b4[3] = {1.4, 1.5, 4},
            b5[3] = {-0.5, 1, 5}, b6[3] = {- 1, 0, 4.2}, b7[3] = {0, -1, 3.4}, b8[3] = {-3.3, 2.6};
    Eigen::Map<Eigen::Vector3d> B1(b1), B2(b2), B3(b3), B4(b4), B5(b5), B6(b6), B7(b7), B8(b8);

    Eigen::Vector3d B11 = B1 / 2.0, B12 = B2 / 2.0, B13 = B3 / 2.0, B14 = B4 / 2.0,
            B15 = B5 / 2.0, B16 = B6 / 2.0, B17 = B7 / 2.0, B18 = B8 / 2.0;

    Eigen::Vector3d B21 = B1 * 2.0, B22 = B2 * 2.0, B23 = B3 * 2.0, B24 = B4 * 2.0,
            B25 = B5 * 2.0, B26 = B6 * 2.0, B27 = B7 * 2.0, B28 = B8 * 2.0;

    points.push_back(B1);
    points.push_back(B2);
    points.push_back(B3);
    points.push_back(B4);
    points.push_back(B5);
    points.push_back(B6);
    points.push_back(B7);
    points.push_back(B8);
    points.push_back(B11);
    points.push_back(B12);
    points.push_back(B13);
    points.push_back(B14);
    points.push_back(B15);
    points.push_back(B16);
    points.push_back(B17);
    points.push_back(B18);
    points.push_back(B21);
    points.push_back(B22);
    points.push_back(B23);
    points.push_back(B24);
    points.push_back(B25);
    points.push_back(B26);
    points.push_back(B27);
    points.push_back(B28);

    g2o::GaussianSampler<Eigen::Vector2d, Eigen::Matrix2d> featNoise;
    featNoise.setDistribution(featVar);

    Eigen::Matrix<double, 6, 6> poseVar = 0.01 * Eigen::Matrix<double, 6, 6>::Identity();

    g2o::GaussianSampler<Eigen::Matrix<double, 6, 1>, Eigen::Matrix<double, 6, 6>> poseNoise;
    poseNoise.setDistribution(poseVar);

    Eigen::Vector3d zx(0, 0, 1);
    bool plus = true;
    double x = 0.0, y = 0.0, z = 0.0;
    int theta = 45;
    for(int k = 0; k <= 1000; ++k) {
        z = 6.0 - k * 0.012;
        x = r * 1.2 * exp(k / 800.0) * std::cos(3.1415 / 180.0 * theta);
        y = r * exp(k / 1000.0) * std::sin(3.1415 / 180.0 * theta);
        if(theta == 135)
            plus = false;
        if(theta == 45)
            plus = true;
        if(plus)
            theta++;
        else
            theta--;

        Eigen::Vector3d n_(x, y, z);
        auto z1 = -n_.normalized();
        Eigen::Vector3d x1 = -zx.cross(z1);
        Eigen::Vector3d y1 = z1.cross(x1);
        Eigen::Matrix3d R;
        R.block<3, 1>(0, 0) = x1;
        R.block<3, 1>(0, 1) = y1;
        R.block<3, 1>(0, 2) = z1;
        Sophus::SE3d se3(R, n_);
        Pose pose;
        pose.pose = se3;

        pose.projectPoints.push_back(se3 * B1);
        pose.projectPoints.push_back(se3 * B2);
        pose.projectPoints.push_back(se3 * B3);
        pose.projectPoints.push_back(se3 * B4);
        pose.projectPoints.push_back(se3 * B5);
        pose.projectPoints.push_back(se3 * B6);
        pose.projectPoints.push_back(se3 * B7);
        pose.projectPoints.push_back(se3 * B8);
        pose.projectPoints.push_back(se3 * B11);
        pose.projectPoints.push_back(se3 * B12);
        pose.projectPoints.push_back(se3 * B13);
        pose.projectPoints.push_back(se3 * B14);
        pose.projectPoints.push_back(se3 * B15);
        pose.projectPoints.push_back(se3 * B16);
        pose.projectPoints.push_back(se3 * B17);
        pose.projectPoints.push_back(se3 * B18);
        pose.projectPoints.push_back(se3 * B21);
        pose.projectPoints.push_back(se3 * B22);
        pose.projectPoints.push_back(se3 * B23);
        pose.projectPoints.push_back(se3 * B24);
        pose.projectPoints.push_back(se3 * B25);
        pose.projectPoints.push_back(se3 * B26);
        pose.projectPoints.push_back(se3 * B27);
        pose.projectPoints.push_back(se3 * B28);

        pose.noisePose = se3 * Sophus::SE3d::exp(poseNoise.generateSample());

        for(size_t i = 0; i < pose.projectPoints.size(); ++i) {
            Eigen::Vector2d feat_;
            feat_(0) = pose.projectPoints[i](0) / pose.projectPoints[i](2);
            feat_(1) = pose.projectPoints[i](1) / pose.projectPoints[i](2);
            feat_ = feat_ + featNoise.generateSample();
            pose.noiseFeats.push_back(Eigen::Vector2i(int(feat_(0)), int(feat_(1))));
        }

        poses.push_back(pose);
    }
}

