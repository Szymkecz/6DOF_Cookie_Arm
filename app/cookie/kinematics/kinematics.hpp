#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <Eigen/Dense>

namespace Kinematics {

    extern Eigen::Matrix<double, 6, 1>angles;
    extern Eigen::Matrix<double, 6, 1>cords;

    extern Eigen::Matrix4d T6_0;
    extern Eigen::Matrix<double, 6, 6> jacobian;

    Eigen::Matrix4d
    DhTransform(double a_i, double alpha_i, double d_i, double theta_i);
    void calcT6_0(double angles[6]);

} // namespace Kinematics

#endif // KINEMATICS_HPP