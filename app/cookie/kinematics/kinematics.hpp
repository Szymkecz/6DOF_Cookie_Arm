#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <Eigen/Dense>
#include <array>

namespace Kinematics {

    // kinematics data-------------------------------
    extern Eigen::Matrix<double, 6, 1> angles;
    extern Eigen::Matrix<double, 6, 1> cords;
    extern Eigen::Matrix4d T6_0;
    extern Eigen::Matrix<double, 6, 6> jacobian;

    Eigen::Matrix4d
    DhTransform(double a_i, double alpha_i, double d_i, double theta_i);
    void calcT6_0(std::array<double, 7> &angles);
    void calcJacobian(std::array<double, 7> &angles);
    void set_cords(Eigen::Matrix4d& A);

    // print functions
    void print_fk();
    void print_cords();
    // void print_jacobian() {};

} // namespace Kinematics

#endif // KINEMATICS_HPP