#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <Eigen/Dense>
#include <array>

namespace Kinematics {

    // kinematics data-------------------------------
    // extern Eigen::Matrix<double, 6, 1> angles; // maybe useless
    extern std::array<double, 6> angles;
    extern Eigen::Matrix<double, 6, 1> cords;
    // extern Eigen::Matrix4d T6_0;
    // extern Eigen::Matrix<double, 6, 6> jacobian;

    Eigen::Matrix4d
    dh_transform(double a_i, double alpha_i, double d_i, double theta_i);
    void calc_T6_0(std::array<double, 7>& angles);
    void calc_jacobian(std::array<double, 7>& angles);
    Eigen::Matrix<double, 6, 6> calc_inv_jacobian();
    void set_cords(Eigen::Matrix4d& A);

    Eigen::Matrix4d cords_to_T6_0(Eigen::Matrix<double, 6, 1>& cords);
    void inverse_kinematics(Eigen::Matrix<double, 6, 1>& cords);

#ifdef DEBUG
    // print functions will only be included in debug builds
    void print_fk();
    void print_cords();
    void print_jacobian();
#endif // DEBUG

} // namespace Kinematics

#endif // KINEMATICS_HPP