#include "kinematics.hpp"
#include <Eigen/Dense>
#include <array>
#include <cmath>

using namespace Eigen;
namespace Kinematics {

    constexpr double PI_2 = EIGEN_PI / 2.0;
    constexpr double DEG2RAD = EIGEN_PI / 180.0;
    constexpr double RAD2DEG = 180.0 / EIGEN_PI;

    // DH parameters------------------------------------------------------
    constexpr double BASE_OFFSET = 40.0; // height of wooden pedastal
    constexpr double a1 = 47.0;
    constexpr double a2 = 135.0;
    constexpr double a3 = 29.55;
    constexpr double d1 = 101.0 + BASE_OFFSET;
    constexpr double d4 = 154.25;
    constexpr double d6 = 56.122;

    constexpr double a_i[6] = {a1, a2, a3, 0, 0, 0};
    constexpr double alpha_i[6] = {PI_2, 0, PI_2, -PI_2, PI_2, 0};
    constexpr double d_i[6] = {d1, 0, 0, d4, 0, d6};
    std::array<double, 6> offsets = {0, PI_2, 0, 0, 0, 0};

    // Matrixes-------------------------------------------------------------------
    Eigen::Matrix4d T6_0 = Matrix4d::Identity();
    Eigen::Matrix<double, 6, 6> jacobian = Matrix<double, 6, 6>::Identity();

    Matrix4d DhTransform(double a_i, double alpha_i, double d_i, double theta_i)
    {
        Matrix4d A = Matrix4d::Zero();

        // Compute trigonometric values
        double cos_theta = cos(theta_i);
        double sin_theta = sin(theta_i);
        double cos_alpha = cos(alpha_i);
        double sin_alpha = sin(alpha_i);

        // Assign values based on the DH convention
        A << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha,
            a_i * cos_theta, sin_theta, cos_theta * cos_alpha,
            -cos_theta * sin_alpha, a_i * sin_theta, 0, sin_alpha, cos_alpha,
            d_i, 0, 0, 0, 1;

        return A;
    }
    void calcT6_0(double angles[6])
    {
        // Create temp variables
        std::array<double, 6> q{};

        // Setting temporary variables
        for (uint8_t i = 0; i < 6; i++) {
            q[i] = DEG2RAD * angles[i] + offsets[i];
        }

        constexpr uint8_t num_matrices = 6;
        std::array<Eigen::Matrix4d, num_matrices> A_matrices;

        for (uint8_t i = 0; i < num_matrices; i++) {
            A_matrices[i] = DhTransform(a_i[i], alpha_i[i], d_i[i], q[i]);
        }

        Eigen::Matrix4d temp = A_matrices[0];
        for (uint8_t i = 1; i < A_matrices.size(); i++) {
            temp = temp * A_matrices[i];
        }

        T6_0 = temp;
        // setCords(this->T6_0, this->cords);
    }
} // namespace Kinematics