#include "kinematics.hpp"
#include "kinematics_constants.hpp"
// #include <Eigen/Dense>
#include <array>
#include <cmath>

using namespace Eigen;
namespace Kinematics {

    // kinematics data--------------------------------
    Eigen::Matrix<double, 6, 1> angles;
    Eigen::Matrix<double, 6, 1> cords;
    Eigen::Matrix4d T6_0 = Eigen::Matrix4d::Identity();
    Eigen::Matrix<double, 6, 6> jacobian = Matrix<double, 6, 6>::Identity();

    Matrix4d
    dh_transform(double a_i, double alpha_i, double d_i, double theta_i)
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
    void calc_T6_0(std::array<double, 7>& angles)
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
            A_matrices[i] = dh_transform(a_i[i], alpha_i[i], d_i[i], q[i]);
        }

        Eigen::Matrix4d temp = A_matrices[0];
        for (uint8_t i = 1; i < A_matrices.size(); i++) {
            temp = temp * A_matrices[i];
        }

        T6_0 = temp;
        set_cords(T6_0);
    }
    void calc_jacobian(std::array<double, 7>& angles)
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
            A_matrices[i] = dh_transform(a_i[i], alpha_i[i], d_i[i], q[i]);
        }

        // Jacobian pi and zi-1 calculation
        Eigen::Matrix4d temp = A_matrices[0];
        std::array<Eigen::Vector3d, num_matrices + 1>
            p_positions; // Pi positions for Jacobian
        std::array<Eigen::Vector3d, num_matrices>
            z_axes; // define rotation speeds

        p_positions[0] << 0, 0, 0;
        p_positions[1] = temp.block<3, 1>(0, 3);
        z_axes[0] << 0, 0, 1;

        // Compute Forward kinematics T0_1*T1_2...*T5_6
        for (uint8_t i = 1; i < A_matrices.size(); i++) {
            // extract 3x1 vector beginning position is 0,3 so the a14
            z_axes[i] = temp.block<3, 1>(0, 2);
            temp *= A_matrices[i];
            p_positions[i + 1] = temp.block<3, 1>(0, 3);
        }

        // calculate Jacobian
        Eigen::Matrix<double, num_matrices, 6> jacobi;
        for (int i = 0; i < num_matrices; ++i) {
            Eigen::Vector3d linear_velocity =
                z_axes[i].cross(p_positions[6] - p_positions[i]);
            jacobi.block<3, 1>(0, i) = linear_velocity;
        }
        for (int i = 0; i < num_matrices; ++i) {
            jacobi.block<3, 1>(3, i) = z_axes[i];
        }

        // Copy the final transformation matrix to the struct
        T6_0 = temp;
        Kinematics::jacobian = jacobi;

        // set x,y,z, calculate Rz, Ry, Rx
        set_cords(T6_0);
    }
    Eigen::Matrix<double, 6, 6> calc_inv_jacobian()
    {
        constexpr double lambda = 0.01;
        const auto& J = Kinematics::jacobian;

        Matrix<double, 6, 6> identity = Matrix<double, 6, 6>::Identity();
        Matrix<double, 6, 6> JJT = J * J.transpose();
        Matrix<double, 6, 6> damped = JJT + lambda * lambda * identity;

        return J.transpose() * damped.inverse();
    }
    void set_cords(Eigen::Matrix4d& A)
    {
        Kinematics::cords.segment<3>(0) = A.block<3, 1>(0, 3);

        // // Extract the rotation matrix
        // Eigen::Matrix3d rotationMatrix = A.block<3, 3>(0, 0);

        // // Compute the ZYX Euler angles (yaw, pitch, roll)
        // Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(2, 1, 0);

        // // Assign the Euler angles to the cords vector
        // Kinematics::cords.segment<3>(3) = eulerAngles;

        // diffrent kind of euler angles
        //  Manual extraction of ZYX Euler angles (yaw-pitch-roll)
        Eigen::Matrix3d R = A.block<3, 3>(0, 0);

        double roll = std::atan2(R(2, 1), R(2, 2)); // X-axis rotation (ϕ)
        double pitch =
            std::atan2(-R(2, 0),
                       std::sqrt(std::pow(R(0, 0), 2) +
                                 std::pow(R(1, 0), 2))); // Y-axis rotation (θ)
        double yaw = std::atan2(R(1, 0), R(0, 0));

        Kinematics::cords.segment<3>(3) = Vector3d(yaw, pitch, roll);
    }

#ifdef DEBUG
    // print functions
    void print_fk()
    {
        char buffer[128];

        printf("fk_matrix\r\n");
        for (int i = 0; i < 4; ++i) {
            sprintf(buffer,
                    "[%.3f, %.3f, %.3f, %.3f]\r\n",
                    Kinematics::T6_0(i, 0),
                    Kinematics::T6_0(i, 1),
                    Kinematics::T6_0(i, 2),
                    Kinematics::T6_0(i, 3));
            printf(buffer);
        }
        printf("----------------\r\n");
    }
    void print_cords()
    {
        char buffer[128];

        printf("cords_vector\r\n");
        for (uint8_t i = 0; i < 6; ++i) {
            sprintf(buffer, "[%.3f]\r\n", Kinematics::cords(i));
            printf(buffer);
        }

        printf("----------------\r\n");
    }
    void print_jacobian()
    {
        char buffer[128];

        printf("jacobian\r\n");
        for (int i = 0; i < 6; ++i) {
            sprintf(buffer,
                    "[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\r\n",
                    Kinematics::jacobian(i, 0),
                    Kinematics::jacobian(i, 1),
                    Kinematics::jacobian(i, 2),
                    Kinematics::jacobian(i, 3),
                    Kinematics::jacobian(i, 4),
                    Kinematics::jacobian(i, 5));
            printf(buffer);
        }
        printf("----------------\r\n");
    }
#endif // DEBUG
} // namespace Kinematics