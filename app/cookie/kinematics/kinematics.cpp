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
    // local
    Eigen::Matrix4d T6_0 = Matrix4d::Identity();
    Eigen::Matrix<double, 6, 6> jacobian = Matrix<double, 6, 6>::Identity();
    Eigen::Matrix4d Ttool_6 = Matrix4d::Identity();

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

        /* // Extract the rotation matrix
        Eigen::Matrix3d rotationMatrix = A.block<3, 3>(0, 0);

        // Compute the ZYX Euler angles (yaw, pitch, roll)
        Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(2, 1, 0);

        // Assign the Euler angles to the cords vector
        Kinematics::cords.segment<3>(3) = eulerAngles; */

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

    // for Inverse kinematics calculation
    Eigen::Matrix4d cords_to_T6_0(Eigen::Matrix<double, 6, 1>& cords)
    {
        Matrix4d temp = Eigen::Matrix4d::Identity();
        // 1srt row
        temp(0, 0) = cos(cords(3)) * cos(cords(4));
        temp(0, 1) = cos(cords(3)) * sin(cords(4)) * sin(cords(5)) -
                     sin(cords(3)) * cos(cords(5));
        temp(0, 2) = cos(cords(3)) * sin(cords(4)) * sin(cords(5)) +
                     sin(cords(3)) * cos(cords(5));

        // 2nd row
        temp(1, 0) = sin(cords(3)) * cos(cords(4));
        temp(1, 1) = sin(cords(3)) * sin(cords(4)) * sin(cords(5)) +
                     sin(cords(3)) * cos(cords(5));
        temp(1, 2) = sin(cords(3)) * sin(cords(4)) * sin(cords(5)) -
                     sin(cords(3)) * cos(cords(5));
        // 3rd row
        temp(2, 0) = -sin(cords(4));
        temp(2, 1) = cos(cords(4)) * sin(cords(5));
        temp(2, 2) = cos(cords(4)) * cos(cords(5));

        return temp;
    }
    void inverse_kinematics(Eigen::Matrix<double, 6, 1>& cords)
    {
        // calculate from Ttool_0 to T5_0
        Eigen::Matrix4d temp = cords_to_T6_0(cords);

        Eigen::Matrix4d inv_Ttool;
        inv_Ttool = Matrix4d::Identity();

        Eigen::Matrix3d R_inv_tool = Ttool_6.block<3, 3>(0, 0).transpose();
        // FIX THE CODE
        //  inv_Ttool =

        // R 0-6 reverse kin
        temp = temp * inv_Ttool;
        // R 0-6 negate
        Matrix4d T5_6 = Matrix4d::Identity();
        T5_6(2, 3) = -d_i[5];

        // R 0-5 reverse kin    (center spherical wrist)
        temp = temp * T5_6;

        // calculate theta1,2,3-------------------------------
        // theta1
        angles[0] = RAD2DEG * atan2(temp(1, 3), temp(0, 3));

        // theta2,3
        double new_x = temp(0, 3) * cos(cords[0]) - temp(1, 3) * sin(cords[0]);
        double new_y = temp(1, 3) * cos(cords[0]) + temp(0, 3) * sin(cords[0]);

        double L1, L2, L3, L4, theta_B, theta_C, theta_D, theta_E;
        L1 = abs(new_x - a_i[0]);
        L4 = temp(2, 3) - d_i[0];
        L2 = sqrt(pow(L1, 2) + pow(L4, 2));
        L3 = sqrt(pow(d_i[3], 2) + pow(a_i[2], 2));
        theta_B = atan(L1 / L4);
        theta_C = acos((pow(a_i[1], 2) + pow(L2, 2) - pow(L3, 2)) /
                       (2 * L2 * a_i[1]));
        theta_D = acos((pow(L3, 2) + pow(a_i[1], 2) - pow(L2, 2)) /
                       (2 * L3 * a_i[1]));
        theta_E = atan(a_i[2] / d_i[3]);

        // theta2 calc
        if (new_x > a_i[0]) {
            if (L4 > 0) {
                angles[1] = (theta_B - theta_C) * RAD2DEG;
            } else {
                angles[1] = (theta_B - theta_C) * RAD2DEG + 180;
            }

        } else {
            angles[1] = -(theta_B + theta_C) * RAD2DEG;
        }

        // theta3 calc
        angles[2] = -(theta_D + theta_E) * RAD2DEG + 90;
    }

    // #ifdef DEBUG
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

        /*   printf("cords_vector\r\n");
          for (uint8_t i = 0; i < 6; ++i) {
              sprintf(buffer, "[%.3f]\r\n", Kinematics::cords(i));
              printf(buffer);
          }

          printf("----------------\r\n"); */
        printf("CC[%.1f,%.1f,%.1f,%.1f,%.1f,%.1f];\r\n",
               Kinematics::cords[0],
               Kinematics::cords[1],
               Kinematics::cords[2],
               Kinematics::cords[3] * RAD2DEG,
               Kinematics::cords[4] * RAD2DEG,
               Kinematics::cords[5] * RAD2DEG);
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
    // #endif // DEBUG
} // namespace Kinematics