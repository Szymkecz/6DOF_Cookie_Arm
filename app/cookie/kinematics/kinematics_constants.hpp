#ifndef KINEMATICS_CONSTANTS_HPP
#define KINEMATICS_CONSTANTS_HPP

namespace Kinematics {

    // extra math
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

} // namespace Kinematics
#endif // KINEMATICS_CONSTANTS_HPP