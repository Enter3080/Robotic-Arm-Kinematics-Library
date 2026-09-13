#pragma once

#include <Eigen/Dense>
#include <Eigen/SVD>

#include <array>
#include <cmath>
#include <stdexcept>

namespace kinematics {

/**
 * @brief 6-DOF robotic arm kinematics library
 * 
 * Uses Craig Modified DH parameters:
 * A_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
 */
class Kinematics {
public:
    static constexpr double PI = 3.14159265358979323846;

    /**
     * @brief Constructor, initializes with Modified DH parameters
     * @param alpha Link twist angles (6)
     * @param a     Link lengths (6)
     * @param d     Link offsets (6)
     */
    Kinematics(
        const std::array<double, 6>& alpha,
        const std::array<double, 6>& a,
        const std::array<double, 6>& d
    );

    /**
     * @brief Set current joint angles
     */
    void set_joint_angles(const std::array<double, 6>& q);

    /**
     * @brief Get current joint angles
     */
    std::array<double, 6> get_joint_angles() const;

    /**
     * @brief Set MDH theta fixed offsets (installation offsets for each joint)
     * @param offsets 6 joint offset values (radians)
     *
     * General note: Different robot arms' URDF joint origins may have fixed offsets
     * such as ±PI/2. This library remains generic (all zeros by default);
     * offsets are provided by the caller per specific robot.
     */
    void set_theta_offsets(const std::array<double, 6>& offsets);

    /**
     * @brief Set joint limits (radians)
     * @param min_limits Lower limits for each joint
     * @param max_limits Upper limits for each joint
     *
     * General conventions:
     * - Default limits are ±1e9 (treated as continuous, only normalized to [-PI, PI]);
     * - If min/max are in the ±1e9 range => continuous joint;
     * - Joints with finite limits are clamped to [min, max].
     * Limits are provided by the caller (arm_action_server), the library
     * does not embed any robot-specific data.
     */
    void set_joint_limits(
        const std::array<double, 6>& min_limits,
        const std::array<double, 6>& max_limits
    );

    /**
     * @brief Rotation matrix about X axis
     */
    static Eigen::Matrix4d rotation_x(double angle);

    /**
     * @brief Translation matrix along X axis
     */
    static Eigen::Matrix4d translation_x(double distance);

    /**
     * @brief Rotation matrix about Z axis
     */
    static Eigen::Matrix4d rotation_z(double angle);

    /**
     * @brief Translation matrix along Z axis
     */
    static Eigen::Matrix4d translation_z(double distance);

    /**
     * @brief Single Modified DH transformation
     * @param alpha Link twist angle
     * @param a     Link length
     * @param theta Joint angle
     * @param d     Link offset
     * @return 4x4 homogeneous transformation matrix
     */
    Eigen::Matrix4d dh_transform(
        double alpha,
        double a,
        double theta,
        double d
    ) const;

    /**
     * @brief Compute total transformation from base_link to hand_link
     */
    Eigen::Matrix4d total_transform() const;

    /**
     * @brief Get current end-effector position (extracted from total_transform)
     */
    Eigen::Vector3d end_position() const;

    /**
     * @brief Compute 3x6 position Jacobian matrix
     * 
     * J_v_i = z_i × (p_e - p_i)
     */
    Eigen::Matrix<double, 3, 6> position_jacobian() const;

    /**
     * @brief Compute Moore-Penrose pseudo-inverse (using SVD)
     * @param matrix             Input matrix
     * @param relative_tolerance Singular value truncation threshold
     * @return Pseudo-inverse matrix
     */
    static Eigen::MatrixXd pseudo_inverse(
        const Eigen::MatrixXd& matrix,
        double relative_tolerance = 1e-6
    );

    /**
     * @brief Position inverse kinematics (numerical iteration)
     * @param target_position    Target position (3x1)
     * @param max_iterations     Maximum number of iterations
     * @param tolerance          Convergence tolerance (m)
     * @param step_size          Step size factor
     * @param max_joint_step     Maximum joint change per step
     * @return true if converged, false otherwise
     */
    bool solve_inverse_kinematics(
        const Eigen::Vector3d& target_position,
        int max_iterations = 1000,
        double tolerance = 1e-4,
        double step_size = 0.2,
        double max_joint_step = 0.1
    );

    /**
     * @brief Normalize angle to [-PI, PI]
     */
    static double normalize_angle(double angle);

    /**
     * @brief Apply joint limits (clamp)
     */
    void apply_joint_limits();

private:
    /**
     * @brief Update MDH theta from current q
     * theta[i] = q[i] + theta_offset_[i]
     */
    void update_mdh_theta();

    std::array<double, 6> alpha_;        // Link twist angles
    std::array<double, 6> a_;            // Link lengths
    std::array<double, 6> d_;            // Link offsets
    std::array<double, 6> theta_;        // MDH joint angles (computed from q)
    std::array<double, 6> q_;            // Actual joint angles
    std::array<double, 6> theta_offset_; // MDH theta fixed offsets (default all zero)
    std::array<double, 6> joint_min_;    // Joint lower limits (default -1e9: unlimited)
    std::array<double, 6> joint_max_;    // Joint upper limits (default +1e9: unlimited)
};

} // namespace kinematics