// Copyright 2026 Entan Zhang
// SPDX-License-Identifier: Apache-2.0

/**
 * @file kinematics.cpp
 * @brief Implementation of 6-DOF robotic arm kinematics library
 * Uses Craig Modified DH parameters:
 * A_i = Rx(alpha_{i-1}) * Tx(a_{i-1}) * Rz(theta_i) * Tz(d_i)
 */

#include "my_lib/kinematics.h"

namespace kinematics {

    /**
     * @brief Constructor, initializes with Modified DH parameters
     * @param alpha Link twist angles (6)
     * @param a     Link lengths (6)
     * @param d     Link offsets (6)
     */
    Kinematics::Kinematics(
        const std::array<double, 6>& alpha,
        const std::array<double, 6>& a,
        const std::array<double, 6>& d
    ) : alpha_(alpha), a_(a), d_(d) 
    {
        // Initialize joint angles to 0 (avoid uninitialized values)
        q_.fill(0.0);
        theta_.fill(0.0);
        theta_offset_.fill(0.0);
        joint_min_.fill(-1e9);
        joint_max_.fill(1e9);
    }

    /**
     * @brief Set current joint angles
     * @param q Six joint angles (radians)
     */
    void Kinematics::set_joint_angles(const std::array<double, 6>& q) {
        q_ = q;
        update_mdh_theta();
    }

    void Kinematics::set_theta_offsets(const std::array<double, 6>& offsets) {
        theta_offset_ = offsets;
        update_mdh_theta();
    }

    void Kinematics::set_joint_limits(
        const std::array<double, 6>& min_limits,
        const std::array<double, 6>& max_limits
    ) {
        joint_min_ = min_limits;
        joint_max_ = max_limits;
    }

    /**
     * @brief Get current joint angles
     * @return std::array<double, 6> Six joint angles (radians)
     */
    std::array<double, 6> Kinematics::get_joint_angles() const {
        return q_;
    }

    /**
     * @brief Rotation matrix about X axis
     * @param angle Rotation angle (radians)
     * @return Eigen::Matrix4d 4x4 homogeneous transformation matrix
     */
    Eigen::Matrix4d Kinematics::rotation_x(double angle) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = 
            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
        return transform;
    }

    /**
     * @brief Translation matrix along X axis
     * @param distance Translation distance
     * @return Eigen::Matrix4d 4x4 homogeneous transformation matrix
     */
    Eigen::Matrix4d Kinematics::translation_x(double distance) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform(0, 3) = distance;
        return transform;
    }

    /**
     * @brief Rotation matrix about Z axis
     * @param angle Rotation angle (radians)
     * @return Eigen::Matrix4d 4x4 homogeneous transformation matrix
     */
    Eigen::Matrix4d Kinematics::rotation_z(double angle) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = 
            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        return transform;
    }

    /**
     * @brief Translation matrix along Z axis
     * @param distance Translation distance
     * @return Eigen::Matrix4d 4x4 homogeneous transformation matrix
     */
    Eigen::Matrix4d Kinematics::translation_z(double distance) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform(2, 3) = distance;
        return transform;
    }


    /**
     * @brief Single Modified DH transformation
     * @param alpha Link twist angle
     * @param a     Link length
     * @param theta Joint angle
     * @param d     Link offset
     * @return Eigen::Matrix4d 4x4 homogeneous transformation matrix
     */
    Eigen::Matrix4d Kinematics::dh_transform(
        double alpha, 
        double a, 
        double theta, 
        double d
    ) const {
        return rotation_x(alpha) * translation_x(a) * 
            rotation_z(theta) * translation_z(d);
    }

    /**
     * @brief Update MDH theta from current q
     * theta[i] = q[i] + theta_offset_[i]
     */
    void Kinematics::update_mdh_theta() {
        theta_[0] = q_[0] + theta_offset_[0];
        theta_[1] = q_[1] + theta_offset_[1];
        theta_[2] = q_[2] + theta_offset_[2];
        theta_[3] = q_[3] + theta_offset_[3];
        theta_[4] = q_[4] + theta_offset_[4];
        theta_[5] = q_[5] + theta_offset_[5];
    }


    /**
     * @brief Compute total transformation from base_link to hand_link
     * @return Eigen::Matrix4d 4x4 homogeneous transformation matrix
     */
    Eigen::Matrix4d Kinematics::total_transform() const {
        Eigen::Matrix4d total = Eigen::Matrix4d::Identity();
        for (std::size_t i = 0; i < alpha_.size(); ++i) {
            total = total * dh_transform(alpha_[i], a_[i], theta_[i], d_[i]);
        }
        return total;
    }

    /**
     * @brief Get current end-effector position (extracted from total_transform)
     * @return Eigen::Vector3d End-effector position (x, y, z)
     */
    Eigen::Vector3d Kinematics::end_position() const {
        return total_transform().block<3, 1>(0, 3);
    }


    /**
     * @brief Compute 3x6 position Jacobian matrix
     * J_v_i = z_i × (p_e - p_i)
     * @return Eigen::Matrix<double, 3, 6> Position Jacobian matrix
     */
    Eigen::Matrix<double, 3, 6> Kinematics::position_jacobian() const {
        std::array<Eigen::Vector3d, 6> joint_axes;
        std::array<Eigen::Vector3d, 6> joint_origins;

        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

        for (std::size_t i = 0; i < alpha_.size(); ++i) {
            // Perform Rx(alpha) * Tx(a)
            const Eigen::Matrix4d joint_axis_transform = 
                transform * rotation_x(alpha_[i]) * translation_x(a_[i]);

            // Point on the joint axis (origin)
            joint_origins[i] = joint_axis_transform.block<3, 1>(0, 3);

            // Direction of the joint's local Z axis in the base_link frame
            joint_axes[i] = joint_axis_transform.block<3, 3>(0, 0) * 
                            Eigen::Vector3d::UnitZ();

            // Complete the current MDH transform: Rz(theta) * Tz(d)
            transform = joint_axis_transform * 
                        rotation_z(theta_[i]) * translation_z(d_[i]);
        }

        const Eigen::Vector3d end_effector_position = transform.block<3, 1>(0, 3);

        Eigen::Matrix<double, 3, 6> jacobian;
        for (std::size_t i = 0; i < joint_axes.size(); ++i) {
            jacobian.col(i) = joint_axes[i].cross(
                end_effector_position - joint_origins[i]
            );
        }

        return jacobian;
    }

    /**
     * @brief Compute Moore-Penrose pseudo-inverse (using SVD)
     * J = U Σ V^T
     * J+ = V Σ+ U^T
     * @param matrix             Input matrix
     * @param relative_tolerance Singular value truncation threshold (relative to max singular value)
     * @return Eigen::MatrixXd   Pseudo-inverse matrix
     * @throws std::invalid_argument If input matrix is empty
     */
    Eigen::MatrixXd Kinematics::pseudo_inverse(
        const Eigen::MatrixXd& matrix,
        double relative_tolerance
    ) {
        if (matrix.rows() == 0 || matrix.cols() == 0) {
            throw std::invalid_argument(
                "Cannot compute pseudoinverse of an empty matrix."
            );
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(
            matrix, 
            Eigen::ComputeThinU | Eigen::ComputeThinV
        );

        const Eigen::VectorXd singular_values = svd.singularValues();
        const double largest = singular_values(0);
        const double threshold = relative_tolerance * 
                                std::max(matrix.rows(), matrix.cols()) * 
                                largest;

        Eigen::VectorXd inverse_singular_values = 
            Eigen::VectorXd::Zero(singular_values.size());

        for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
            if (singular_values(i) > threshold) {
                inverse_singular_values(i) = 1.0 / singular_values(i);
            }
        }

        return svd.matrixV() * inverse_singular_values.asDiagonal() * 
            svd.matrixU().transpose();
    }


    /**
     * @brief Position inverse kinematics (numerical iteration)
     * e_k = p_target - p_current
     * delta_q = η · J+ · e_k
     * q_(k+1) = q_k + delta_q
     * @param target_position    Target position (3x1)
     * @param max_iterations     Maximum number of iterations
     * @param tolerance          Convergence tolerance (m)
     * @param step_size          Step size factor
     * @param max_joint_step     Maximum joint change per step (radians)
     * @return true if converged, false otherwise
     */
    bool Kinematics::solve_inverse_kinematics(
        const Eigen::Vector3d& target_position,
        int max_iterations,
        double tolerance,
        double step_size,
        double max_joint_step
    ) {
        for (int iteration = 0; iteration < max_iterations; ++iteration) {
            update_mdh_theta();

            const Eigen::Vector3d current_position = end_position();
            const Eigen::Vector3d error = target_position - current_position;
            const double error_norm = error.norm();

            if (error_norm < tolerance) {
                return true;
            }

            const Eigen::Matrix<double, 3, 6> jacobian = position_jacobian();
            const Eigen::MatrixXd jacobian_pinv = pseudo_inverse(jacobian);

            Eigen::Matrix<double, 6, 1> delta_q = 
                step_size * jacobian_pinv * error;

            // Limit single-step update to prevent oscillation
            const double delta_norm = delta_q.norm();
            if (delta_norm > max_joint_step) {
                delta_q *= max_joint_step / delta_norm;
            }

            // Limit projection: zero out joint components that push beyond limits,
            // preventing idle oscillation at boundary (root cause of previous deadlock)
            for (std::size_t i = 0; i < q_.size(); ++i) {
                if (q_[i] >= joint_max_[i] - 1e-9 && delta_q(i) > 0.0) {
                    delta_q(i) = 0.0;
                }
                if (q_[i] <= joint_min_[i] + 1e-9 && delta_q(i) < 0.0) {
                    delta_q(i) = 0.0;
                }
            }

            for (std::size_t i = 0; i < q_.size(); ++i) {
                q_[i] += delta_q(i);
            }

            apply_joint_limits();
        }

        return false;
    }


    /**
     * @brief Normalize angle to [-PI, PI]
     * @param angle Input angle (radians)
     * @return double Normalized angle
     */
    double Kinematics::normalize_angle(double angle) {
        while (angle > PI) {
            angle -= 2.0 * PI;
        }
        while (angle < -PI) {
            angle += 2.0 * PI;
        }
        return angle;
    }

    /**
     * @brief Apply joint limits (clamp)
     * Constrains q according to URDF joint limits:
     */
    void Kinematics::apply_joint_limits() {

        for (std::size_t i = 0; i < q_.size(); ++i) {
            if (joint_min_[i] <= -1e9 && joint_max_[i] >= 1e9) {
                /* continuous joint: normalize to [-PI, PI] */
                q_[i] = normalize_angle(q_[i]);
            } else {
                q_[i] = std::clamp(q_[i], joint_min_[i], joint_max_[i]);
            }
        }

        /* Limits may change q, sync theta to keep FK/Jacobian consistent */
        update_mdh_theta();
    }

} // namespace kinematics
