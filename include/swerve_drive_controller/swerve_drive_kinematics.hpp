#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/msg/twist.hpp>

namespace swerve_drive_controller
{
    struct SwerveModuleState
    {
        double velocity;
        double angle;
    };

    struct SwerveModulePosition
    {
        double distance;
        double angle;
    };

    class SwerveDriveKinematics
    {
    public:
        explicit SwerveDriveKinematics(const std::vector<Eigen::Translation2d> moduleLocations);

        std::vector<SwerveModuleState> to_module_states(geometry_msgs::msg::Twist chassisSpeeds);
        geometry_msgs::msg::Twist to_twist(std::vector<SwerveModulePosition> moduleDeltas);

    protected:
        std::vector<Eigen::Translation2d> moduleLocations;
        Eigen::HouseholderQR<Eigen::MatrixX3d> m_forwardKinematics;
        Eigen::MatrixX3d m_inverseKinematics;
    };
}