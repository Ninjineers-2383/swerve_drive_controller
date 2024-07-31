#include "swerve_drive_controller/swerve_drive_kinematics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

namespace swerve_drive_controller
{
    SwerveDriveKinematics::SwerveDriveKinematics(const std::vector<Eigen::Translation2d> moduleLocations)
        : moduleLocations{moduleLocations},
          m_inverseKinematics{4 * 2, 3}
    {
        m_inverseKinematics.resize(4 * 2, 3);
        Eigen::Array<double, 2, 3> location;
        location << 1, 0, 0,
            0, 1, 0;
        for (size_t i = 0; i < 4; i++)
        {
            location(0, 2) = -moduleLocations.at(i).y();
            location(1, 2) = moduleLocations.at(i).x();
            m_inverseKinematics.block<2, 3>(i * 2, 0) << location;
        }

        m_forwardKinematics = m_inverseKinematics.householderQr();
    }

    std::vector<SwerveModuleState> SwerveDriveKinematics::to_module_states(geometry_msgs::msg::Twist chassisSpeeds)
    {
        Eigen::Vector3d chassisSpeedsVector{chassisSpeeds.linear.x, chassisSpeeds.linear.y, chassisSpeeds.angular.z};

        Eigen::MatrixXd moduleStateMatrix = m_inverseKinematics * chassisSpeedsVector;

        std::vector<SwerveModuleState> moduleStates;
        for (size_t i = 0; i < 4; i++)
        {
            double x = moduleStateMatrix(i * 2, 0);
            double y = moduleStateMatrix(i * 2 + 1, 0);

            moduleStates.emplace_back(SwerveModuleState{sqrt(x * x + y * y), atan2(y, x)});
        }
        return moduleStates;
    }

    geometry_msgs::msg::Twist SwerveDriveKinematics::to_chassis_speeds(std::vector<SwerveModuleState> moduleStates)
    {
    }
}