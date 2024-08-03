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

    geometry_msgs::msg::Twist SwerveDriveKinematics::to_twist(std::vector<SwerveModulePosition> moduleDeltas)
    {
        auto moduleDeltaMatrix = Eigen::MatrixXd{};
        moduleDeltaMatrix.resize(moduleDeltas.size() * 2, 1);

        for (size_t i = 0; i < moduleDeltas.size(); i++)
        {
            moduleDeltaMatrix(i * 2, 0) = moduleDeltas[i].distance * cos(moduleDeltas[i].angle);
            moduleDeltaMatrix(i * 2 + 1, 0) = moduleDeltas[i].distance * sin(moduleDeltas[i].angle);
        }

        auto chassisForwardVector = m_forwardKinematics.solve(moduleDeltaMatrix);

        auto twist = geometry_msgs::msg::Twist{};
        twist.linear.x = chassisForwardVector(0, 0);
        twist.linear.y = chassisForwardVector(1, 0);
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = chassisForwardVector(2, 0);

        return twist;
    }
}