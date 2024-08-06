#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rsl/static_vector.hpp>
#include <manif/manif.h>

#include "swerve_drive_controller/swerve_drive_kinematics.hpp"

namespace swerve_drive_controller
{
    class SwerveDriveOdometry
    {
    public:
        SwerveDriveOdometry(std::shared_ptr<SwerveDriveKinematics> kinematics);

        bool configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                       std::string odom_fram_id,
                       std::string base_frame_id,
                       rsl::StaticVector<double, 6> pose_covariance_diagonal,
                       rsl::StaticVector<double, 6> twist_covariance_diagonal);

        void setInitialModulePosition(std::vector<SwerveModulePosition> initialModulePosition);

        void update(std::vector<SwerveModulePosition> newModulePositions);

        bool publish(const rclcpp::Time &time);

        double getX();
        double getY();
        double getHeading();
        geometry_msgs::msg::Twist getTwist();

    protected:
        std::shared_ptr<SwerveDriveKinematics> kinematics_;

        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odometry_publisher_ = nullptr;
        std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
            realtime_odometry_publisher_ = nullptr;

        std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>>
            odometry_transform_publisher_ = nullptr;
        std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
            realtime_odometry_transform_publisher_ = nullptr;

        manif::SE2d pose;

        std::vector<SwerveModulePosition> previous_module_positions_;
        geometry_msgs::msg::Twist previous_twist_;
    };
}