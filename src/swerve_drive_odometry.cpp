#include "swerve_drive_controller/swerve_drive_odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"

namespace swerve_drive_controller
{
    SwerveDriveOdometry::SwerveDriveOdometry(std::shared_ptr<SwerveDriveKinematics> kinematics)
        : kinematics_{kinematics}
    {
    }

    bool SwerveDriveOdometry::configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
                                        std::string odom_fram_id, std::string base_frame_id,
                                        rsl::StaticVector<double, 6> pose_covariance_diagonal,
                                        rsl::StaticVector<double, 6> twist_covariance_diagonal)
    {
        odometry_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>(
            "~/odom", rclcpp::SystemDefaultsQoS());
        realtime_odometry_publisher_ =
            std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
                odometry_publisher_);

        auto &odom_message = realtime_odometry_publisher_->msg_;
        odom_message.header.frame_id = odom_fram_id;
        odom_message.child_frame_id = base_frame_id;
        odom_message.twist =
            geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);

        constexpr size_t NUM_DIMENSIONS = 6;
        for (size_t index = 0; index < 6; ++index)
        {
            // 0, 7, 14, 21, 28, 35
            const size_t diagonal_index = NUM_DIMENSIONS * index + index;
            odom_message.pose.covariance[diagonal_index] = *(pose_covariance_diagonal.begin() + index);
            odom_message.twist.covariance[diagonal_index] = *(twist_covariance_diagonal.begin() + index);
        }

        odometry_transform_publisher_ = node->create_publisher<tf2_msgs::msg::TFMessage>(
            "/tf", rclcpp::SystemDefaultsQoS());
        realtime_odometry_transform_publisher_ =
            std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
                odometry_transform_publisher_);

        auto &tf_message = realtime_odometry_transform_publisher_->msg_;
        tf_message.transforms.resize(1);
        tf_message.transforms.front().header.frame_id = odom_fram_id;
        tf_message.transforms.front().child_frame_id = base_frame_id;

        return true;
    }

    void SwerveDriveOdometry::setInitialModulePosition(std::vector<SwerveModulePosition> initialModulePosition)
    {
        previousModulePositions_.clear();
        previousModulePositions_.reserve(initialModulePosition.size());

        for (auto &modulePosition : initialModulePosition)
        {
            previousModulePositions_.emplace_back(modulePosition);
        }
    }

    void SwerveDriveOdometry::update(std::vector<SwerveModulePosition> newModulePositions)
    {
        std::vector<SwerveModulePosition> moduleDiffs;
        moduleDiffs.reserve(newModulePositions.size());
        for (size_t i = 0; i < newModulePositions.size(); i++)
        {
            moduleDiffs.emplace_back(SwerveModulePosition{
                previousModulePositions_[i].distance - newModulePositions[i].distance,
                previousModulePositions_[i].angle - newModulePositions[i].angle});
        }
        auto twist = kinematics_->to_twist(moduleDiffs);
        x_ += twist.linear.x * 1.0 / 30.0;
        y_ += twist.linear.y * 1.0 / 30.0;
        heading_ += twist.angular.z * 1.0 / 30.0;
    }

    bool SwerveDriveOdometry::publish(const rclcpp::Time &time)
    {
        bool success = true;

        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, getHeading());

        if (realtime_odometry_publisher_->trylock())
        {
            auto &odometry_message = realtime_odometry_publisher_->msg_;
            odometry_message.header.stamp = time;
            odometry_message.pose.pose.position.x = getX();
            odometry_message.pose.pose.position.y = getY();
            odometry_message.pose.pose.orientation.x = orientation.x();
            odometry_message.pose.pose.orientation.y = orientation.y();
            odometry_message.pose.pose.orientation.z = orientation.z();
            odometry_message.pose.pose.orientation.w = orientation.w();
            odometry_message.twist.twist = getTwist();

            realtime_odometry_publisher_->unlockAndPublish();

            success &= true;
        }
        else
        {
            success = false;
        }

        if (realtime_odometry_transform_publisher_->trylock())
        {
            auto &odometry_message = realtime_odometry_transform_publisher_->msg_;
            odometry_message.transforms.front().transform.translation.x = getX();
            odometry_message.transforms.front().transform.translation.y = getY();
            odometry_message.transforms.front().transform.rotation.x = orientation.x();
            odometry_message.transforms.front().transform.rotation.y = orientation.y();
            odometry_message.transforms.front().transform.rotation.z = orientation.z();
            odometry_message.transforms.front().transform.rotation.w = orientation.w();
            realtime_odometry_transform_publisher_->unlockAndPublish();
            success &= true;
        }
        else
        {
            success = false;
        }

        return success;
    }

    void SwerveDriveOdometry::exp(geometry_msgs::msg::Twist twist)
    {
        double dx = twist.linear.x;
        double dy = twist.linear.y;
        double dtheta = twist.angular.z;

        double sinTheta = sin(dtheta);
        double cosTheta = cos(dtheta);

        double s{};
        double c{};
        if (abs(dtheta) < 1E-9)
        {
            s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
            c = 0.5 * dtheta;
        }
        else
        {
            s = sinTheta / dtheta;
            c = (1 - cosTheta) / dtheta;
        }
    }

    double SwerveDriveOdometry::getX()
    {
        return x_;
    }

    double SwerveDriveOdometry::getY()
    {
        return y_;
    }

    double SwerveDriveOdometry::getHeading()
    {
        return heading_;
    }

    geometry_msgs::msg::Twist SwerveDriveOdometry::getTwist()
    {
        return twist_;
    }

} // namespace swerve_drive_controller
