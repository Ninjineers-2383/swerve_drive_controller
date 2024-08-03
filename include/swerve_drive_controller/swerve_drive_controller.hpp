#pragma once

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "swerve_drive_controller/swerve_drive_kinematics.hpp"
#include "swerve_drive_controller/swerve_drive_odometry.hpp"

#include "swerve_drive_controller_parameters.hpp"

namespace swerve_drive_controller
{
    using DataType = geometry_msgs::msg::TwistStamped;
    class SwerveDriveController : public controller_interface::ControllerInterface
    {
    public:
        controller_interface::CallbackReturn on_init() override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    protected:
        struct WheelHandle
        {
            std::string name;
            Eigen::Translation2d translation;
            hardware_interface::LoanedCommandInterface *wheel;
            const hardware_interface::LoanedStateInterface *wheelFeedbackPosition;
            const hardware_interface::LoanedStateInterface *wheelFeedbackVelocity;
            hardware_interface::LoanedCommandInterface *steer;
            const hardware_interface::LoanedStateInterface *steerFeedbackPosition;
        };

        controller_interface::CallbackReturn register_wheels(
            const std::map<std::string, Params::Joints::MapWheelNames> jointsMap,
            std::vector<WheelHandle> &registered_handles);

        std::vector<SwerveModulePosition> getModulePositions(std::vector<WheelHandle> wheelHandles);

        bool registerCommandInterface(
            hardware_interface::LoanedCommandInterface *&handle, std::vector<hardware_interface::LoanedCommandInterface> &interfaces,
            std::string joint_name, std::string interface_name,
            const rclcpp::Logger &logger);
        bool registerStateInterface(
            const hardware_interface::LoanedStateInterface *&handle, std::vector<hardware_interface::LoanedStateInterface> &interfaces,
            std::string joint_name, std::string interface_name,
            const rclcpp::Logger &logger);

        std::vector<WheelHandle> registered_wheel_handles;

        std::shared_ptr<ParamListener> param_listener_;
        Params params_;

        realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>> rt_buffer_ptr_;
        rclcpp::Subscription<DataType>::SharedPtr velocity_command_subscriber;

        std::shared_ptr<SwerveDriveKinematics> kinematics;
        std::unique_ptr<SwerveDriveOdometry> odometry{nullptr};
    };
}