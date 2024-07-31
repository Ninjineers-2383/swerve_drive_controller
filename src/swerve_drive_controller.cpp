#include "swerve_drive_controller/swerve_drive_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace swerve_drive_controller
{
    controller_interface::CallbackReturn SwerveDriveController::on_init()
    {
        try
        {
            param_listener_ = std::make_shared<ParamListener>(get_node());
            params_ = param_listener_->get_params();
        }
        catch (std::exception &e)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn SwerveDriveController::on_configure(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        params_ = param_listener_->get_params();
        if (params_.steer_joint_names.size() != params_.wheel_joint_names.size())
        {
            RCLCPP_ERROR(get_node()->get_logger(), "The number of wheel joints [%zu] and steer joints [%zu] are different",
                         params_.wheel_joint_names.size(), params_.steer_joint_names.size());
            return controller_interface::CallbackReturn::ERROR;
        }

        velocity_command_subscriber = get_node()->create_subscription<DataType>(
            "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
            [this](const DataType::SharedPtr msg)
            {
                if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
                {
                    RCLCPP_WARN_ONCE(
                        get_node()->get_logger(),
                        "Received TwistStamped with zero timestamp, setting it to current "
                        "time, this message will only be shown once");
                    msg->header.stamp = get_node()->get_clock()->now();
                }
                RCLCPP_INFO(get_node()->get_logger(), "Received msg with timestamp %d", msg->header.stamp.sec);
                rt_buffer_ptr_.writeFromNonRT(msg);
            });

        std::vector<Eigen::Translation2d> moduleTranslations{4};

        moduleTranslations[0] = Eigen::Translation2d{0.3175, 0.3175};
        moduleTranslations[1] = Eigen::Translation2d{0.3175, -0.3175};
        moduleTranslations[2] = Eigen::Translation2d{-0.3175, 0.3175};
        moduleTranslations[3] = Eigen::Translation2d{-0.3175, -0.3175};

        kinematics = std::make_unique<SwerveDriveKinematics>(moduleTranslations);

        RCLCPP_INFO(this->get_node()->get_logger(), "configure successful");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration SwerveDriveController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (const auto &joint_name : params_.steer_joint_names)
        {
            conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
        }
        for (const auto &joint_name : params_.wheel_joint_names)
        {
            conf_names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::InterfaceConfiguration SwerveDriveController::state_interface_configuration() const
    {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE};
    }

    controller_interface::CallbackReturn SwerveDriveController::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // reset command buffer if a command came through callback when controller was inactive
        rt_buffer_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>>(nullptr);

        const auto result = register_wheels(params_.wheel_joint_names, params_.steer_joint_names, registered_wheel_handles);

        if (result != controller_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(get_node()->get_logger(), "Could not find matching wheel and steer joints");
            return result;
        }

        RCLCPP_INFO(this->get_node()->get_logger(), "activate successful");

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn SwerveDriveController::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/)
    {
        registered_wheel_handles.clear();
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type SwerveDriveController::update(
        const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
    {
        std::shared_ptr<DataType> last_command_msg = *(rt_buffer_ptr_.readFromRT());
        if (last_command_msg == nullptr)
        {
            return controller_interface::return_type::OK;
        }

        std::vector<SwerveModuleState> moduleStates = kinematics->to_module_states(last_command_msg->twist);

        for (size_t i = 0; i < moduleStates.size(); i++)
        {
            registered_wheel_handles[i].wheel.get().set_value(moduleStates.at(i).velocity);
            registered_wheel_handles[i].steer.get().set_value(moduleStates.at(i).angle);
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn SwerveDriveController::register_wheels(
        const std::vector<std::string> &wheel_names,
        const std::vector<std::string> &steer_names,
        std::vector<WheelHandle> &registered_handles)
    {
        auto logger = get_node()->get_logger();

        // register handles
        registered_handles.reserve(wheel_names.size());
        for (size_t i = 0; i < wheel_names.size(); i++)
        {
            auto wheel_name = wheel_names[i];
            auto wheel_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&wheel_name](auto &interface)
                {
                    return interface.get_prefix_name() == wheel_name &&
                           interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                });

            if (wheel_handle == command_interfaces_.cend())
            {
                RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            auto steer_name = steer_names[i];

            auto steer_handle = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [&steer_name](auto &interface)
                {
                    return interface.get_prefix_name() == steer_name &&
                           interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                });

            if (steer_handle == command_interfaces_.end())
            {
                RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", steer_name.c_str());
                return controller_interface::CallbackReturn::ERROR;
            }

            registered_handles.emplace_back(
                WheelHandle{std::ref(*wheel_handle), std::ref(*steer_handle)});
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(swerve_drive_controller::SwerveDriveController, controller_interface::ControllerInterface)