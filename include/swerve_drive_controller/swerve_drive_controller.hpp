#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/twist_stamped.hpp"

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
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> wheel;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> steer;
        };

        controller_interface::CallbackReturn register_wheels(
            const std::vector<std::string> &wheel_names,
            const std::vector<std::string> &steer_names,
            std::vector<WheelHandle> &registered_handles);

        std::vector<WheelHandle> registered_wheel_handles;

        std::shared_ptr<ParamListener> param_listener_;
        Params params_;

        realtime_tools::RealtimeBuffer<std::shared_ptr<DataType>> rt_buffer_ptr_;
        rclcpp::Subscription<DataType>::SharedPtr velocity_command_subscriber;
    };
}