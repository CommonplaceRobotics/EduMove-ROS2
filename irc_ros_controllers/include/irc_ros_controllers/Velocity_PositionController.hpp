//Example Controller that receives velocity commands and controls the robot via the PositionInterface
//

#pragma once

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/msg/int16.h"

namespace irc_ros_controllers
{
    class VelocityPositionController : public controller_interface::ControllerInterface
    {
    public:
        /**
        * @brief Get configuration for controller's required command interfaces.
        */
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        /**
        * @brief Get configuration for controller's required state interfaces.
        */
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_init() override;

    private: 

        bool is_halted = false;
        bool subscriber_is_active_ = false;

        std::mutex cmdMutex;
        std::array<float, 6> latestVelCommand = {0.0f, 0.0f, 0.0f ,0.0f ,0.0f ,0.0f};
        
        std::vector<std::string> joints_;
        std::vector<std::string> jointPositions_;

        rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_command_subscriber_ = nullptr;

        void receive_velocity_callback_sub_(const std_msgs::msg::Float64MultiArray & jointVelocityCmd);

        void halt();
    };

} //namespace irc_ros_controllers