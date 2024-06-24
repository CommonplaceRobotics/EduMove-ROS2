#include "irc_ros_controllers/Velocity_PositionController.hpp"

namespace irc_ros_controllers
{
    /**
    * @brief Get configuration for controller's required command interfaces.
    */
    controller_interface::InterfaceConfiguration VelocityPositionController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (std::string name : joints_)
        {
            conf_names.emplace_back(name + "/position");            
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    /**
    * @brief Get configuration for controller's required state interfaces.
    */
    controller_interface::InterfaceConfiguration VelocityPositionController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for (std::string name : joints_)
        {
            conf_names.emplace_back(name + "/position");
        }

        return {controller_interface::interface_configuration_type::INDIVIDUAL, conf_names};
    }

    controller_interface::return_type VelocityPositionController::update(const rclcpp::Time & time, const rclcpp::Duration & period)
    {

        if (state_interfaces_.size() != command_interfaces_.size())
        {
            return controller_interface::return_type::ERROR;
        }

        if (latestVelCommand.size() != command_interfaces_.size())
        {
            RCLCPP_ERROR(rclcpp::get_logger("iRC_ROS_Controllers::VelocityPositionController"), "Invalid Command; command vector %li; interface length: %li", latestVelCommand.size(), command_interfaces_.size());
            return controller_interface::return_type::ERROR;
        }   

        std::vector<float> tempPos; 
        for(auto && si : state_interfaces_)
        {
            tempPos.emplace_back(si.get_value());
        }
        
        int cnt = 0;
        for(auto && ci : command_interfaces_)
        {
            ci.set_value(tempPos[cnt] + latestVelCommand[cnt] * 0.01);
            cnt++;
        }
        
        return controller_interface::return_type::OK;
    }
        
    controller_interface::CallbackReturn VelocityPositionController::on_configure(const rclcpp_lifecycle::State & previous_state)
    {   
        std::string name = get_node()->get_name();
        get_node()->get_parameter("joints", joints_);

        velocity_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(("~/commands"), 10, 
            std::bind(&VelocityPositionController::receive_velocity_callback_sub_, this, std::placeholders::_1));

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn VelocityPositionController::on_activate(const rclcpp_lifecycle::State & previous_state)
    {
        is_halted = false;
        subscriber_is_active_ = true;

        return controller_interface::CallbackReturn::SUCCESS;    
    }
    controller_interface::CallbackReturn VelocityPositionController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
    {
        subscriber_is_active_ = false;
        if (!is_halted)
        {
            halt();
            is_halted = true;
        }

        return controller_interface::CallbackReturn::SUCCESS;    
    }
    
    void VelocityPositionController::halt()
    {   
        std::vector<float> tempPos; 
        for(auto && si : state_interfaces_)
        {
            tempPos.emplace_back(si.get_value());
        }
        int cnt = 0;
        for (auto && ci : command_interfaces_)
        {   
            ci.set_value(tempPos[cnt]);
            cnt++;
        }

    }

    controller_interface::CallbackReturn VelocityPositionController::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }


    void VelocityPositionController::receive_velocity_callback_sub_(const std_msgs::msg::Float64MultiArray & jointVelocityCmd)
    {
        std::unique_lock<std::mutex> lock(cmdMutex);
        // latestVelCommand.clear();
        for (int i=0; i< latestVelCommand.size(); i++)
        {
            latestVelCommand[i] = jointVelocityCmd.data[i];
        }
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  irc_ros_controllers::VelocityPositionController, controller_interface::ControllerInterface)