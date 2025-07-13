#ifndef ROVER_HARDWARE_HPP_
#define ROVER_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"


#include "ros2_control_rover_hardware/esp_comms.hpp"
#include "ros2_control_rover_hardware/wheel.hpp"



namespace rover_hardware_
{
class RoverHardware : public hardware_interface::SystemInterface
{

// This struct holds the parameters for the hardware 
// component which are configured in the URDF or XACRO file.
struct rover_config
{
  std::string left_wheel_name = ""; 
  std::string right_wheel_name = "";
  std::string left_wheel_void_name = ""; 
  std::string right_wheel_void_name = "";
  float loop_rate = 0.0;  
  std::string serial_device = "";
  int baud_rate = 0;
  int timeout_ms = 0;
  int left_enc_counts_per_rev = 0;
  int right_enc_counts_per_rev = 0;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoverHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    ESPComms comms_;              ///< ESPComms instance for serial communication;
    rover_config sys_params_;     ///< Parameters for the rover system
    Wheel left_wheel_;            ///< Left wheel instance
    Wheel right_wheel_;           ///< Right wheel instance
}; 

}  // namespace rover_hardware_

#endif  // ROVER_HARDWARE_HPP_