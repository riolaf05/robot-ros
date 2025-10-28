#ifndef ARDUINO_HARDWARE_INTERFACE_HPP_
#define ARDUINO_HARDWARE_INTERFACE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Include per comunicazione seriale
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace arduino_hardware_interface
{

class ArduinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArduinoHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Comunicazione seriale
  int serial_fd_;
  std::string device_;
  int baud_rate_;
  double timeout_;
  
  // Parametri robot
  double wheel_radius_;
  double wheel_separation_;
  int enc_counts_per_rev_;
  
  // Thread per lettura encoder
  std::thread encoder_thread_;
  std::atomic<bool> keep_reading_;
  std::mutex data_mutex_;
  
  // Stato delle ruote
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  
  // Nomi dei joint
  std::string left_wheel_name_;
  std::string right_wheel_name_;
  
  // Metodi di supporto
  bool connect_arduino();
  void disconnect_arduino();
  bool send_command(const std::string& command);
  std::string read_response();
  void send_motor_commands(int left_pwm, int right_pwm);
  void read_encoders_loop();
  int velocity_to_pwm(double velocity);
  double ticks_to_position(int ticks);
  
  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("ArduinoHardwareInterface")};
};

}  // namespace arduino_hardware_interface

#endif  // ARDUINO_HARDWARE_INTERFACE_HPP_