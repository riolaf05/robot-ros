#include "arduino_hardware_interface.hpp"

#include <chrono>
#include <cstring>
#include <sstream>
#include <thread>
#include <errno.h>

namespace arduino_hardware_interface
{

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Leggi parametri
  device_ = info_.hardware_parameters.at("device");
  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  timeout_ = std::stod(info_.hardware_parameters.at("timeout")) / 1000.0;
  
  // Parametri opzionali
  wheel_radius_ = 0.065;  // 65mm default
  wheel_separation_ = 0.17;  // 170mm default

  // Verifica che ci siano esattamente 2 joint
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(logger_, "Expected exactly 2 joints, got %zu", info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Ottieni i nomi dei joint
  left_wheel_name_ = info_.joints[0].name;
  right_wheel_name_ = info_.joints[1].name;

  // Inizializza i vettori di stato
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  RCLCPP_INFO(logger_, "Arduino Hardware Interface initialized");
  RCLCPP_INFO(logger_, "Device: %s, Baud: %d", device_.c_str(), baud_rate_);
  RCLCPP_INFO(logger_, "Left wheel: %s, Right wheel: %s", 
              left_wheel_name_.c_str(), right_wheel_name_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArduinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "position", &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, "velocity", &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArduinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, "velocity", &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Activating Arduino Hardware Interface...");
  
  if (!connect_arduino()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Test connessione Arduino con comando baudrate
  send_command("b");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  
  // Reset encoders se presenti
  send_command("r");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  RCLCPP_INFO(logger_, "Arduino Hardware Interface activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArduinoHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Deactivating Arduino Hardware Interface...");

  // Ferma i motori con comando PWM
  send_motor_commands(0, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  disconnect_arduino();

  RCLCPP_INFO(logger_, "Arduino Hardware Interface deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ArduinoHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Simula movimento delle ruote in base ai comandi
  // In un sistema reale qui leggeresti gli encoder
  double dt = period.seconds();
  
  if (dt > 0) {
    // Simula il movimento basato sui comandi attuali
    for (size_t i = 0; i < hw_commands_.size(); i++) {
      hw_velocities_[i] = hw_commands_[i]; // Velocità = comando
      hw_positions_[i] += hw_velocities_[i] * dt; // Integra posizione
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArduinoHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // DEBUG: Logga velocità ricevute dai controller
  if (hw_commands_[0] != 0.0 || hw_commands_[1] != 0.0) {
    RCLCPP_INFO(logger_, "RECEIVED VELOCITIES: Left=%.3f rad/s, Right=%.3f rad/s", 
                hw_commands_[0], hw_commands_[1]);
  }
  
  // Converti velocità in PWM e invia comandi
  int left_pwm = velocity_to_pwm(hw_commands_[0]);
  int right_pwm = velocity_to_pwm(hw_commands_[1]);
  
  send_motor_commands(left_pwm, right_pwm);

  return hardware_interface::return_type::OK;
}

bool ArduinoHardwareInterface::connect_arduino()
{
  // Apri porta seriale con flag diversi per evitare lock
  serial_fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY);
  
  if (serial_fd_ == -1) {
    RCLCPP_ERROR(logger_, "Failed to open serial port: %s", device_.c_str());
    return false;
  }

  // Configura porta seriale in modo più robusto
  struct termios options;
  memset(&options, 0, sizeof(options));
  
  // Imposta baudrate
  speed_t speed;
  switch (baud_rate_) {
    case 9600:   speed = B9600;   break;
    case 19200:  speed = B19200;  break;
    case 38400:  speed = B38400;  break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:
      RCLCPP_ERROR(logger_, "Unsupported baud rate: %d", baud_rate_);
      close(serial_fd_);
      return false;
  }
  
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);
  
  // Configura parametri seriali - configurazione robusta
  options.c_cflag = CS8 | CLOCAL | CREAD;  // 8 bit, locale, enable receiver
  options.c_iflag = IGNPAR;                // Ignore parity errors
  options.c_oflag = 0;                     // Raw output
  options.c_lflag = 0;                     // Raw input
  
  // Timeout
  options.c_cc[VMIN] = 0;  // Non-blocking read
  options.c_cc[VTIME] = 1; // 0.1 second timeout

  // Applica configurazione immediatamente
  if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
    RCLCPP_ERROR(logger_, "Failed to configure serial port");
    close(serial_fd_);
    return false;
  }
  
  // Aspetta che Arduino si stabilizzi
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  // Pulisci buffer entrambe le direzioni
  tcflush(serial_fd_, TCIOFLUSH);
  
  // Test di connessione con comando semplice
  std::string test_cmd = "b\n";
  if (::write(serial_fd_, test_cmd.c_str(), test_cmd.length()) == -1) {
    RCLCPP_ERROR(logger_, "Failed to test Arduino connection");
    close(serial_fd_);
    return false;
  }

  RCLCPP_INFO(logger_, "Arduino connected successfully on %s at %d baud", device_.c_str(), baud_rate_);
  return true;
}

void ArduinoHardwareInterface::disconnect_arduino()
{
  if (serial_fd_ != -1) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool ArduinoHardwareInterface::send_command(const std::string& command)
{
  if (serial_fd_ == -1) return false;
  
  std::string cmd = command + "\n";
  size_t total_bytes = cmd.length();
  size_t bytes_sent = 0;
  int attempts = 0;
  const int max_attempts = 3;
  
  // Prova a inviare con retry per gestire "Partial write"
  while (bytes_sent < total_bytes && attempts < max_attempts) {
    ssize_t bytes_written = ::write(serial_fd_, cmd.c_str() + bytes_sent, total_bytes - bytes_sent);
    
    if (bytes_written < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Buffer pieno, aspetta un po'
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        attempts++;
        continue;
      } else {
        RCLCPP_ERROR(logger_, "Write error for command: %s, errno: %d", command.c_str(), errno);
        return false;
      }
    }
    
    bytes_sent += bytes_written;
    attempts++;
    
    // Se non abbiamo inviato tutto, aspetta un po'
    if (bytes_sent < total_bytes) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  
  if (bytes_sent < total_bytes) {
    RCLCPP_WARN(logger_, "Partial write for command: %s (%zu/%zu bytes)", command.c_str(), bytes_sent, total_bytes);
    return false;
  }
  
  // Forza l'invio immediato dei dati seriali (sync invece di flush)
  fsync(serial_fd_);
  
  return true;
}

void ArduinoHardwareInterface::send_motor_commands(int left_pwm, int right_pwm)
{
  // Limita valori PWM
  left_pwm = std::max(-255, std::min(255, left_pwm));
  right_pwm = std::max(-255, std::min(255, right_pwm));
  
  RCLCPP_INFO(logger_, "Sending motor commands: LEFT=%d, RIGHT=%d", left_pwm, right_pwm);
  
  // PROTOCOLLO CORRETTO per ROSArduinoBridge!
  // Comando: "o left_pwm right_pwm" per controllo PWM diretto
  std::string motor_cmd = "o " + std::to_string(left_pwm) + " " + std::to_string(right_pwm);
  
  if (send_command(motor_cmd)) {
    RCLCPP_INFO(logger_, "Motor PWM command sent: %s", motor_cmd.c_str());
  } else {
    RCLCPP_WARN(logger_, "Failed to send motor command: %s", motor_cmd.c_str());
  }
}

int ArduinoHardwareInterface::velocity_to_pwm(double velocity)
{
  // Converti velocità angolare rad/s in PWM
  const double max_velocity = 5.0; // rad/s massima
  double pwm = (velocity / max_velocity) * 255.0;
  return static_cast<int>(std::max(-255.0, std::min(255.0, pwm)));
}

}  // namespace arduino_hardware_interface

// Macro per registrare il plugin
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  arduino_hardware_interface::ArduinoHardwareInterface,
  hardware_interface::SystemInterface
)