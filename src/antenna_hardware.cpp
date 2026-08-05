#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>

// Linux I2C & IO System includes
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <numbers>
#include <cerrno>

#ifndef MAG_ADDR
#define MAG_ADDR 0x0D
#endif

namespace umrt_com_antenna
{
class AntennaHardware : public hardware_interface::SystemInterface
{
public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
    {
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize state and command storage
        hw_position_state_ = 0.0;
        hw_velocity_command_ = 0.0;

        // Parse hardware parameters from the URDF <ros2_control> tag
        if (info.hardware_parameters.find("i2c_bus") != info.hardware_parameters.end()) {
            i2c_bus_ = info.hardware_parameters.at("i2c_bus");
        } else {
            i2c_bus_ = "/dev/i2c-1"; // Sensible default
        }

        declination_deg_ = get_param_or_default(info, "declination_deg", 0.0);
        
        // Calibration parameters
        offset_x_ = get_param_or_default(info, "offset_x", 0.0);
        offset_y_ = get_param_or_default(info, "offset_y", 0.0);
        offset_z_ = get_param_or_default(info, "offset_z", 0.0);
        scale_x_  = get_param_or_default(info, "scale_x", 1.0);
        scale_y_  = get_param_or_default(info, "scale_y", 1.0);
        scale_z_  = get_param_or_default(info, "scale_z", 1.0);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_activate(const rclcpp::LifecycleState & /*previous_state*/) override
    {
        // Open file descriptor on lifecycle activation
        i2c_fd_ = open(i2c_bus_.c_str(), O_RDWR);
        if (i2c_fd_ < 0) {
            RCLCPP_FATAL(rclcpp::get_logger("AntennaHardware"), "Failed to open I2C bus %s: %s", 
                         i2c_bus_.c_str(), std::strerror(errno));
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Setup and wake the sensor
        if (!configureSensor()) {
            RCLCPP_FATAL(rclcpp::get_logger("AntennaHardware"), "Sensor configuration failed.");
            close(i2c_fd_);
            i2c_fd_ = -1;
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("AntennaHardware"), "QMC5883P hardware driver successfully activated.");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn on_deactivate(const rclcpp::LifecycleState & /*previous_state*/) override
    {
        if (i2c_fd_ >= 0) {
            close(i2c_fd_);
            i2c_fd_ = -1;
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            "antenna_joint", hardware_interface::HW_IF_POSITION, &hw_position_state_));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            "antenna_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocity_command_));
        return command_interfaces;
    }

    hardware_interface::return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
    {
        if (i2c_fd_ < 0) {
            return hardware_interface::return_type::ERROR;
        }

        int16_t mag_raw[3];
        // Non-blocking status check built inside readMagRaw
        if (readMagRaw(mag_raw)) {
            // Apply hard-iron (offsets) and soft-iron (scale factors) calibrations
            double mx_cal = (static_cast<double>(mag_raw[0]) - offset_x_) * scale_x_;
            double my_cal = (static_cast<double>(mag_raw[1]) - offset_y_) * scale_y_;

            // Heading calculated from calibrated flat-mounted X/Y values
            double heading_rad = std::atan2(my_cal, mx_cal);
            double heading_deg = (heading_rad * 180.0 / std::numbers::pi) + declination_deg_;

            // Robust floating-point wrapping
            heading_deg = std::fmod(heading_deg, 360.0);
            if (heading_deg < 0.0) {
                heading_deg += 360.0;
            }

            // Expose the final physical position state in radians to your controller
            hw_position_state_ = heading_deg * std::numbers::pi / 180.0;
        }
        // If data was not ready on this cycle, we safely keep the last known positioning state

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
    {
        // 🚨 PLACE YOUR MOTOR ACTUATOR OUTPUT HERE 🚨
        // Pass 'hw_velocity_command_' directly down to your motor controller driver.
        // Example: serial_port_.write_speed(hw_velocity_command_);

        return hardware_interface::return_type::OK;
    }

private:
    // Helper to extract double params from URDF cleanly
    double get_param_or_default(const hardware_interface::HardwareInfo & info, const std::string & key, double default_val)
    {
        if (info.hardware_parameters.find(key) != info.hardware_parameters.end()) {
            return std::stod(info.hardware_parameters.at(key));
        }
        return default_val;
    }

    void readBlock(uint8_t reg, uint8_t * buf, size_t len)
    {
        i2c_msg msgs[2] = {};
        msgs[0] = {MAG_ADDR, 0, 1, &reg};
        msgs[1] = {MAG_ADDR, I2C_M_RD, static_cast<uint16_t>(len), buf};
        
        i2c_rdwr_ioctl_data ioctl_data{msgs, 2};
        if (ioctl(i2c_fd_, I2C_RDWR, &ioctl_data) < 0) {
            RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("AntennaHardware"), *rclcpp::Clock::make_shared(), 1000,
                                  "I2C read block failed at reg 0x%02X", reg);
        }
    }

    uint8_t readRegister(uint8_t reg)
    {
        uint8_t val = 0;
        readBlock(reg, &val, 1);
        return val;
    }

    void writeRegister(uint8_t reg, uint8_t value)
    {
        uint8_t buf[2] = {reg, value};
        i2c_msg msg = {};
        msg = {MAG_ADDR, 0, 2, buf};
        
        i2c_rdwr_ioctl_data ioctl_data{&msg, 1};
        if (ioctl(i2c_fd_, I2C_RDWR, &ioctl_data) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("AntennaHardware"), "I2C write failed (reg 0x%02X)", reg);
        }
    }

    bool configureSensor()
    {
        uint8_t chip_id = readRegister(0x00);
        RCLCPP_INFO(rclcpp::get_logger("AntennaHardware"), "QMC5883P CHIPID = 0x%02X (expect 0x80)", chip_id);
        
        writeRegister(0x0B, 0x80); // Reset
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        
        writeRegister(0x29, 0x06); // Axis sign setup sequence
        writeRegister(0x0B, 0x0C); // +-2 Gauss range, SET/RESET active
        writeRegister(0x0A, 0x0F); // 8x oversampling, 200Hz ODR, continuous
        return true;
    }

    bool readMagRaw(int16_t out[3])
    {
        uint8_t status = readRegister(0x09);
        if (!(status & 0x01)) {
            return false; // Non-blocking exit: Data not ready yet
        }
        
        uint8_t raw[6] = {0};
        readBlock(0x01, raw, 6);
        out[0] = static_cast<int16_t>((raw[1] << 8) | raw[0]);
        out[1] = static_cast<int16_t>((raw[3] << 8) | raw[2]);
        out[2] = static_cast<int16_t>((raw[5] << 8) | raw[4]);
        return true;
    }

    // Command and state registries
    double hw_position_state_;
    double hw_velocity_command_;

    // I2C Handles and Param Storage
    int i2c_fd_{-1};
    std::string i2c_bus_;
    double declination_deg_;
    
    // Calibration parameters
    double offset_x_, offset_y_, offset_z_;
    double scale_x_, scale_y_, scale_z_;
};
} // namespace umrt_com_antenna

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(umrt_com_antenna::AntennaHardware, hardware_interface::SystemInterface)
