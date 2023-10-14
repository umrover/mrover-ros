#pragma once

#include <string>

#include <ros/ros.h>

#include <can_manager.hpp>
#include <units/units.hpp>

namespace mrover {

    class Controller {
    public:
        Controller(ros::NodeHandle& nh, std::string const& name) : name{name}, can_manager{nh, name} {}

        virtual ~Controller() = default;

        // will receive CAN frame with updated motor information (current speed, position, etc.)
        virtual void update(std::span<std::byte const> frame) = 0;

        virtual void set_desired_throttle(Dimensionless throttle) = 0;    // from -1.0 to 1.0
        virtual void set_desired_velocity(RadiansPerSecond velocity) = 0; // joint output
        virtual void set_desired_position(Radians position) = 0;          // joint output

        CANManager& get_can_manager() {
            return can_manager;
        }

    protected:
        std::string name;
        CANManager can_manager;
        RadiansPerSecond velocity{};
        Radians position{};
        RadiansPerSecond min_velocity{}; // this is min_velocity of joint output
        RadiansPerSecond max_velocity{}; // this is max_velocity of joint output
        Radians min_position{};          // this is min_position of joint output
        Radians max_position{};          // this is min_position of joint output

        //    virtual void send_CAN_frame(uint64_t frame) = 0;
    };

} // namespace mrover
