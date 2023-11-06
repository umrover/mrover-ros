#pragma once

#include <string>

#include <ros/ros.h>

#include <can_manager.hpp>
#include <units/units.hpp>

namespace mrover {

    class Controller {
    public:
        Controller(ros::NodeHandle& nh, std::string const& name, std::string const& controller_name)
            : m_name{name}, m_can_manager{nh, name} {}

        virtual ~Controller() = default;

        // TODO: receive information

        virtual void set_desired_throttle(Dimensionless throttle) = 0;    // from -1.0 to 1.0
        virtual void set_desired_velocity(RadiansPerSecond velocity) = 0; // joint output
        virtual void set_desired_position(Radians position) = 0;          // joint output

    protected:
        std::string m_name, m_controller_name;
        CANManager m_can_manager;
        RadiansPerSecond m_min_velocity{};
        RadiansPerSecond m_max_velocity{};
        Radians m_min_position{};
        Radians m_max_position{};

        //    virtual void send_CAN_frame(uint64_t frame) = 0;
    };

} // namespace mrover
