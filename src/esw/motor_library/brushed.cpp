#include "brushed.hpp"

namespace mrover {

    void BrushedController::set_desired_throttle(Percent throttle) {
        if (!m_is_configured) {
            send_configuration();
            return;
        }

        assert(throttle >= -1_percent && throttle <= 1_percent);

        m_can_manager.send_message("devboard", InBoundMessage{ThrottleCommand{.throttle = throttle}});
    }

    void BrushedController::set_desired_position(Radians position) {
        if (!m_is_configured) {
            send_configuration();
            return;
        }

        assert(position >= m_min_position && position <= m_max_position);

        m_can_manager.send_message("devboard", InBoundMessage{PositionCommand{.position = position}});
    }

    void BrushedController::set_desired_velocity(RadiansPerSecond velocity) {
        if (!m_is_configured) {
            send_configuration();
            return;
        }

        assert(velocity >= m_min_velocity && velocity <= m_max_velocity);

        m_can_manager.send_message("devboard", InBoundMessage{VelocityCommand{.velocity = velocity}});
    }

    void BrushedController::send_configuration() {
        // TODO - need to set config_command
        m_can_manager.send_message("devboard", InBoundMessage{m_config_command});
    }

} // namespace mrover
