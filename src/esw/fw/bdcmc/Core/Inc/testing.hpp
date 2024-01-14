#pragma once

namespace mrover {

    auto get_test_msgs() -> std::vector<std::pair<InBoundMessage, uint32_t>> {
        std::vector<std::pair<InBoundMessage, uint32_t>> tests;

        // TODO add tests here

        /*
         * Tests to Add:
         *
         * 1. ConfigCommand (make sure this configures everything that needs to be tested)
         * 2. EnableLimitSwitchesCommand
         * 3. AdjustCommand
         * 4. ThrottleCommand (Test Both Directions, Test Interrupt w/Limit Switches)
         * 5. VelocityCommand (If PID Configured, Test Both Directions)
         * 6. PositionCommand (If PID Configured)
         * 7. IdleCommand
         */


        return tests;
    }

} // namespace mrover
