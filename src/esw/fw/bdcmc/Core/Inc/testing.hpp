//#pragma once
//
//namespace mrover {
//
//    auto get_test_msgs() -> std::vector<std::pair<InBoundMessage, uint32_t>> {
//        std::vector<std::pair<InBoundMessage, uint32_t>> tests;
//
//        // TODO add tests here
//
//        /*
//         * Tests to Add:
//         *
//         * 1. ConfigCommand (make sure this configures everything that needs to be tested)
//         * 2. EnableLimitSwitchesCommand
//         * 3. AdjustCommand
//         * 4. ThrottleCommand (Test Both Directions, Test Interrupt w/Limit Switches)
//         * 5. VelocityCommand (If PID Configured, Test Both Directions)
//         * 6. PositionCommand (If PID Configured)
//         * 7. IdleCommand
//         */
//        ConfigCommand configCommand = {
//                1.0,
////                ConfigLimitSwitchInfo,
////                ConfigEncoderInfo,
//                1.0,
//                1.0,
//                1.0,
//                1.0,
//                1.0
//        };
//        tests.push_back({configCommand, 2});
//
////        EnableLimitSwitchesCommand limitSwitchesCommand = { true };
////        tests.push_back({limitSwitchesCommand, 2});
////
////        AdjustCommand adjustCommand = { 1.0 };
////        tests.push_back({adjustCommand, 2});
////
////        ThrottleCommand throttleCommand = { 0.2 };
////        tests.push_back({throttleCommand, 2});
//
//        return tests;
//
//
//} // namespace mrover
