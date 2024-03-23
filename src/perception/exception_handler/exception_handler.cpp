#include "exception_handler.hpp"

namespace mrover{
    auto ExceptionHandler::onInit() -> void{
        //Get Node handelers
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
        ROS_INFO("WADDUP IM ALIVE");
    }
};