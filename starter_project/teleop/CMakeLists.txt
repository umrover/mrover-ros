## Generate messages in the 'msg' folder
# Uncomment this after adding your first msg to the msg folder

add_message_files(
        DIRECTORY
        ${CMAKE_CURRENT_LIST_DIR}/msg
        FILES
        Joystick.msg
        WheelCmd.msg
)