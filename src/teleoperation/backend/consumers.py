import json
from channels.generic.websocket import JsonWebsocketConsumer
import rospy
from sensor_msgs.msg import Joy

class GUIConsumer(JsonWebsocketConsumer):

    def connect(self):
        self.accept()

    def disconnect(self, close_code):

    def receive(self, text_data):
        """
        Receive message from WebSocket.
        """

        message = json.loads(text_data)
        if message['type'] == "joystick_values":
            self.handle_joystick_message(message)

# include calculations from wheel cmd velocity
    def handle_joystick_message(message):
        # split up message to get specific values
        # create an instance of message type Joy
        # assign the properties of message to 
        axes = message['axes']
        buttons = message['buttons']
        pub = rospy.Publisher('/joystick', Joy, queue_size=100)
        message = Joy()
        message.axes = axes
        message.buttons = buttons
        pub.publish(message)

        twist_pub = ros.Publisher("/cmd_vel", Twist, queue_size=100)
        joints: typing.Dict[str, JointState] = {}

        # Super small deadzone so we can safely e-stop with dampen switch
        dampen = deadzone(msg.axes[self.joystick_mappings["dampen"]], 0.01)

        # Makes dampen [0,1] instead of [-1,1]
        # negative sign required to drive forward by default instead of backward
        # (-1*dampen) because the top of the dampen switch is -1.0
        dampen = -1 * ((-1 * dampen) + 1) / 2

        linear = deadzone(
            msg.axes[self.joystick_mappings["forward_back"]] * self.drive_config["forward_back"]["multiplier"], 0.05
        )

        # Convert from [0,1] to [0, max_wheel_speed] and apply dampen
        linear *= self.max_wheel_speed * dampen

        # Deadzones for each axis
        left_right = (
            deadzone(
                msg.axes[self.joystick_mappings["left_right"]] * self.drive_config["left_right"]["multiplier"], 0.4
            )
            if self.drive_config["left_right"]["enabled"]
            else 0
        )
        twist = quadratic(
            deadzone(msg.axes[self.joystick_mappings["twist"]] * self.drive_config["twist"]["multiplier"], 0.1)
        )

        angular = twist + left_right

        # Same as linear but for angular speed
        angular *= self.max_angular_speed * dampen
        # Clamp if both twist and left_right are used at the same time
        if abs(angular) > self.max_angular_speed:
            angular = copysign(self.max_angular_speed, angular)
        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular

        self.twist_pub.publish(twist_msg)
