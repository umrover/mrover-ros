import pyglet
import rospy

class Xbox():
    def __init__(self):
        controllers = pyglet.input.get_controllers()
        if controllers:
            self.gamepad = controllers[0] # assumes Xbox is the first controller
            self.gamepad.open()
            rospy.logerr(self.gamepad)
            

    def run(self):
        pyglet.app.run()
    
    def get_gamepad_input(self):
        """
        Reads Xbox input and returns an array of axes and buttons
        """
        rospy.logerr(self.gamepad)
        axes = [
            self.gamepad.leftx,
            self.gamepad.lefty,
            self.gamepad.rightx,
            self.gamepad.righty,
            self.gamepad.lefttrigger,
            self.gamepad.righttrigger,
        ]
        buttons = [
            self.gamepad.a,
            self.gamepad.b,
            self.gamepad.x,
            self.gamepad.y,
            self.gamepad.leftshoulder,
            self.gamepad.rightshoulder, #can add more buttons (dpad, start, back)
        ]
        return axes, buttons