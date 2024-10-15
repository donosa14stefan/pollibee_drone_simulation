import unittest
import rospy
import rostest

class TestPollinationMotor(unittest.TestCase):
    def setUp(self):
        # Initialize ROS node
        rospy.init_node('test_pollination_motor')

    def test_pollination_motor(self):
        # Create a ROS publisher to publish motor commands
        motor_pub = rospy.Publisher('motor_commands', std_msgs.msg.Float32, queue_size=10)

        # Create a ROS subscriber to receive motor state
        motor_state_sub = rospy.Subscriber('motor_state', std_msgs.msg.Float32)

        # Publish motor command
        motor_command = std_msgs.msg.Float32()
        motor_command.data = 50.0  # example motor command (50% duty cycle)
        motor_pub.publish(motor_command)

        # Receive motor state
        motor_state = motor_state_sub.recv()

        # Assert motor state is as expected
        self.assertAlmostEqual(motor_state.data, 50.0, delta=1.0)  # example expected motor state

if __name__ == '__main__':
    rostest.rosrun('pollination_motor', 'test_pollination_motor', TestPollinationMotor)
