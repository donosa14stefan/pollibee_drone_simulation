import unittest
import rospy
import rostest
import matplotlib.pyplot as plt

class TestVisualization(unittest.TestCase):
    def setUp(self):
        # Initialize ROS node
        rospy.init_node('test_visualization')

    def test_visualization(self):
        # Create a ROS subscriber to receive visualization data
        vis_sub = rospy.Subscriber('visualization_data', std_msgs.msg.Float32MultiArray)

        # Receive visualization data
        vis_data = vis_sub.recv()

        # Assert visualization data is not None
        self.assertIsNotNone(vis_data)

        # Check visualization data values
        self.assertGreaterEqual(len(vis_data.data), 1)  # at least one value in visualization data

        # Create a figure and axis for plotting
        fig, ax = plt.subplots()

        # Plot visualization data
        ax.plot(vis_data.data)

        # Assert plot is not empty
        self.assertIsNotNone(ax.lines)

        # Close the figure
        plt.close(fig)

    def test_visualization_update(self):
        # Create a ROS subscriber to receive visualization data
        vis_sub = rospy.Subscriber('visualization_data', std_msgs.msg.Float32MultiArray)

        # Receive initial visualization data
        vis_data_init = vis_sub.recv()

        # Assert initial visualization data is not None
        self.assertIsNotNone(vis_data_init)

        # Publish update command
        update_pub = rospy.Publisher('visualization_update', std_msgs.msg.Bool, queue_size=10)
        update_msg = std_msgs.msg.Bool()
        update_msg.data = True
        update_pub.publish(update_msg)

        # Receive updated visualization data
        vis_data_updated = vis_sub.recv()

        # Assert updated visualization data is not None
        self.assertIsNotNone(vis_data_updated)

        # Check updated visualization data values
        self.assertGreaterEqual(len(vis_data_updated.data), 1)  # at least one value in updated visualization data

        # Create a figure and axis for plotting
        fig, ax = plt.subplots()

        # Plot updated visualization data
        ax.plot(vis_data_updated.data)

        # Assert plot is not empty
        self.assertIsNotNone(ax.lines)

        # Close the figure
        plt.close(fig)

if __name__ == '__main__':
    rostest.rosrun('visualization', 'test_visualization', TestVisualization)
