import unittest
import rospy
from std_msgs.msg import String
from butler_bot_interfaces.msg import HighLevelCommand
from geometry_msgs.msg import PoseStamped
import time

class TestButlerBotIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('butler_bot_integration_tester', anonymous=True)
        cls.voice_command_publisher = rospy.Publisher('audio_input', String, queue_size=10)
        cls.high_level_command_subscriber = rospy.Subscriber('high_level_commands', HighLevelCommand, cls.high_level_command_callback)
        cls.detected_objects_subscriber = rospy.Subscriber('detected_objects', String, cls.detected_object_callback)
        cls.vslam_pose_subscriber = rospy.Subscriber('vslam_pose', PoseStamped, cls.vslam_pose_callback)
        cls.semantic_map_subscriber = rospy.Subscriber('semantic_map', String, cls.semantic_map_callback)

        cls.received_high_level_command = None
        cls.received_detected_object = None
        cls.received_vslam_pose = None
        cls.received_semantic_map = None

        rospy.loginfo("Integration Test Node Initialized.")
        time.sleep(2) # Give time for publishers/subscribers to set up

    @classmethod
    def high_level_command_callback(cls, data):
        cls.received_high_level_command = data

    @classmethod
    def detected_object_callback(cls, data):
        cls.received_detected_object = data

    @classmethod
    def vslam_pose_callback(cls, data):
        cls.received_vslam_pose = data

    @classmethod
    def semantic_map_callback(cls, data):
        cls.received_semantic_map = data

    def test_voice_to_navigate_command(self):
        test_command = "navigate to kitchen"
        self.voice_command_publisher.publish(test_command)
        rospy.loginfo(f"Published voice command: {test_command}")
        time.sleep(3) # Wait for processing

        self.assertIsNotNone(self.received_high_level_command)
        self.assertEqual(self.received_high_level_command.command_type.data, "NAVIGATE")
        self.assertEqual(self.received_high_level_command.target_location.data, "kitchen")
        rospy.loginfo("Voice to navigation command test passed.")

    def test_object_detection_and_semantic_mapping(self):
        # This test relies on Isaac Sim object_detector publishing a "Cube"
        # and semantic_mapper processing it.
        time.sleep(5) # Give time for object detector and semantic mapper to run

        self.assertIsNotNone(self.received_detected_object)
        self.assertEqual(self.received_detected_object.data, "Cube")
        self.assertIsNotNone(self.received_semantic_map)
        self.assertIn("Cube', 'box_object', self.received_semantic_map) # Check for semantic mapping
        rospy.loginfo("Object detection and semantic mapping test passed.")

    def test_grasp_command_integration(self):
        # Simulate a grasp command directly to the task planner
        grasp_command = HighLevelCommand()
        grasp_command.command_type.data = "GRASP"
        grasp_command.target_object.data = "Cube"

        # Simulate detected object and its pose for the grasp integrator
        # (In a real scenario, these would come from the actual nodes)
        self.detected_object_callback(String(data="Cube"))
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "Cube"
        pose_msg.pose.position.x = 1.0
        pose_msg.pose.position.y = 1.0
        pose_msg.pose.position.z = 0.5
        self.object_pose_callback(pose_msg)

        self.voice_command_publisher.publish("pick up cup") # This will be processed by vla_task_planner
        time.sleep(3)

        # The actual grasping controller call is simulated in vla_grasp_integrator.py
        # We can only verify that the high-level command was processed.
        self.assertIsNotNone(self.received_high_level_command)
        self.assertEqual(self.received_high_level_command.command_type.data, "GRASP")
        self.assertEqual(self.received_high_level_command.target_object.data, "cup")
        rospy.loginfo("Grasp command integration test passed.")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('butler_bot_integration_test', 'test_suite', TestButlerBotIntegration)
