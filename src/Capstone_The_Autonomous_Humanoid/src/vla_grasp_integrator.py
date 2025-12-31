import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from butler_bot_interfaces.msg import HighLevelCommand
# In a real scenario, this would interface with the C++ GraspingController via ROS services

class VLAGraspIntegrator:
    def __init__(self):
        rospy.init_node('vla_grasp_integrator', anonymous=True)
        rospy.Subscriber('high_level_commands', HighLevelCommand, self.high_level_command_callback)
        rospy.Subscriber('detected_objects', String, self.detected_object_callback)
        rospy.Subscriber('object_poses', PoseStamped, self.object_pose_callback) # Assuming object poses are published
        rospy.loginfo("VLA Grasp Integrator Node Initialized.")

        self.current_high_level_command = None
        self.current_detected_objects = []
        self.object_poses = {}

    def high_level_command_callback(self, data):
        self.current_high_level_command = data
        rospy.loginfo(f"Grasp Integrator received high-level command: {data.command_type.data}")
        self.process_command_for_grasping()

    def detected_object_callback(self, data):
        detected_object_name = data.data
        if detected_object_name not in self.current_detected_objects:
            self.current_detected_objects.append(detected_object_name)
        rospy.loginfo(f"Grasp Integrator received detected object: {detected_object_name}")

    def object_pose_callback(self, data):
        object_id = data.header.frame_id # Assuming object ID is in frame_id
        self.object_poses[object_id] = data.pose
        rospy.loginfo(f"Grasp Integrator received pose for object: {object_id}")

    def process_command_for_grasping(self):
        if self.current_high_level_command and self.current_high_level_command.command_type.data == "GRASP":
            target_object = self.current_high_level_command.target_object.data
            rospy.loginfo(f"Attempting to grasp: {target_object}")

            if target_object in self.current_detected_objects:
                if target_object in self.object_poses:
                    target_pose = self.object_poses[target_object]
                    rospy.loginfo(f"Found pose for {target_object}: {target_pose.position.x}, {target_pose.position.y}, {target_pose.position.z}")
                    # In a real system, this would call a service of the GraspingController
                    rospy.loginfo("Simulating call to GraspingController to perform inverse kinematics and command gripper.")
                    # Simulate gripper command
                    # self.grasping_controller_service.call(target_pose) # Example service call
                else:
                    rospy.logwarn(f"Pose for {target_object} not available. Cannot grasp.")
            else:
                rospy.logwarn(f"{target_object} not currently detected. Cannot grasp.")

            # Reset command after processing
            self.current_high_level_command = None

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        integrator = VLAGraspIntegrator()
        integrator.run()
    except rospy.ROSInterruptException:
        pass
