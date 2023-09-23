import rospy
import random
from geometry_msgs.msg import Pose

def publish_pose():
    rospy.init_node('pose_publisher', anonymous=True)

    pose_publisher = rospy.Publisher('/first_topic', Pose, queue_size=10)
    while not rospy.is_shutdown():
        # Create a Pose message and populate its fields
        pose_msg = Pose()
        pose_msg.position.x = random.uniform(-100.0, 100.0)
        pose_msg.position.y = random.uniform(-100.0, 100.0)
        pose_msg.position.z = random.uniform(-100.0, 100.0)
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0

        rate = rospy.Rate(5)  # 10 Hz
        pose_publisher.publish(pose_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass
