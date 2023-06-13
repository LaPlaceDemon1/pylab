import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from fiducial_msgs.msg import FiducialTransformArray

"""
rospy.init_node('odometry_reader')
rospy.Subscriber('/pose', Odometry, odometry_callback)
rospy.spin()
"""


def pose_callback(msg):
    # Extract x, y, and yaw from the pose message
    x = msg.pose.position.x
    y = msg.pose.position.y
    quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    )
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    print("X: {}, Y: {}, Yaw: {}".format(x, y, yaw))



def odometry_callback(msg):
    global x, y, yaw

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    orientation = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    print(x)

# extract distance and angle from the camera from fiducial_tranfors topic


def aruco_callback(msg):
    global distance, angle
    distance = msg.transforms[0].transform.translation.x
    angle = msg.transforms[0].transform.translation.y
    print(distance, angle)


rospy.init_node('pioneer_odometry_node')

rospy.Subscriber('/pose', Odometry, odometry_callback)
rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, aruco_callback)

rospy.spin()