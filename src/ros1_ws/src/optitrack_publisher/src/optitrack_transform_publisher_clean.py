import time
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
import zmq
import struct
from geometry_msgs.msg import PointStamped, Point, Quaternion, PoseStamped

def transform_new_base(base_pose, object_pose):
    quat_base = base_pose.pose.orientation
    quat_array_base = np.array([quat_base.x, quat_base.y, quat_base.z, quat_base.w])

    pos_base = base_pose.pose.position
    pos_array_base = np.array([pos_base.x, pos_base.y, pos_base.z])

    pos_object = object_pose.pose.position
    pos_array_object = np.array([pos_object.x, pos_object.y, pos_object.z])

    q = Rotation.from_quat(quat_array_base)
    R = q.as_matrix()

    M = np.column_stack([np.concatenate([R, pos_array_base]), [0, 0, 0, 1]])
    M_rot = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])

    objtemp = np.append(pos_array_object, 1)
    outobj = np.dot(np.dot(np.linalg.inv(M_rot), np.linalg.inv(M)), objtemp)

    object_position_transform = object_pose
    object_position_transform.header.frame_id = "object_transform"
    object_position_transform.pose.position = Point(*outobj[:3])
    
    return object_position_transform

def process_data(data):
    data_size = len(data)
    subdata_size = 36
    body_datas = []

    if data_size % subdata_size == 0:
        num_subdata = data_size // subdata_size
        for i in range(num_subdata):
            start = i * subdata_size
            end = (i + 1) * subdata_size
            subdata = data[start:end]
            body_data = struct.unpack('iffffffff', subdata)
            body_datas.append(body_data)
    else:
        print("Data size is not a multiple of the subdata size (36 bytes).")

    return body_datas

def publish_pose(publisher, body_data, frame_id):
    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame_id
    pose.pose.position = Point(body_data[2], body_data[3], body_data[4])
    pose.pose.orientation = Quaternion(body_data[6], body_data[7], body_data[8], body_data[5])
    publisher.publish(pose)

rospy.init_node("optitrack_client", anonymous=True)

base_pub = rospy.Publisher('base_position', PoseStamped, queue_size=1)
object_pub = rospy.Publisher('object_position', PoseStamped, queue_size=1)
object_transform_pub = rospy.Publisher('object_position_transform', PoseStamped, queue_size=1)

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.bind("tcp://0.0.0.0:5511")
socket.setsockopt(zmq.SUBSCRIBE, b"")

id_base = 17
id_object = 1001

r = rospy.Rate(1000)  # in Hz

while not rospy.is_shutdown():
    data = socket.recv()
    body_datas = process_data(data)

    for body_data in body_datas:
        if body_data[0] == id_base:
            publish_pose(base_pub, body_data, "base")

        if body_data[0] == id_object:
            publish_pose(object_pub, body_data, "object")

    if "base_position" in locals() and "object_position" in locals():
        object_transform_position = transform_new_base(base_position, object_position)
        object_transform_pub.publish(object_transform_position)

    r.sleep()

socket.close()
