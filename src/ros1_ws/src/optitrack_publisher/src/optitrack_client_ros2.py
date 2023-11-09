import time
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
import zmq
import struct
import sys

from geometry_msgs.msg import PointStamped, Point, Quaternion, PoseStamped

def process_data_chunks(data, chunk_size=36, format_string='iffffffff'):
    """
    Process data into chunks of a specified size and format.

    Args:
        data (bytes): The data to process.
        chunk_size (int, optional): The size of each chunk in bytes. Default is 36.
        format_string (str, optional): The format string for unpacking the data. Default is 'iffffffff'.

    Returns:
        list: A list of unpacked data chunks.
    """
    data_size = len(data)
    body_datas = []

    if data_size % chunk_size == 0:
        num_chunks = data_size // chunk_size
        for i in range(num_chunks):
            start = i * chunk_size
            end = (i + 1) * chunk_size
            chunk_data = data[start:end]

            # Unpack and process each chunk
            chunk = struct.unpack(format_string, chunk_data)
            body_datas.append(chunk)
        return body_datas
    else:
        print("Data size is not a multiple of the chunk size ({} bytes).".format(chunk_size))
        return []

class OptitrackClient(Node):
    def __init__(self, id_base=None, id_object=None):
        super().__init__('optitrack_client')
        self.base_pub = self.create_publisher(PoseStamped, 'base_position', 1)
        self.object_pub = self.create_publisher(PoseStamped, 'object_position', 1)
        self.object_transform_pub = self.create_publisher(PoseStamped, 'object_position_transform', 1)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.bind("tcp://0.0.0.0:5511")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.r = self.create_rate(1000)
        self.id_base = id_base
        self.id_object = id_object

    def transform_new_base(self, base_pose_stamped, object_pose_stamped):
        # Transform the quaternion to a rotation matrix
        quat_base = base_pose_stamped.pose.orientation
        quat_array_base = np.array([quat_base.x, quat_base.y, quat_base.z, quat_base.w])

        pos_base = base_pose_stamped.pose.position
        position_array_base = np.array([pos_base.x, pos_base.y, pos_base.z])

        pos_object = object_pose_stamped.pose.position
        position_array_object = np.array([pos_object.x, pos_object.y, pos_object.z])

        q = Rotation.from_quat(quat_array_base)
        R = q.as_matrix()

        # Create the transformation matrix M
        M = np.array([
            [R[0, 0], R[0, 1], R[0, 2], position_array_base[0]],
            [R[1, 0], R[1, 1], R[1, 2], position_array_base[1]],
            [R[2, 0], R[2, 1], R[2, 2], position_array_base[2]],
            [0, 0, 0, 1]
        ])

        # Create the rotation matrix M_rot for a 90-degree rotation around the x-axis
        M_rot = np.array([
            [1, 0, 0, 0],
            [0, 0, -1, 0],
            [0, 1, 0, 0],
            [0, 0, 0, 1]
        ])

        # Define the 4D point objtemp
        objtemp = np.array([position_array_object[0], position_array_object[1], position_array_object[2], 1])
        # Calculate the transformed point outobj
        outobj = np.linalg.inv(M_rot) @ np.linalg.inv(M) @ objtemp

        object_position_transform = PoseStamped()
        object_position_transform.header.frame_id = "object_transform"
        object_position_transform.header.stamp = self.get_clock().now().to_msg()
        object_position_transform.pose.position = Point(outobj[0], outobj[1], outobj[2])
        return object_position_transform

    def run(self):
        while rclpy.ok():
            data = self.socket.recv()
            body_datas = process_data_chunks(data)
            
            for i in range(len(body_datas)):
                body_data = body_datas[i]
                data_position = PoseStamped()
                data_position.header.stamp = self.get_clock().now().to_msg()
                data_position.pose.position = Point(body_data[2], body_data[3], body_data[4])
                data_position.pose.orientation = Quaternion(body_data[6], body_data[7], body_data[8], body_data[5])

                if body_data[0] == self.id_base:
                    data_position.header.frame_id = "base"
                    base_position = data_position
                    self.base_pub.publish(base_position)

                if body_data[0] == self.id_object:
                    data_position.header.frame_id = "object"
                    object_position = data_position
                    self.object_pub.publish(object_position)

            object_transform_position = self.transform_new_base(base_position, object_position)
            self.object_transform_pub.publish(object_transform_position)

            self.r.sleep()

        self.socket.close()

def main(args=None):
    rclpy.init(args=args)
    if len(sys.argv) == 3:
        id_base = int(sys.argv[1])
        id_object = int(sys.argv[2])
        optitrack_client = OptitrackClient(id_base, id_object)
    else:
        optitrack_client = OptitrackClient()
    optitrack_client.run()
    rclpy.spin(optitrack_client)
    optitrack_client.shutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
