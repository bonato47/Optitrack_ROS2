import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation
import sys
import zmq
import struct
import numpy as np

class OptitrackClientNode(Node):

    def __init__(self,id_base,id_object):
        super().__init__('optitrack_client')

        self.publisher_base = self.create_publisher(PoseStamped, 'base_position_' +str(id_base), 1)
        self.publisher_object = self.create_publisher(PoseStamped, 'object_position_' + str(id_object), 1)
        self.publisher_transform = self.create_publisher(PoseStamped, 'object_position_transform_'+ str(id_object), 1)

    def fill_data(self,name,body_data):
        data_position = PoseStamped()
        data_position.header.frame_id = name
        data_position.header.stamp = self.get_clock().now().to_msg()
        # set the position
        data_position.pose.position.x =body_data[2]
        data_position.pose.position.y =body_data[3]
        data_position.pose.position.z =body_data[4]
        # set the orientation (quat)
        data_position.pose.orientation.w =body_data[5]
        data_position.pose.orientation.x =body_data[6]
        data_position.pose.orientation.y =body_data[7]
        data_position.pose.orientation.z =body_data[8]

        return   data_position 
    def publish_data(self, dataBase,dataObject,DataTransform):
        
        self.publisher_base.publish(dataBase)
        self.publisher_object.publish(dataObject)
        self.publisher_transform.publish(DataTransform)

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
    
    
    
def transform_new_base(basePoseStamped, objectPoseStamped):
    # Transform the quaternion to a rotation matrix
    quatBase = basePoseStamped.pose.orientation
    quat_array_base = np.array([quatBase.x, quatBase.y, quatBase.z, quatBase.w])
    
    posBase= basePoseStamped.pose.position
    position_array_base = np.array([posBase.x, posBase.y, posBase.z])
    
    posObject= objectPoseStamped.pose.position
    position_array_object = np.array([posObject.x, posObject.y, posObject.z])

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

    object_position_transform = objectPoseStamped
    object_position_transform.header.frame_id = "object_transform"

    object_position_transform.pose.position.x = outobj[0]
    object_position_transform.pose.position.y = outobj[1]
    object_position_transform.pose.position.z = outobj[2]
    return  object_position_transform
    
    
    
def main(id_base=None, id_object=None):
    
    if id_base is None:
        # id_base = int(input("Enter the ID for the base: "))
        id_base = 19
    if id_object is None:
        # id_object = int(input("Enter the ID for the object: "))
        id_object = 99
        
        
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    # socket.setsockopt(zmq.CONFLATE, 1) # commenting out else it get rids of ball position message
    socket.bind("tcp://0.0.0.0:5511")
    socket.setsockopt(zmq.SUBSCRIBE, b"")

        
    rclpy.init()

    optitrack_client_node = OptitrackClientNode(id_base,id_object)
    rate = optitrack_client_node.create_rate(1000)  # 1000 Hz

    init_object = False
    init_base = False

    try:
        while rclpy.ok():
                 
            data = socket.recv()
            body_datas = process_data_chunks(data)
            for i in range(len(body_datas)):  
                # If we received base position
                body_data =body_datas[i]
                

                if body_data[0] == id_base:
                    base_position= optitrack_client_node.fill_data("base"+ str(id_base),body_data)
                    init_base = True

                # If we received object position
                if body_data[0] == id_object:
                    object_position= optitrack_client_node.fill_data("base"+ str(id_object),body_data)
                    init_object = True
            
            # make the gransform if the other object are received
            if (init_base and init_object) == True:
                object_transform_position= transform_new_base(base_position,object_position)
                optitrack_client_node.publish_data(base_position,object_position,object_transform_position)  # Replace with your desired x, y, z values
            
            rclpy.spin_once(optitrack_client_node)
            rate.sleep()
    except KeyboardInterrupt:
        pass
    
    
    

    optitrack_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    if len(sys.argv) == 3:
        id_base = int(sys.argv[1])
        id_object = int(sys.argv[2])
        main(id_base, id_object)
    else:
        main()
