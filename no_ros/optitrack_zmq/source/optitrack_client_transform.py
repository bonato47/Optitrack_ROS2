import time
import numpy as np
from scipy.spatial.transform import Rotation
import rospy
import zmq
import struct
import sys

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point, Quaternion
from geometry_msgs.msg import PoseStamped

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

    object_position_transform.pose.position = Point(outobj[0], outobj[1], outobj[2]) # Shift optitrack origin to robot base
    return  object_position_transform
    
    
def main(id_base=None, id_object=None):
    
    
    if id_base is None:
        # id_base = int(input("Enter the ID for the base: "))
        id_base = 17
    if id_object is None:
        # id_object = int(input("Enter the ID for the object: "))
        id_object = 1001




# -------------------------------------------------------------------



# change this to publish in zmq
    # Init ROS
    rospy.init_node("optitrack_client", anonymous=True)#<-----------------------------------------------------------------ROS

    base_pub = rospy.Publisher('base_position_' + str(id_base), PoseStamped, queue_size=1)
    object_pub = rospy.Publisher('object_position_'+ str(id_object), PoseStamped, queue_size=1)
    object_transform_pub = rospy.Publisher('object_position_transform_'+ str(id_object), PoseStamped, queue_size=1)
    r = rospy.Rate(1000) # in Hz

# --------------------------------------------------------------------------

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    # socket.setsockopt(zmq.CONFLATE, 1) # commenting out else it get rids of ball position message
    socket.bind("tcp://0.0.0.0:5511")
    socket.setsockopt(zmq.SUBSCRIBE, b"")



    while not rospy.is_shutdown():#<-----------------------------------------------------------------ROS
    
        data = socket.recv()
        body_datas = process_data_chunks(data)
                
        
        for i in range(len(body_datas)):  
            # If we received base position
            body_data =body_datas[i]
            
            data_position = PoseStamped()
            data_position.header.stamp = rospy.Time.now()
            data_position.pose.position = Point(body_data[2], body_data[3], body_data[4]) # Shift optitrack origin to robot base
            # qx,qy,qz,qw
            data_position.pose.orientation = Quaternion(body_data[6], body_data[7], body_data[8],body_data[5]) # Shift optitrack origin to robot base

            if body_data[0] == id_base:
                data_position.header.frame_id = "base"+ str(id_base)
                base_position = data_position
                base_pub.publish(base_position) #<-----------------------------------------------------------------ROS

            # If we received object position
            if body_data[0] == id_object:
                data_position.header.frame_id = "object"+ str(id_object)
                object_position = data_position
                object_pub.publish(object_position)#<-----------------------------------------------------------------ROS
        
        object_transform_position= transform_new_base(base_position,object_position)
        
        object_transform_pub.publish(object_transform_position)#<-----------------------------------------------------------------ROS

        r.sleep() #<-----------------------------------------------------------------ROS

    socket.close()

if __name__ == '__main__':
    if len(sys.argv) == 3:
        id_base = int(sys.argv[1])
        id_object = int(sys.argv[2])
        main(id_base, id_object)
    else:
        main()

