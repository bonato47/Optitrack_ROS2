#!/usr/bin/env python3
# license removed for brevity

from dependencies import *
from utility import *
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

def my_exit():
    global logger, save_to_file
    print(Fore.RED)
    print('>> Ctrl+C is simulated!')
    print('>> Exiting...')
    print(Style.RESET_ALL)
    os._exit(0)

def signal_handler(sig, frame):
    my_exit()

def parse_ros_msg(packet):
    if (packet != "client_is_ready"):

        rev_packet = packet[1:-1]
        rev_packet = rev_packet.split(", ")
        rigid_body_count = int(rev_packet[1])
        rev_packet = rev_packet[2:]

        pose_array = PoseArray()
        pose_array.poses.clear() 

        list_ids = str()

        for i in range(rigid_body_count):
            list_ids += rev_packet[i*8]
            
            if (i != rigid_body_count - 1):
                list_ids += ","

            pose = Pose()
            pose.position.x = float(rev_packet[i*8 + 1])
            pose.position.y = float(rev_packet[i*8 + 2])
            pose.position.z = float(rev_packet[i*8 + 3])
            pose.orientation.x = float(rev_packet[i*8 + 4])
            pose.orientation.y = float(rev_packet[i*8 + 5])
            pose.orientation.z = float(rev_packet[i*8 + 6])
            pose.orientation.w = float(rev_packet[i*8 + 7])
            pose_array.poses.append(pose)
        
        pose_array.header.frame_id = list_ids
        return pose_array
        
    else: 
        return False

def main():
    # to end the program with ctrl+c
    signal.signal(signal.SIGINT, signal_handler)

    rospy.init_node('opti_track_ros_interface_node', anonymous=True)
    pub_opti_track = rospy.Publisher('/hasler/optitrackTrio', PoseArray, queue_size=2)
    rate = rospy.Rate(NODE_FREQUENCY)

    # read ROS parameters
    socket_role = rospy.get_param("~socket_role")
    udp_ip = rospy.get_param("~udp_ip")
    udp_port = rospy.get_param("~udp_port")
    udp_buffer_size = rospy.get_param("~udp_buffer_size")
    udp_update_rate = rospy.get_param("~udp_update_rate")
    is_print_log_enabled = rospy.get_param("~print_log")

    print(Fore.YELLOW)
    print("++ Experimental parameters:")
    print(f">> Socket role: {socket_role}")
    print(f">> UDP ip: {udp_ip}")
    print(f">> UDP port: {udp_port}")
    print(f">> UDP buffer size: {udp_buffer_size}")
    print(f">> UDP update rate: {udp_update_rate}")
    print(f">> Print log: {is_print_log_enabled}")
    print(Style.RESET_ALL)

    # create the udp socket thread (server for receiving data)
    q_received_udp_packet_server = queue.Queue()
    thrd_udp_socket_server = UDPSocket("SERVER", UDP_IPS[0], UDP_PORTS[0], UDP_PACKET_BUFFER_SIZE,
                                       UDP_UPDATE_RATE, UDP_LOG_ENABLED, q_received_udp_packet_server)
    thrd_udp_socket_server.bind()
    thrd_udp_socket_server.start()

    # create the udp socket thread (client for sending data)
    q_received_udp_packet_client = queue.Queue()
    thrd_udp_socket_client = UDPSocket("CLIENT", UDP_IPS[1], UDP_PORTS[1], UDP_PACKET_BUFFER_SIZE,
                                       UDP_UPDATE_RATE, UDP_LOG_ENABLED, q_received_udp_packet_client)
    thrd_udp_socket_client.connect()
    thrd_udp_socket_client.start()


    print(Fore.BLACK + Back.WHITE)
    print(">> Waiting for the first client message to establish the communication...")
    print(Style.RESET_ALL)

    while (q_received_udp_packet_server.empty()):
        time.sleep(0.05)

    # to empty the queue
    q_received_udp_packet_server.queue.clear()
    print("[OK] The first client message is received!")
    time.sleep(0.5)
        
    print(Fore.GREEN)
    print(f">> The main loop is started!")
    print(Style.RESET_ALL)    

    for i in range(5):
        thrd_udp_socket_client.send_udp_packet(SERVER_MSGS[1])

    while not rospy.is_shutdown():
        if (not q_received_udp_packet_server.empty()):
            received_packet = q_received_udp_packet_server.get()
           
            msg = parse_ros_msg(received_packet)
            if (msg):
                # print("sss")
                pub_opti_track.publish(msg)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        my_exit()
