
global k1, k2, k3
k1 = k2 = k3 = 0


def test_communication(msg, udp_socket, t):
    global k1, k2, k3
    if (t > 0 and t <= 5):
        if (k1 < 5):
            udp_socket.send_udp_packet(msg[0])
        k1 = k1 + 1
    elif (t > 5 and t <= 25):
        if (k2 < 5):
            udp_socket.send_udp_packet(msg[1])
        k2 = k2 + 1
    elif (t > 25):
        if (k3 < 5):
            udp_socket.send_udp_packet(msg[2])
        k3 = k3 + 1

def generate_test_packet(pb_bio_sensors):
        # msg_hr = "N"
        msg_hr = "72*(265,369)"
        msg_ecg = "23*21*[1,2,3,4,5,6,7,8,9,10]"
        msg_pupil_0 = "0.55*25.0"
        msg_pupil_1 = "0.65*27.0"
        msg_blinks = "0.45*onset" 
        msg_opti_track = "[1,22334.45645,2,[10,20,20],[100,200,300,400]]"
        received_packet = msg_hr + ":" + msg_ecg + ":" + msg_pupil_0 + ":" + msg_pupil_1 + ":" + msg_blinks + ":" + msg_opti_track
        return received_packet

#  usage
    # print(delta_t, t)
    # test_communication(SERVER_MSGS, udp_socket, t)
    # received_packet = generate_test_packet(pb_bio_sensors)
    # msg = parse_ros_msg(received_packet, rospy.Time.now(), t)
    # send_ros_msg(pb_bio_sensors, msg)