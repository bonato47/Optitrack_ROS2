
import socket
from dependencies import *


class UDPSocket(Thread):
    def __init__(self, role=None,
                 ip_address="127.0.0.1", port_number=20001, buffer_size=1024,
                 update_rate=0.05, is_log_enabled=False, q=None):
        Thread.__init__(self)
        self.role_ = role
        self.udp_socket_ = None
        self.ip_address_ = ip_address
        self.port_number_ = port_number
        self.buffer_size_ = buffer_size
        self.received_packet_ = None
        self.update_rate_ = update_rate
        self.is_log_enabled_ = is_log_enabled
        self.q_ = q
        self.init_udp_socket()

    def init_udp_socket(self):
        self.ip_address_and_port_number_ = (self.ip_address_,
                                            self.port_number_)
        self.udp_socket_ = socket.socket(family=socket.AF_INET,
                                         type=socket.SOCK_DGRAM)

    def bind(self):
        try:
            self.udp_socket_.bind(self.ip_address_and_port_number_)
            print("[OK] UDPSocket - bounded!")
        except:
            print("[EXCEPTION] UDPSocket - bind()")

    def connect(self):
        try:
            self.udp_socket_.connect(self.ip_address_and_port_number_)
            print("[OK] UDPSocket - connected!")
        except:
            print("[EXCEPTION] UDPSocket - connect()")

    def send_udp_packet(self, msg):
        try:
            if (self.is_log_enabled_):
                print(
                    f"[{self.role_}] sending ({msg}) to {self.ip_address_and_port_number_}")
            self.udp_socket_.sendto(str.encode(msg),
                                    self.ip_address_and_port_number_)
        except:
            print("[EXCEPTION] UDPSocket - send_udp_packet()")

    def receive_udp_packet(self):
        try:
            bytes_address_pair = self.udp_socket_.recvfrom(self.buffer_size_)
            message = ""
            message = bytes_address_pair[0]
            address = ""
            address = bytes_address_pair[1]
            if (message):
                self.received_packet_ = message.decode()
                if (self.is_log_enabled_):
                    print(
                        f"[{self.role_}] received message from: {address} is\n>> {message}")
                return True
            else:
                return False
        except:
            print("[EXCEPTION] UDPSocket - receive_udp_packet()")

    def get_received_packet(self):
        return self.received_packet_

    def run(self):
        while True:
            if (self.receive_udp_packet()):
                self.q_.put(self.get_received_packet())
            time.sleep(self.update_rate_)
