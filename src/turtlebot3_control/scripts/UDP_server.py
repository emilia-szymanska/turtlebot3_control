#!/usr/bin/env python3

import socket
import time
import rospy
from std_msgs.msg import String

LOCAL_IP         = "0.0.0.0"
LOCAL_PORT       = 20001
BUFFER_SIZE      = 1024
MSG_FROM_SERVER  = "OK"
START_MSG_CLIENT = "Hello UDP server"
END_MSG_CLIENT   = "Bye UDP server"
BYTES_TO_SEND    = str.encode(MSG_FROM_SERVER)
MAX_TIMEOUT      = 20.0                              # in seconds


class UDPServer:
    def __init__(self):
        # create a datagram socket
        self.udp_server_socket = socket.socket(family = socket.AF_INET, 
                                                type = socket.SOCK_DGRAM)               
        
        # bind to address and ip
        self.udp_server_socket.bind((LOCAL_IP, LOCAL_PORT))

        self.command = rospy.Publisher('app_command', String, queue_size = 1)


    def run(self):
        
        print("UDP server up and listening")
        
        while not rospy.is_shutdown():

            # listen for incoming datagrams
            print("waiting")
            bytes_address_pair = self.udp_server_socket.recvfrom(BUFFER_SIZE)
            print("something arrived")
            message = bytes_address_pair[0].decode()
            print(message)
                
            if message == START_MSG_CLIENT:
                print("Good message!")
                address = bytes_address_pair[1]
    
                print(f"Message from Client: {message}")
                print(f"Client IP Address: {address}")
    
                # sending a reply to client
                self.udp_server_socket.sendto(BYTES_TO_SEND, address)

                start = time.time()

                while not rospy.is_shutdown():
                    
                    rx_command = self.udp_server_socket.recvfrom(BUFFER_SIZE)
                    command = rx_command[0].decode()
                    end = time.time()

                    if command == END_MSG_CLIENT or end - start > MAX_TIMEOUT:
                        print("end of this connection")
                        break
                    
                    start = end
                    self.command.publish(command)

            


if __name__ == "__main__":
    
    rospy.init_node('udp_server')
    udp_server = UDPServer()

    try:
        udp_server.run()
    except rospy.ROSInterruptException:
        pass

