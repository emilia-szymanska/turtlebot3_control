
import socket
from std_msgs.msg import String

LOCAL_IP         = "0.0.0.0"
LOCAL_PORT       = 20001
BUFFER_SIZE      = 1024
MSG_FROM_SERVER  = "OK"
START_MSG_CLIENT = "Hello UDP Server"
END_MSG_CLIENT   = "Bye UDP Server"
BYTES_TO_SEND    = str.encode(MSG_FROM_SERVER)


class UDPServer:
    def __init__(self):
        
        self.udp_server_socket = socket.socket(family = socket.AF_INET, type = socket.SOCK_DGRAM)       # create a datagram socket
        self.udp_server_socket.bind((LOCAL_IP, LOCAL_PORT))                                             # bind to address and ip
        
        self.command = rospy.Publisher('app_command', String, queue_size = 1)


    def run(self):
        
        print("UDP server up and listening")
        
        while not rospy.is_shutdown():

            # listen for incoming datagrams
            bytes_address_pair = UDPServerSocket.recvfrom(buffer_size)
            message = bytes_address_pair[0].decode()
                
            if message == START_MSG_CLIENT:

                address = bytes_address_pair[1]
    
                client_msg = "Message from Client:{}".format(message)
                client_ip  = "Client IP Address:{}".format(address)
    
                print(client_msg)
                print(client_ip)

                # sending a reply to client
                self.udp_server_socket.sendto(BYTES_TO_SEND, address)

                while rospy.is_shutdown():
                    
                    rx_command = UDPServerSocket.recvfrom(buffer_size)
                    command = bytes_address_pair[0].decode()
                    
                    if command == END_MSG_CLIENT:
                        break

                    self.command.publish(command)

            


if __name__ == "__main__":
    
    rospy.init_node('udp_server')
    udp_server = UDPServer()

    try:
        udp_server.run()
    except rospy.ROSInterruptException:
        pass
