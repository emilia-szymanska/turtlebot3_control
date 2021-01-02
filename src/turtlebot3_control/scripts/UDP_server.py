
import socket
from std_msgs.msg import String

LOCAL_IP        = "0.0.0.0"
LOCAL_PORT      = 20001
BUFFER_SIZE     = 1024
MSG_FROM_SERVER = "OK"
BYTES_TO_SEND   = str.encode(MSG_FROM_SERVER)


class UDPServer:
    def __init__(self):
        
        self.udp_server_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)   # create a datagram socket
        self.udp_server_socket.bind((LOCAL_IP, LOCAL_PORT))                                     # bind to address and ip
        
        self.command = rospy.Publisher('app_command', String, queue_size = 1)


    def run(self):
        while not rospy.is_shutdown():
            print("UDP server up and listening")
            rospy.spin()


# Listen for incoming datagrams
while(True):

    bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
    message = bytesAddressPair[0]
    address = bytesAddressPair[1]
    
    clientMsg = "Message from Client:{}".format(message)
    clientIP  = "Client IP Address:{}".format(address)
    
    print(clientMsg)
    print(clientIP)

    # Sending a reply to client
    UDPServerSocket.sendto(bytesToSend, address)




if __name__ == "__main__":
    rospy.init_node('udp_server')

    udp_server = UDPServer()

    try:
        udp_server.run()
    except rospy.ROSInterruptException:
        pass

