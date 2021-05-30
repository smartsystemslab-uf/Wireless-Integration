# In order to stop the server, in the terminal press CTRL+C and then on the app press disconnect

import socket
import network
#from mobile-cmd_conversion import convertToTwist

# create socket 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # TODO SOCK_STREAM is for TCP, we need to change to SOCK_DGRAM for udp
SERVER_HOSTNAME = socket.gethostname()
SERVER_IP = socket.gethostbyname(SERVER_HOSTNAME) 
print ("Socket successfully created. The server Hostname is", SERVER_HOSTNAME, "and the IP address is", SERVER_IP, "\n")
port = 8001
s.bind(('', port))
print ("Socket binded to port %s\n" %(port))
s.listen(1)
print ("Socket is listening... Connect to the server via a mobile app\n")


while True:
    c, addr = s.accept()
    print ('Got connection from', addr )
    c.send('Thank you for connecting\n'.encode('utf-8'))
    while True:
        try:
            data =c.recv(1024)
            data =data.decode('utf8')
            print( data)
            #convertToTwist(data) # TODO use imports to call function
            if not data:
                print("Disconnected from ", addr)
                break

        except KeyboardInterrupt:
            print("keyboard interrupted")
            s.close()
            exit()
        except Exception as error:
            print("Failed to insert record into Laptop table {}".format(error))
            exit()
                                                                                  
