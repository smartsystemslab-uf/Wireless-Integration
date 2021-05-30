# in order to reset this program, delete the terminal and restart the script

# this script is for testing server connection to the esp8266

import socket
#from mobile-cmd_conversion import convertToTwist

# testing branch change

# create socket 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # TODO SOCK_STREAM is for TCP, we need to change to SOCK_DGRAM for udp
SERVER_HOSTNAME = socket.gethostname()
SERVER_IP = socket.gethostbyname(SERVER_HOSTNAME) 
print ("Socket successfully created. The server Hostname is", SERVER_HOSTNAME, "and the IP address is", SERVER_IP, "\n")
port = 8001
s.bind(('', port))
print ("Socket binded to port %s\n" %(port))
s.listen(1)
print ("Socket is listening... waiting for the ESP to connect\n")

while True:
    esp, ESP_IP = s.accept()
    print ('Got connection from', ESP_IP )
    esp.send('Thank you for connecting\n'.encode('utf-8'))
    try:
        data = esp.recv(1024)                                    # TODO this is going to be from the mobile app
        data = data.decode('utf-8')
        while True:
            print(data)
            if not data:
                print("Disconnected from ", ESP_IP)
                break
            esp.send('This is test message A\n'.encode('utf-8'))      # TODO for testing purposes
            esp.send('This is test message B\n'.encode('utf-8'))      # TODO for testing purposes
            

    except KeyboardInterrupt:
            print("keyboard interrupted")
            s.close()
            exit()
    except Exception as error:
            print("Failed to insert record into Laptop table {}".format(error))
            exit()
                                                                                  
