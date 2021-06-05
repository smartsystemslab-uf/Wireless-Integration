# in order to reset this program, delete the terminal and restart the script 

# this script is for testing server connection to the esp8266 and only works with one ESP chip - will need to be serve multiple ESP clients

import socket
#from mobile-cmd_conversion import convertToTwist

# testing branch change

# create socket 
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # TODO SOCK_STREAM is for TCP, we need to change to SOCK_DGRAM for udp
SERVER_HOSTNAME = socket.gethostname()
SERVER_IP = socket.gethostbyname(SERVER_HOSTNAME) 
print ("Socket successfully created. The server Hostname is", SERVER_HOSTNAME, "and the Server IP address is", SERVER_IP, "\n")
port = 8001
s.bind(('', port))
print ("Socket binded to port %s\n" %(port))

# connect to ESP
s.listen(1)
print ("Socket is listening for the ESP to connect. Run the ESP program.\n")
esp, ESP_IP = s.accept()
print ('Got connection from the ESP at', ESP_IP )
esp.send('Thank you for connecting\n'.encode('utf-8'))
# receive confirmation message from ESP
data = esp.recv(1024)
data = data.decode('utf-8')
print(data)

# connect to the mobile app
s.listen(1)
print ("Socket is listening for the mobile app. Press connect on the mobile app\n")
app, APP_IP = s.accept()
print ('Got connection from the mobile app at', APP_IP )
app.send('Thank you for connecting\n'.encode('utf-8'))
print("Now send commands using the mobile app\n")

# main loop
while True:
    try:
        # main message loop
        app_cmd = app.recv(1024)
        app_cmd = app_cmd.decode('utf-8')
        if not data:
            print("Disconnected from ", APP_IP)
            break
        esp.send(app_cmd.encode('utf-8'))
        print("Command sent to the ESP:", app_cmd)

    except KeyboardInterrupt:
            print("keyboard interrupted")
            s.close()
            exit()
    except Exception as error:
            print("Failed to insert record into Laptop table {}".format(error))
            exit()
                                                                                    
