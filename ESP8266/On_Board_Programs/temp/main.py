import network
import time
import socket


#setup TCP connection
addr_server = ('192.168.1.105',8001)
port = 8001
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.bind((ESP_IP,port)) #binding the port
#print('port 8001 binded')
 
#connect to Server
print('Connecting to Server...')
s.connect(addr_server)
print('Connected to Server')

#sending data
s.sendto(ESP_IP,addr_server)#sending IP of ESP
s.sendall('\nHello to Server from ESP \n')

#receiving data 
while True:   
# #    data,addr=s.recvfrom(1024)
# #    print('received:',data,'from',addr)
    data = s.recv(1024)
    data = data.decode('utf-8')
#    print('Received', data)
    print(data)

#print('done')

# from machine import UART
# uart = UART(0, baudrate=9600)
# 
# uart.write('UART Write Test')
# time.sleep(1)
# #uart.read(5) # read up to 5 bytes
