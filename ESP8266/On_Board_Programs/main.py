import network
import time
import socket


#test
# addr_info = socket.getaddrinfo("towel.blinkenlights.nl", 23)
# addr = addr_info[0][-1]
# s = socket.socket()
# s.connect(addr)
# 
# while True:
#     data = s.recv(500)
#     print(str(data, 'utf8'), end='')


#setup TCP connection
addr_server = ('192.168.1.102',8001)
port = 8001
s=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#s.bind((ESP_IP,port)) #binding the port
#print('port 8001 binded')
 
#connect to Server
print('Connecting to Server...')
s.connect(addr_server)
print('Connected to Server')

#sending data
#s.sendto(ESP_IP,addr_server)#sending IP of ESP
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