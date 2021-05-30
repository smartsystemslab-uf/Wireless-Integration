#File uneeded, incorporated into the boot.py
import network
import time
import socket

sta_if = network.WLAN(network.STA_IF)
ap_if = network.WLAN(network.AP_IF)

print('Station (connnection to router) is active:',sta_if.active())
print('Acess point (other devices connect to ESP) is active:',ap_if.active())

#activate station interface
if not sta_if.active():
    sta_if.active(True)
    print('Station (connnection to router) is active:\n',sta_if.active())
    

#connect to the Wifi Network (router)
if not sta_if.isconnected():
    print('connecting to network...')
    sta_if.connect('TP-Link_SSL','72123379')
    print('Station is connected:',sta_if.isconnected()  )
          

#checking IP address
print('returned values are: IP address, netmask, gateway, DNS\n',sta_if.ifconfig())
ESP_IP = sta_if.ifconfig()[0]
print('ESP IP address:',ESP_IP,'\n')




