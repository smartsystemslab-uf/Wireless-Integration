# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
import uos, machine
#uos.dupterm(None, 1) # disable REPL on UART(0)
import gc
#import webrepl
#webrepl.start()
gc.collect()

import network

sta_if = network.WLAN(network.STA_IF)
ap_if = network.WLAN(network.AP_IF)

sta_if.active(True) #activate station interface
ap_if.active(True) #activate access point interface

print('\n\n\n')
print('Station (connnection to router) is active:',sta_if.active(),'\n')
print('Access Point (other devices connect to ESP) is active:',ap_if.active(),'\n')
    
#connect to the Wifi Network (router)
if not sta_if.isconnected():
    print('connecting to network...\n')
    sta_if.connect('TP-Link_SSL','72123379')
    while not sta_if.isconnected(): #loop until connected
        pass
    print('ESP connected to network:',sta_if.isconnected(),'\n')
    
#checking IP address
print(sta_if.ifconfig())
print('returned values are: IP address, netmask, gateway, DNS\n')
ESP_IP = sta_if.ifconfig()[0]
print('ESP IP address:',ESP_IP,'\n')    
    
