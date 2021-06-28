import pylibmodbus as mod
import numpy as np
import time
import socket

module_number = 6

def get_bit(variable,index):
    return ((variable&(1<<index))!=0)

#limit_switch = mod.ModbusTcp(ip="192.168.1.100", port=502)
#limit_switch.connect()
#limit_switch_response = limit_switch.read_registers(31,1)[0] #register 31, single word
#limit_switch_status = get_bit(limit_switch_response, module_number-1)
#print(limit_switch_status)
#limit_switch.close()

#servo = mod.ModbusTcp(ip=("192.168.1.3" + str(module_number)), port=502)
#servo.connect()
#servo_3x_words = servo.read_input_registers(0,54) #register 0, 54 words
#servo_4x_words = servo.read_registers(0,54) #register 0, 54 words
#servo.close()

sock = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 8080))

while True:
    bytess = sock.recvfrom(1024)
    message = bytess[0]
    address = bytess[1]

    sock.sendto(str.encode("two"), address)

    print(message)

print("done")