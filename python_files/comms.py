import pylibmodbus as mod
import numpy as np
from time import time, sleep
import socket
import sys
from functions import get_bit, set_bit, clear_bit, write_bit, write_float, write_32_bit_word, pw

module_number = 6
program = 2
trim = 0

limit_switch = mod.ModbusTcp(ip="192.168.1.100", port=502)
limit_switch.connect()
limit_switch_response = limit_switch.read_registers(31,1)[0] #register 31, single word

servo = mod.ModbusTcp(ip=("192.168.1.3" + str(module_number)), port=502)
servo.connect()
servo_3x_words = servo.read_input_registers(0,54) #register 0, 54 words
servo_4x_words = servo.read_registers(0,54) #register 0, 54 words

try:
    tag_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    tag_server.settimeout(0.025)
    tag_server.connect(('192.168.1.21', 8080))
    tag_server.sendall('hello'.encode('utf-8'))
    #command_tags = tag_server.recv(100)
except:
    pass

servo_4x_words[0] = clear_bit(servo_4x_words[0], 0)   # make sure servo is off
servo_4x_words[0] = set_bit(servo_4x_words[0], 2)     # start homing
servo_4x_words[48] = 2                                  # servo decimal places
servo_4x_words[49] = 1                                  # servo homing type
servo_4x_words[51] = 1                                  # servo move type
servo_4x_words[18] = servo_3x_words[48]
servo_4x_words[19] = servo_3x_words[49]                 # set servo home position to current position
servo.write_registers(0,servo_4x_words)
sleep(0.5)
servo_4x_words[0] = clear_bit(servo_4x_words[0], 2)   # stop homing
servo_4x_words[20] = 400                                # set acceleration to 4 in/s^2
servo_4x_words[21] = 0
servo_4x_words[24] = 400                                # set deceleration to 4 in/s^2
servo_4x_words[25] = 0
servo_4x_words[28] = 200                                # set servo speed to 2 in/s
servo_4x_words[29] = 0
servo.write_registers(0,servo_4x_words)
heartbeat_timeout = time()

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_address = ('127.0.0.1', 8079)
server_socket.bind(server_address)
server_socket.listen(1)

while True:
    connection, client_address = server_socket.accept()
    try:
        even_loop = True
        coded_message = 'hello'.encode('utf-8')
        command_tags  = 'B20E'.encode('utf-8')
        while True:
            try:
                coded_message = connection.recv(100)
            except:
                break
            if coded_message:
                # these messages are coming from the vision program running on this same pc
                # a message looks like this: "6:1.23:0.1234:2:M"
                message = coded_message.decode('utf-8')   # convert from internet freindly byte array to string.
                message = message.split('M')        # the string may have multiple messages.
                message = message[0]                # Only the first one is likely to be uncorrupted
                message = message.split(':')        # split message into its separate variables
                module = 0
                speed = 0
                deviation = 0
                current_program = 0
                try:                                # if the message is corrupted, converting from string to float or int will fail.
                    module = message[0]
                    deviation = float(message[1])
                    speed = message[2]
                    current_program = message[3]
                    #print(message)
                except:
                    pass
                
                connection.sendall(command_tags)

                try:
                    limit_switch_response = limit_switch.read_registers(31,1)[0] #register 31, single word
                    limit_switch_status = get_bit(limit_switch_response, module_number-1)
                except: # if reading registers doesn't work, assume bad connection and reset socket
                    limit_switch.close()
                    limit_switch = mod.ModbusTcp(ip="192.168.1.100", port=502)
                    limit_switch.connect()
                
                try:
                    servo_3x_words = servo.read_input_registers(0,54) #register 0, 54 words
                    #f.pw((servo_3x_words[44],servo_4x_words[2]))

                    if get_bit(servo_3x_words[44],0) != get_bit(servo_4x_words[2],0): 
                        heartbeat_timeout = time()

                    if (time()-heartbeat_timeout) > 3: #if the heartbeat flag hasn't changed in 3 seconds, assume bad connnection and reset socket
                        print('reconnecting')
                        heartbeat_timeout = time()
                        servo.close()
                        servo = mod.ModbusTcp(ip=("192.168.1.3" + str(module_number)), port=502)
                        servo.connect()

                    servo_4x_words[0] = write_bit(servo_4x_words[0], 0, limit_switch_status) # servo enable

                    servo_4x_words[2] = write_bit(servo_4x_words[2], 0, get_bit(servo_3x_words[44],0)) #rebound heartbeat
                    
                    current_position = write_float(servo_3x_words[49], servo_3x_words[48], 2) # format current position from modbus words to float
                    desired_position = current_position - deviation
                    servo_4x_words[37], servo_4x_words[36] = write_32_bit_word(desired_position, 2) # format desired position from float to modbus words
                    
                    if even_loop:
                        servo_4x_words[0] = clear_bit(servo_4x_words[0], 3) # set start move to LOW
                        even_loop = False
                    else:
                        if limit_switch_status == 1 and abs(deviation) > 0.01:
                            servo_4x_words[0] = set_bit(servo_4x_words[0], 3) # set start move to HIGH
                        even_loop = True
                    
                    servo.write_registers(0,servo_4x_words)
                except:
                    servo.close()
                    servo = mod.ModbusTcp(ip=("192.168.1.3" + str(module_number)), port=502)
                    servo.connect()
                try:
                    #print ('sending tags')
                    tag_server.sendall(coded_message)
                except:
                    print ('tag server lost conection')
                    try:
                        tag_server.close()
                        tag_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        tag_server.settimeout(0.025)
                        tag_server.connect(('192.168.1.21', 8080))
                        print('tag server reconnected')
                    except:
                        pass
                try:
                    command_tags = tag_server.recv(100)
                    #print(command_tags)
                except:
                    print('tags not recieved')
                    pass
            else:
                break
    finally:
        connection.close()

tag_server.close()
limit_switch.close()
servo.close()
print("done")