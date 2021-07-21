import pylibmodbus as mod
import numpy as np
from time import sleep
from time import time as now
from threading import Thread, Lock
import socket
import sys
from bitFunctions import get_bit, set_bit, clear_bit, write_bit, write_float, write_32_bit_word, pw
from tcpIPFunctions import get_IP_address, connect_to_limit_switch, get_switch_status, connect_to_servo, read_servo, configure_servo
from tcpIPFunctions import get_heartbeat_status, get_heartbeat_response, set_heartbeat_response, set_servo_enable, get_servo_position, set_servo_position, set_start_move
from tcpIPFunctions import format_message, tag_server, set_motor_speed

ip_address = get_IP_address()
module_number = int(ip_address[-1])
print("knife #", module_number)

limit_switch = connect_to_limit_switch()
limit_switch_status, limit_switch = get_switch_status(limit_switch, module_number)

servo = connect_to_servo(module_number)
sero_input_registers, servo_output_registers, servo = read_servo(servo, both=True)
configure_servo(servo, sero_input_registers, servo_output_registers)
heartbeat_timeout = now()

even_loop = True
coded_message = 'hello'.encode('utf-8')
command_tags  = bytearray("B20E", 'utf-8')

previous_deviation_direction = 0

tags = { 'module' : module_number }
tags['enabled'] = False
tags['deviation'] = 0.0
tags['speed'] = 0.0
tags['program'] = 2
tags['trim'] = 0
tags['program command'] = 2
tags['trim command'] = 0
tags['stop'] = False
tags['timeout'] = False
tag_lock = Lock()

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_address = ('127.0.0.1', 8079)
server_socket.bind(server_address)
server_socket.listen()

while True: 
    tag_server_thread = Thread(target=tag_server, args=(tags,tag_lock))
    tag_server_thread.start()

    connection, client_address = server_socket.accept()
        
    while True:
        try:
            coded_message = connection.recv(1024)
            connection.sendall(command_tags)
        except:
            break
        
        tags = format_message(tags, tag_lock, coded_message)
        if tags['timeout']:
            continue
        elif tags['stop']:
            break

        command_tags[1] = tags['program command']
        command_tags[2] = tags['trim command'] + 127

        tags['enabled'], limit_switch = get_switch_status(limit_switch, tags['module'])

        sero_input_registers, servo = read_servo(servo)

        servo_output_registers = set_servo_enable(servo_output_registers, tags['enabled'])

        #if true, a heartbeat transition has occured
        if get_heartbeat_status(sero_input_registers) != get_heartbeat_response(servo_output_registers): 
            heartbeat_timeout = now()

        #if the heartbeat flag hasn't changed in 3 seconds, assume bad connnection and reset socket
        if (now()-heartbeat_timeout) > 3: 
            heartbeat_timeout = now()
            servo.close()
            servo = connect_to_servo(tags['module'])

        servo_output_registers = set_heartbeat_response(servo_output_registers, get_heartbeat_status(sero_input_registers))

        deviation_direction = tags['deviation'] / abs(tags['deviation'] + 0.0000001)
        if previous_deviation_direction != deviation_direction:
            tags['deviation'] = tags['deviation'] * 1.05
        previous_deviation_direction = tags['deviation'] / abs(tags['deviation'] + 0.0000001)

        current_position = get_servo_position(sero_input_registers)
        desired_position = current_position - tags['deviation']
        servo_output_registers = set_servo_position(servo_output_registers, desired_position)

        if tags['speed'] < 0.2:
            servo_output_registers =  set_motor_speed(servo_output_registers, 1.5, 3, 3)
        else:
            servo_output_registers =  set_motor_speed(servo_output_registers, 2, 4, 4)
        
        if even_loop:
            servo_output_registers = set_start_move(servo_output_registers, False)
            even_loop = False
        else:
            if tags['enabled'] and abs(tags['deviation']) > 0.01 and tags['speed'] > 0.05:
                servo_output_registers = set_start_move(servo_output_registers, True)
            even_loop = True
        
        servo.write_registers(0,servo_output_registers)
        #pw((sero_input_registers[44], servo_output_registers[2]))

limit_switch.close()
servo.close()
tag_server_thread.join()