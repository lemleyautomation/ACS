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
from tcpIPFunctions import format_message, tag_server

ip_address = get_IP_address()
module_number = int(ip_address[-1])
print("knife #", module_number)

limit_switch = connect_to_limit_switch()
limit_switch_status, limit_switch = get_switch_status(limit_switch, module_number)

servo = connect_to_servo(module_number)
sero_input_registers, servo_output_registers, servo = read_servo(servo, both=True)
configure_servo(servo, sero_input_registers, servo_output_registers)
heartbeat_timeout = now()

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_address = ('127.0.0.1', 8079)
server_socket.bind(server_address)
server_socket.listen(1)

even_loop = True
coded_message = 'hello'.encode('utf-8')
command_tags  = 'B20E'.encode('utf-8')
program = 2
trim = 0

connection_count = 0
while connection_count < 5:
    connection, client_address = server_socket.accept()
    connection_count = connection_count +1

    tags = [0,0,0,0,0,0,0]
    tag_lock = Lock()
    tag_server_thread = Thread(target=tag_server, args=(tags,tag_lock))
    tag_server_thread.start()
        
    while True:
        try:
            coded_message = connection.recv(100)
        except:
            continue
        
        module, deviation, speed, current_program, current_trim = format_message(coded_message)
        if module == -1:
            continue
        elif module == -2:
            tag_lock.acquire()
            tags[0] = -1
            tag_lock.release()
            break

        tag_lock.acquire()
        tags[0] = module
        tags[1] = deviation
        tags[2] = speed
        tags[3] = current_program
        tags[4] = current_trim
        program_command = tags[5]
        trim_command = tags[6]
        tag_lock.release()

        connection.sendall(command_tags)

        limit_switch_status, limit_switch = get_switch_status(limit_switch, module_number)
        sero_input_registers, servo = read_servo(servo)

        servo_output_registers = set_servo_enable(servo_output_registers, limit_switch_status)

        #if true, a heartbeat transition has occured
        if get_heartbeat_status(sero_input_registers) != get_heartbeat_response(servo_output_registers): 
            heartbeat_timeout = now()

        #if the heartbeat flag hasn't changed in 3 seconds, assume bad connnection and reset socket
        if (now()-heartbeat_timeout) > 3: 
            heartbeat_timeout = now()
            servo.close()
            servo = connect_to_servo(module_number)

        servo_output_registers = set_heartbeat_response(servo_output_registers, get_heartbeat_status(sero_input_registers))
        
        current_position = get_servo_position(sero_input_registers)
        desired_position = current_position - deviation
        servo_output_registers = set_servo_position(servo_output_registers, desired_position)
        
        if even_loop:
            servo_output_registers = set_start_move(servo_output_registers, False)
            even_loop = False
        else:
            if limit_switch_status == 1 and abs(deviation) > 0.01:
                servo_output_registers = set_start_move(servo_output_registers, True)
            even_loop = True
        
        servo.write_registers(0,servo_output_registers)
        #pw((sero_input_registers[44], servo_output_registers[2]))
        
limit_switch.close()
servo.close()
tag_server_thread.join()