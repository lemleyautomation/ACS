import pylibmodbus as mod
import numpy as np
from time import time, sleep
import socket
import sys
from bitFunctions import get_bit, set_bit, clear_bit, write_bit, write_float, write_32_bit_word, pw

def get_IP_address():
    # Unless we try to connect to something, the IP address we get will just be 'localhost'
    # but the connection doesn't have to work, it just needs to bind to the ethernet driver
    dummy_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    dummy_socket.settimeout(0.001)
    try:
        dummy_socket.connect(('10.10.10.10', 1))
    except:
        pass
    ip_address = dummy_socket.getsockname()
    ip_address = ip_address[0]
    dummy_socket.close()
    return ip_address

def connect_to_limit_switch():
    limit_switch = mod.ModbusTcp(ip="192.168.1.100", port=502)
    limit_switch.connect()
    return limit_switch

def get_switch_status(limit_switch, module_number):
    limit_switch_status = 0
    try:
        limit_switch_response = limit_switch.read_registers(31,1)[0] #register 31, single word
        limit_switch_status = get_bit(limit_switch_response, module_number-1)
    except:
        limit_switch.close()
        limit_switch = connect_to_limit_switch()
    return limit_switch_status, limit_switch

def connect_to_servo(module_number):
    servo = mod.ModbusTcp(ip=("192.168.1.3" + str(module_number)), port=502)
    servo.connect()
    return servo

def read_servo(servo, both=False):
    servo_input_registers = 0
    servo_output_registers = 0
    try:
        if both:
            servo_input_registers = servo.read_input_registers(0,54) #register 0, 54 words
            servo_output_registers = servo.read_registers(0,54) #register 0, 54 words
        else:
            servo_input_registers = servo.read_input_registers(0,54) #register 0, 54 words
    except:
        servo.close()
        servo = connect_to_servo()
    if both:
        return servo_input_registers, servo_output_registers, servo
    else:
        return servo_input_registers, servo
    
def configure_servo(servo, servo_input_registers, servo_output_registers):
    servo_output_registers[0] = clear_bit(servo_output_registers[0], 0)     # make sure servo is off
    servo_output_registers[0] = set_bit(servo_output_registers[0], 2)       # start homing
    servo_output_registers[48] = 2                                  # servo decimal places
    servo_output_registers[49] = 1                                  # servo homing type
    servo_output_registers[51] = 1                                  # servo move type
    servo_output_registers[18] = servo_input_registers[48]
    servo_output_registers[19] = servo_input_registers[49]                 # set servo home position to current position
    servo.write_registers(0,servo_output_registers)
    sleep(0.5)
    servo_output_registers[0] = clear_bit(servo_output_registers[0], 2)     # stop homing
    servo_output_registers[20] = 400                                # set acceleration to 4 in/s^2
    servo_output_registers[21] = 0
    servo_output_registers[24] = 400                                # set deceleration to 4 in/s^2
    servo_output_registers[25] = 0
    servo_output_registers[28] = 200                                # set servo speed to 2 in/s
    servo_output_registers[29] = 0
    servo.write_registers(0,servo_output_registers)
    return

def get_heartbeat_status (servo_input_registers):
    return get_bit(servo_input_registers[44],0)

def get_heartbeat_response(servo_output_registers):
    servo_output_registers[2] = get_bit(servo_output_registers[2],0)
    return servo_output_registers

def set_heartbeat_response(servo_output_registers, value):
    servo_output_registers[2] = write_bit(servo_output_registers[2], 0, value) #rebound heartbeat
    return  servo_output_registers

def set_servo_enable(servo_output_registers, limit_switch_status):
    servo_output_registers[0] = write_bit(servo_output_registers[0], 0, limit_switch_status) # servo enable
    return servo_output_registers

def get_servo_position(servo_input_registers):
    return write_float(servo_input_registers[49], servo_input_registers[48], 2) # format current position from modbus words to float

def set_servo_position(servo_output_registers, desired_position):
    servo_output_registers[37], servo_output_registers[36] = write_32_bit_word(desired_position, 2) # format desired position from float to modbus words
    return servo_output_registers

def set_start_move(servo_output_registers, status):
    if status:
        servo_output_registers[0] = set_bit(servo_output_registers[0], 3) # set start move to LOW
    else:
        servo_output_registers[0] = clear_bit(servo_output_registers[0], 3) # set start move to LOW
    return servo_output_registers

def format_message(coded_message):
    if (len(coded_message) == 0):
        return -2, -2, -2, -2, -2
    # these messages are coming from the vision program running on this same pc
    # a message looks like this: "6:1.23:0.1234:2:M"
    message = coded_message.decode('utf-8')   # convert from internet freindly byte array to string.
    message = message.split('M')        # the string may have multiple messages.
    message = message[0]                # Only the first one is likely to be uncorrupted
    message = message.split(':')        # split message into its separate variables
    try:                                # if the message is corrupted, converting from string to float or int will fail.
        module = int(message[0])
        deviation = float(message[1])
        speed = float(message[2])
        current_program = int(message[3])
        current_trim = int(message[4])
        return module, deviation, speed, current_program, current_trim
    except:
        return -1, -1, -1, -1, -1