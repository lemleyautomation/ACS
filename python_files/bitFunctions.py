import pylibmodbus as mod
import numpy as np
from time import time, sleep
import socket
import sys

def get_bit(variable,index):
    return ((variable&(1<<index))!=0)

def set_bit(variable, index):
    return variable | (1<<index)
def clear_bit(variable, index):
    return variable & ~(1<<index)

def write_bit(variable, index, status):
    if status == True:
        return set_bit(variable, index)
    else:
        return clear_bit(variable,index)

def write_32_bit_word(value, decimal_places):
    int_value = int(value*(10**decimal_places))
    lower_word = 0
    upper_word = 65535*(value<0)
    for i in range(16):
        lower_word = write_bit(lower_word, i, get_bit(int_value, i))
    return upper_word, lower_word

def write_float(upper_word, lower_word, decimal_places):
    value = 0
    for i in range(32):
        if i < 16:
            value = write_bit(value, i, get_bit(lower_word, i))
        else:
            value = write_bit(value, i, (get_bit(upper_word,15)))
    if get_bit(upper_word,15) == 1:
        value =  value - 4294967296
    return value/(10**decimal_places)

def pw(words):
    f_words = []

    for word in words:
        f_word = "b"
        for i in range(16):
            f_word = f_word + str(int(get_bit(word,15-i)))
        f_words.append(f_word)
    
    print(f_words)