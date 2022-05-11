# -*- coding: utf-8 -*-
"""
Created on Wed May 11 21:40:20 2022

@author: forti
"""

# import serial
# import time
# arduino = serial.Serial(port='COM6', baudrate=115200, timeout=.1)
# def write_read(x):
#     arduino.write(bytes(x, 'utf-8'))
#     time.sleep(0.05)
#     data = arduino.readline()
#     return data
# while True:
#     num = input("Enter a number: ") # Taking input from user
#     value = write_read(num)
#     print(value) # printing the value
    





##############
## Script listens to serial port and writes contents into a file
##############
## requires pySerial to be installed 
import serial  # sudo pip install pyserial should work

write_to_file_path = "output.txt";

output_file = open(write_to_file_path, "w+");
ser = serial.Serial(port='COM6', baudrate=115200, timeout=.1)
while True:
    line = ser.readline();
    line = line.decode("utf-8") #ser.readline returns a binary, convert to string
    print(line);
    output_file.write(line);