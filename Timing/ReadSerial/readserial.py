# -*- coding: utf-8 -*-
"""
Created on Wed May 11 21:40:20 2022
參考 https://electronics.stackexchange.com/questions/54/saving-arduino-sensor-data-to-a-text-file

@author: forti
"""

import serial  # sudo pip install pyserial should work

write_to_file_path = "output.txt";

output_file = open(write_to_file_path, "w+");
ser = serial.Serial(port='COM6', baudrate=115200, timeout=.1)
ser.close()
ser.open()
for i in range(100):
    line = ser.readline();
    line = line.decode("utf-8") #ser.readline returns a binary, convert to string
    output_file.writelines(line);
    print(line);

output_file.close()
ser.close()