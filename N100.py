import serial.tools.list_ports
import struct
import time
import crcmod.predefined
import math
import numpy as np
import csv
import pandas as pd

df_head = str('fc')
df_end = str('fd')
addr_imu = str('40')
addr_ahrs = str('41')
imu_len = str('38')
ahrs_len = str('30')

serial_port = str('com6')
serial_bps = int(921600)
serial_to = int(3)

scale1=[1.0037140204271124,0.9600355239786856,1.039423076923077]
offset1=[-28.875,273.75,306.0]

def receive_data():
    try:
        while True:
            check_head = ser.read().hex()
            if check_head != df_head:
                continue
            addr = ser.read().hex()
            len = ser.read().hex()
            check_sn = ser.read().hex()
            head_crc8 = ser.read().hex()
            crc16_H_s = ser.read().hex()
            crc16_L_s = ser.read().hex()
            
            check_headb = bytes.fromhex(check_head)
            addrb = bytes.fromhex(addr)
            lengthb = bytes.fromhex(len)
            check_snb = bytes.fromhex(check_sn)
            
            if addr == addr_imu:
                data_s = ser.read(int(imu_len, 16))
                hex_list = [hex(byte) for byte in data_s]
                
                combined_header = check_headb + addrb + lengthb + check_snb
                crc8_calculated = crc8_maximsss(combined_header)
                combined_load = crc16_H_s + crc16_L_s
                crc16_calculated = calculate_crc16(data_s)
                tail = ser.read().hex()
                
                if(crc8_calculated == int(head_crc8, 16)):
                    magnetometer_x = struct.unpack('f', data_s[24:28])[0]
                    magnetometer_y = struct.unpack('f', data_s[28:32])[0]
                    magnetometer_z = struct.unpack('f', data_s[32:36])[0]
                    x = magnetometer_x*scale1[0]-offset1[0]
                    y = magnetometer_y*scale1[1]-offset1[0]
                    z = magnetometer_z*scale1[2]-offset1[2]
                    
                    heading_rad = math.atan2(y, x)
                    heading_deg = math.degrees(heading_rad)
                    if heading_deg < 0:
                        heading_deg += 360
                    heading_deg = (360 - heading_deg) % 360
                         
                    print("X:",x,"Y:",y)
                    print("Heading (degrees):", heading_deg)

            elif addr == addr_ahrs:
                data_s = ser.read(int(ahrs_len, 16))
            
            else:
                continue
                
    except KeyboardInterrupt :
        ser.close()
        exit(1)

def crc8_maximsss(data):
    crc8_func = crcmod.predefined.mkPredefinedCrcFun("crc-8-maxim")
    return crc8_func(data)

def calculate_crc16(data):
    crc16_func = crcmod.predefined.mkPredefinedCrcFun('crc-16')
    return crc16_func(data)

if __name__ == "__main__":
    try:
        ser = serial.Serial(port=serial_port, baudrate=serial_bps, timeout=serial_to)
    except:
        print("Error: unable to open serial comms")
        exit(1)
        
    receive_data()