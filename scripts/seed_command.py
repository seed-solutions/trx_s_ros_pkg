#!/usr/bin/env python
# coding:utf-8

import serial
from threading import Lock

class SeedCommand:
  def __init__(self,port='ttyUSB0',baud=115200):
    self.ser=serial.Serial('/dev/'+port, baud, timeout=0.1)
    self.mutex = Lock()

  #COM Port Open
  def COM_Open(self):
    self.ser.write("S8\r")
    self.ser.write("O\r")

  #Com Port Close
  def COM_Close(self):
    self.ser.write("C\r")
    self.ser.close()

  #Binary data to Hex String data and decide number of byte
  def num2str(self,value,byte=1):
    if value < 0: value = 0xFFFFFF + 1 + value
    return str(hex(value).upper()[2:][-2:].rjust(2**byte,"0"))

  #Binary data to Hex data and decide number of byte
  def num2hex(self,value,byte=1):
    if value < 0:value = 0xFFFFFF + 1 + value
    return hex(value).upper()[2:][-2:].rjust(2**byte,"0")

  #SEED CAN data send
  def CAN_Snd(self,SndData):
    self.mutex.acquire()
    CAN_Transmit = ''.join(SndData) + '\r'
    self.ser.write(CAN_Transmit)
    #print "send\t:%s" % CAN_Transmit
    self.mutex.release()

  #SEED CAN data read
  def CAN_Read(self):
    ret_str = ''
    while(self.ser.inWaiting() < 1):
      pass
    while(self.ser.read(1) != 't'):
      pass
    ret_str = 't'
    # Get Data Length 
    ret = self.ser.read(4)
    data_len = int(ret[-1],16)
    ret_str += ret
    # Get All data
    ret = self.ser.read(data_len*2)
    ret_str += ret
    #print "Receive Data \t:%s" % ret_str
    return ret_str
	
  ####################################################################################
  ########################## SEED Command Function  ##################################
  ####################################################################################
  '''
  id_num	:ID data
  d3		:SEED Command
  d4~d5	:Parameter
  '''
  ####################################################################################
  def SCM(self,id_num,d3,d4,d5,d6,d7,d8):
    SndData = 12*[0]
    SndData[0] ='t'

    if d3 == 0x53: SndData[1] = self.num2str(0x00,0)
    else: SndData[1] = self.num2str(0x03,0)

    SndData[2] = self.num2str(id_num)
    SndData[3] = self.num2str(8,0)
    SndData[4] = self.num2str(0xF0 + id_num)
    SndData[5] = self.num2str(0x00)
    SndData[6] = self.num2str(d3)
    SndData[7] = self.num2str(d4)
    SndData[8] = self.num2str(d5)
    SndData[9] = self.num2str(d6)
    SndData[10] = self.num2str(d7)
    SndData[11] = self.num2str(d8)

    self.CAN_Snd(SndData)

#---------  Get inormation command(0x40~0x4F)  ---------
  #Get Motor Position
  def Get_Pos(self,id_num):
    self.ser.flushInput()
    self.ser.flushOutput()
    cmd = 0
    speed = 0
    position = 0

    self.SCM(id_num,0x42,id_num,0x00,0x00,0x00,0x00)
    data = self.CAN_Read()
    cmd = int(data[9]+data[10],16)
    if(cmd != 0x42): return ["None","None"]
    else:
      speed = int(data[11]+data[12]+data[13]+data[14],16)
      position = int(data[15]+data[16]+data[17]+data[18]+data[19]+data[20],16)

      if position > 0xFFFFFF/2: position = position - 0xFFFFFF
      elif position == 0xFFFFFF: position = 0
      else : position = position

      return [speed,position]

  #---------  Run motor command(0x50~0x5F)  ---------
  #servo ON/OFF 
  def Servo(self,id_num,data):
    self.SCM(id_num,0x50,id_num,data,0x00,0x00,0x00)

  #Motor Stop
  def Motor_Stop(self,id_num):
    self.SCM(id_num,0x51,0x00,0x00,0x00,0x00,0x00)

  #Run SEED Script
  def Script_Go(self,id_num,s_num):
    if s_num > 0x00 and s_num < 0x0F :
      self.SCM(id_num,0x5F,id_num,s_num,0x00,0x00,0x00)

  #Time and Abusolute Go
  def TMove_Abs(self,id_num,time,pos):
    data = 5*[0]

    if pos >= 0xFFFFFF/2: pos = 0xFFFFFF/2
    elif pos <= -0xFFFFFF/2: pos = -0xFFFFFF/2

    data[0] = time >> 8
    data[1] = time  
    data[2] = pos >> 16
    data[3] = pos >> 8
    data[4] = pos 

    self.SCM(id_num,0x64,data[0],data[1],data[2],data[3],data[4])
