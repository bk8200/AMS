#!/usr/bin/python
# -*- coding: utf-8 -*-
''' Interface for serial communication with the WMR driver

It contains a simple test program that sets constant motor velocities and outputs raw encoder readings. 
'''
import serial
import struct
from time import sleep, time
from ams import wrapToPi
from math import pi

class Wmr(object):
  WHEEL_VEL_GAIN = 1000.0
  AXIS_LENGTH = 0.075 # Axis length, in m
  WHEEL_MAX_V = 0.3 # Max wheel velocity, in m/s
  REPEAT = 10
  TIMEOUT = 0.02
  LOOP = 0.01
  GND = 190.0

  def __init__(self, port='/dev/ttyS0', baud=115200):
    self._serial = serial.Serial(port, baud, timeout=0)
    self._buff = ''

    self._flagMot = None
    self._flagSen = None

    self._ready = True
    self._autoTime = None
    self._autoState = 0

    self.alpha = 0.0
    self.ongnd = False

  def __enter__(self):
    return self

  def __exit__(self, type, value, traceback):
    for i in range(Wmr.REPEAT):
      self.setWheelVel() # Stop the robot
      sleep(Wmr.TIMEOUT)
    for i in range(Wmr.REPEAT):
      self.setPowerMode(motors=False, sensors=False) # Enable power-saving mode
      sleep(Wmr.TIMEOUT)
    self._serial.close()

  # Set tangential and angular velocity, in m/s and rad/s, respectively
  def setVel(self, v=0.0, omega=0.0):
    vl = -v+Wmr.AXIS_LENGTH/2.0*omega
    vr = -v-Wmr.AXIS_LENGTH/2.0*omega
    self.setWheelVel(vl, vr)

  # Set wheel velocities, in m/s
  def setWheelVel(self, left=0.0, right=0.0):
    cmd = [left, -right]
    chk = 0
    for i in xrange(len(cmd)):
      cmd[i] = Wmr.clamp(cmd[i], -Wmr.WHEEL_MAX_V, Wmr.WHEEL_MAX_V)
      cmd[i] = int(cmd[i]*Wmr.WHEEL_VEL_GAIN)
      cmd[i] = Wmr.clampShort(cmd[i])
      chk += (cmd[i]&0xFF) + (cmd[i]>>8)
    chk += 0x5A + 0x20 + 0x04
    msg = struct.pack('<BBBhhB', 0x5A, 0x20, 0x04, cmd[0], cmd[1], (-chk)&0xFF)
    #print("TX: "+":".join("{:02X}".format(ord(x)) for x in msg)) # Debug
    self._serial.write(msg)

  def setPowerMode(self, motors=False, sensors=False):
    chk = 0
    a = int(motors)
    b = int(sensors)
    chk = 0x5A + 0x21 + 0x02 + a + b
    msg = struct.pack('<BBBBBB', 0x5A, 0x21, 0x02, a, b, (-chk)&0xFF)
    #print("TX: "+":".join("{:02X}".format(ord(x)) for x in msg)) # Debug
    self._serial.write(msg)

  def setWatchdog(self, period):
    period = Wmr.clampUShort(period)
    chk = 0x5A + 0x22 + 0x01 + (period>>8) + (period&0xFF)
    msg = struct.pack('<BBBBBB', 0x5A, 0x22, 0x01, period, (-chk)&0xFF)
    #print("TX: "+":".join("{:02X}".format(ord(x)) for x in msg)) # Debug
    self._serial.write(msg)

  # Update sensors
  def updateSensors(self):
    msg = self._serial.read((3+40+1)*20)
    if len(msg):
      #print("RX: "+":".join("{:02X}".format(ord(x)) for x in msg)) # Debug
      self._buff += msg
      n = len(self._buff)
      i = 0
      data = None
      while n-i >= 4:
        head, cmd, length = struct.unpack('<BBB', self._buff[i:i+3])
        if head == 0x5A:
          if n-i >= length + 1 + 3:
            chk = sum([ord(x) for x in self._buff[i:i+3+length+1]])
            if chk & 0xFF == 0:
              if cmd == 0x10:
                data = struct.unpack('<Iiii8B8B8B', self._buff[i+3:i+3+length])
              i += 3 + length + 1
              continue
          else:
            break
        i += 1
      self._buff = self._buff[i:]
      if data is not None:
        self.ongnd = sum(data[4:4+7])/7.0 < self.GND
        self._handleEncoders(data[1], data[2], data[3])
        if self._ready:
          self._handleSensors(data[4:4+7])
        else:
          self._autoCalibrate()
        self._flagMot = (data[0] & 0x01000000) != 0
        self._flagSen = (data[0] & 0x02000000) != 0
        t = data[0] & 0x00FFFFFF

        #print('\033c')
        #print('timer={} ms, motors={}, sensors={}'.format(t, self._flagMot, self._flagSen))
        #print('left={}, right={}, aux={}'.format(*data[1:4]))
        #print('individual: {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}'.format(*data[4:4+8]))
        #print('all on:     {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}'.format(*data[12:12+8]))
        #print('all off:    {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}, {:02X}'.format(*data[20:20+8]))
        #
        #print('Individual   All on       All off')
        #print('----------   ----------   ----------')
        #tmpl = '{:10s}   {:10s}   {:10s}'
        #for i in range(8):
        #  a = [int(x*10/255) for x in data[4+i:21+i:8]]
        #  print(tmpl.format(*['o'*x for x in a]))

  def _handleEncoders(self, left, right, heading):
    #print('left = %d, right = %d, heading = %d' % (left, right, heading)) # Debug
    self.alpha = wrapToPi(-float(heading)*2.0*pi/8192.0)

  def _autoCalibrate(self):
    if self._autoState == 0:
      if not self.ongnd or self._autoTime is None:
        self.setWheelVel(0., 0.)
        self._autoTime = time()
      else:
        self.setVel(0.0, 2.0*pi/5.0*2.1)
        if time() - self._autoTime > 5:
          self._autoState = 1
          self._autoTime = time()
    elif self._autoState == 1:
      ea = wrapToPi(0.0 - self.alpha)
      omega = ea * 20.0
      if omega > 2.0:
        omega = 2.0
      elif omega < -2.0:
        omega = -2.0
      self.setVel(0.0, omega)
      if abs(ea) < 0.01:
        self._autoState = 2
    elif self._autoState == 2:
      self.setWheelVel(0.0, 0.0)
      if time() - self._autoTime > 0.5:
        self._ready = True

  def _handleSensors(self, sensors):
    smin = min(sensors)
    smax = max(sensors)
    av = sum(sensors) / 7    
    threshold = (smin + smax) / 2

    lineRight = None
    lineLeft = None
    
    if smax - smin > 80:
        lineEl = [i for i in range(len(sensors)) if sensors[i] > threshold]

        # Right edge
        if lineEl[0] == 0:
            lineRight = -1
        else:
            vs = sensors[lineEl[0]-1:lineEl[0]+1]
            if abs(vs[1] - vs[0]) > 0:
                lineRight = ((lineEl[0] + float((threshold - vs[0])) / (vs[1] - vs[0])) - 4) / 2.5
            else:
                lineRight = (lineEl[0] + 0.5 - 4) / 2.5
                
        # Left edge
        if lineEl[-1] == 6:
            lineLeft = 1
        else:
            vs = sensors[lineEl[-1]:lineEl[-1]+2]
            if abs(vs[1] - vs[0]) > 0:
                lineLeft = ((lineEl[-1] + float((threshold - vs[1])) / (vs[1] - vs[0])) - 2.5) / 2.5
            else:
                lineLeft = (lineEl[-1] - 2.5) / 2.5


    elif smax > 150:
        # On the line
        lineRight = -1
        lineLeft = 1
        
    self._handleLineSensor(lineLeft, lineRight, sensors)
        
  def _handleLineSensor(self, left, right, sensors):
    pass

  def startUp(self, mode=2):
    if mode>1:
      m = True
      s = True
      i = Wmr.REPEAT
      while self._flagMot != m and self._flagSen != s and i>0:
        self.setPowerMode(motors=m, sensors=s)
        sleep(Wmr.TIMEOUT)
        self.updateSensors()
        i -= 1
      if i==0:
        exit()
    else:
      self.setPowerMode(motors=True, sensors=True)
    
    if mode<2:
      self._ready = True

  @staticmethod
  def clamp(num, low, high):
    return max(min(high, num), low)

  @staticmethod
  def clampShort(num):
    return Wmr.clamp(num, -32768, 32767)

  @staticmethod
  def clampUShort(num):
    return Wmr.clamp(num, 0, 65536)

if __name__ == '__main__':
  try:
    with Wmr() as robot:
      robot.startUp()
      while True:
        robot.updateSensors()
        sleep(Wmr.LOOP)
  except KeyboardInterrupt:
    pass
