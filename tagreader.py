#!/usr/bin/python
# -*- coding: utf-8 -*-
''' NFC-tag reader

This program publishes a ROS message with the ID of the tag whenever a new tag is detected with MFRC-522 module that is attached to the Raspberry Pi. 

# Installation

This program requires an eneable SPI interface on the Raspberry Pi and Python libraries for that interface.

```bash
#Use sudo raspi-config to enable SPI interface or:
sudo sed -i 's/^[# ]*dtparam=spi=.*//' /boot/config.txt
echo 'dtparam=spi=on' | sudo tee -a /boot/config.txt

sudo apt-get update
sudo apt-get upgrade

sudo apt-get install python2.7-dev

cd
git clone https://github.com/lthiery/SPI-Py.git
cd ~/SPI-Py
sudo python setup.py install

#cd
#git clone https://github.com/pimylifeup/MFRC522-python.git
```

# Subscribing to the tag topic

To subscribe to the tag topic, add the following lines to your program.

Add the following line to the import section:
```python
from std_msgs.msg import Int64
```

Add the following line to the `__init__` method of your class:
```python
self._subTag = rospy.Subscriber('tag', Int64, self._handleTag)
```

Add the following method to your class:
```python
def _handleTag(self, msg):
  print('Tag data: {}'.format(msg.data))
```
'''
import MFRC522
import RPi.GPIO as GPIO
import rospy
from std_msgs.msg import Int64
from time import sleep

class TagReader(object):
  REPEAT = 2

  def __init__(self):
    self._reader = MFRC522.MFRC522()

  def read(self):
    status = None
    uid = None
    n = TagReader.REPEAT
    while n > 0:
      n -= 1
      (status, type) = self._reader.MFRC522_Request(self._reader.PICC_REQIDL)
      if status != self._reader.MI_OK:
        continue
      uid = None
      (status, uid) = self._reader.MFRC522_Anticoll()
      if status == self._reader.MI_OK:
        break

    if uid is not None:
      return TagReader.uid2num(uid)
    else:
      return None

  @staticmethod
  def uid2num(uid):
    if len(uid) != 5:
      return None
    else:
      n = 0
      for i in range(0, 5):
        n = n * 256 + uid[i]
      return n

if __name__ == '__main__':
  reader = TagReader()
  pub = rospy.Publisher('tag', Int64, queue_size=10)
  rospy.init_node('tagreader')

  try:
    last = None
    #rate = rospy.Rate(50)
    while not rospy.is_shutdown():
      id = reader.read()
      if id != last and id is not None:
        msg = Int64()
        msg.data = int(id)
        pub.publish(msg)
      last = id
      #rate.sleep()
      sleep(0.015*2)
  finally:
    GPIO.cleanup()
