# parse_strain_gauge

# Scripts to parse phidget strain gauge sensor for raspberry pi. Uses lcm and stores original strings
# the software api doesn't allow the strain gauge readings to be simultaneously read and always couples. 
# Only 1 channed at a time can be read.
# Author: Blair Thornton
# Date: 09/10/2017

import os
import lcm
import time
import math
import threading

from Phidget22.Devices.VoltageRatioInput import *
from Phidget22.PhidgetException import *
from Phidget22.Phidget import *
from Phidget22.Net import *

global channel_count
global value
global toggle
global report_flag

toggle = 0
channel_count = 0
value=[]
report_flag=0

for i in range(4):
    value.append(0)


def VoltageRatioChangeHandler(e, voltageRatio):
    global channel_count
    global value, toggle
    global report_flag
    
    if toggle ==1:
        value[channel_count] = voltageRatio*1000000
        channel_count=channel_count+1
        if channel_count>=4:
            channel_count=0
            report_flag=1
        
        ch.setChannel(channel_count)
        toggle=0
    else:
        toggle=toggle+1
        
# set channel name
ch = VoltageRatioInput()
ch.openWaitForAttachment(5000)
ch.open()
# set initial channel, enable, set gain to max, set data interval to 25ms (100ms for all)
ch.setChannel(channel_count)
ch.setBridgeEnabled(True)
ch.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
ch.setDataInterval(300)
# trigger on interval
ch.setVoltageRatioChangeTrigger(0)
# behaviour on trigger
ch.setOnVoltageRatioChangeHandler(VoltageRatioChangeHandler)

while True:
    if report_flag == 1:
        report_flag = 0
        print(value)

ch.close()

