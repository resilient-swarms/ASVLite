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

global value
global report_flag

value=0
report_flag=False


def VoltageRatioChangeHandler(e, voltageRatio):
    global value, report_flag
    
    value = voltageRatio*1000000
    report_flag=True
        
        
# set channel name
ch = VoltageRatioInput()
ch.openWaitForAttachment(5000)
ch.open()
# set initial channel, enable, set gain to max, set data interval to 25ms (100ms for all)
ch.setChannel(0)
ch.setBridgeEnabled(True)
ch.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
ch.setDataInterval(10)
# trigger on interval
ch.setVoltageRatioChangeTrigger(0)
# behaviour on trigger
ch.setOnVoltageRatioChangeHandler(VoltageRatioChangeHandler)

while True:
    if report_flag == True:
       print(value)
       report_flag = False
 
ch.close()

