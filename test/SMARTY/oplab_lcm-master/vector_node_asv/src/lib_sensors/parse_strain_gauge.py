# parse_strain_gauge

# Scripts to parse phidget strain gauge sensor for raspberry pi. Uses lcm and stores original strings
# the software api doesn't allow the strain gauge readings to be simultaneously read and always couples. Only 1 channed at a time can be read.
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
from strain.strain import strain

global value
global report_flag

output_prefix = "[parse_strain_gauge]"

value = 0
report_flag = False

class parse_strain_gauge(threading.Thread):
    def __init__(self, strain_flag, strain_file):
        threading.Thread.__init__(self)
        
        global value
        global report_flag
        
        lc = lcm.LCM()
        
        # set channel name
        ch = VoltageRatioInput()
        try:
            ch.openWaitForAttachment(5000)
        except PhidgetException as e:
            # print ("%s" % (output_prefix))
            print("%s Phidget Exception %i: %s" % (output_prefix, e.code, e.details))
            print("Press Enter to Exit...\n")
            readin = sys.stdin.read(1)
            exit(1)
        ch.open()
        # set initial channel, enable, set gain to max, set data interval to 25ms (100ms for all)
        ch.setChannel(0)
        ch.setBridgeEnabled(True)
        ch.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
        ch.setDataInterval(50)
        # trigger on interval
        ch.setVoltageRatioChangeTrigger(0)
        # behaviour on trigger
        ch.setOnVoltageRatioChangeHandler(VoltageRatioChangeHandler)
        
        #enter infinite loop, stopped by ._Thread_stop()
        while True:
          if strain_flag == 1:
            # read in data
            if report_flag == True:
              epoch_time = round(time.time()*1000)/1000
              strain_value = value
              report_flag = False
            
              strain_string = "'epoch_time':" + format(epoch_time,'.3f') + ", 'strain':" + str(value)
              #print strain_string
              # generate lcm
              msg=strain()
              msg.timestamp=epoch_time
              msg.strain_value=strain_value

              # publish lcm
              lc.publish("strain", msg.encode())
            
              #write to logs
              with open(strain_file ,'a') as fileout:
                fileout.write(strain_string +"\n")
              fileout.close()
                

def VoltageRatioChangeHandler(e, voltageRatio):
    global value, report_flag
    
    value = voltageRatio*1000000
    report_flag=True