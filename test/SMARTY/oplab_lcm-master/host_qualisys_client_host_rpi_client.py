"""
    Script to connect data from Qualisys using an ethernet cable, and forwards it to a target with a given IP addresses (TCPIP) over wifi access point

    Qualisys acts as a sever, with the host running this code connected via a lan cable, 
    with ethernet IP on same subnet as Qualisys (in Boldrewood tank Qualisys is 192.168.10.1).
    Be careful as Qualisys cameras are also on the same subnet (for Boldrewood tank Qualisys, can use 192.168.10.10 for the host PC)
    Connects to QTM and streams 3D data forever (start QTM project first and Play->Play with Real-Time output)

    Qualisys is the server on the ethernet network, which the host connects to as a client
    The body to which we send the data act as a server, to which the host connects to as a client

    Both the qualisys and body servers need to be running prior to running this code
    Currently this code only supports a single body. To use multiple the host will need to act as the server to the bodies and the code modified accordingly

    Requires python 3.5 or higher

    Author Blair Thornton, 13/10/2018
"""
from threading import Thread
import asyncio, socket, qtm
import math, time
from time import gmtime, strftime
from datetime import datetime

# Mandatory: prepend this to any console output (print messages)
output_prefix = "[host_qualisys] "

# this is the IP address of the ethernet LAN port on qualisys
#QUALISYS_IP_ADDRESS = "192.168.10.1"
QUALISYS_IP_ADDRESS = "192.168.60.71"

# flag to ccontrol coordinated system orientation
right_hand_rule_thumb_up_or_down = 0

# this is the IP addres of the body, which needs to be on the same subnet as the forwarding port of the host
# This values must match those present in "rpi_host_server.py"
BODY_IP_ADDRESS = "192.168.60.201"
BODY_IP_PORT = 50001

on_nan_default_or_blank = 1 # when qualisys is nan 0 to send default position or 1 to send a blank value
global_or_local_m = 1 # 0 sends global position,, 1 sends local position in m relative to qualisys centre

# default values to send if not seen by qualisys for each body (i.e. nan)
default_latitude_values = 0
default_longitude_values = 0

# number of the body you want to forward and IP address: Port to forward to

# for the moored system
# body_number = 1
# default_heading_values = 110 -(-27.6) #also acts as the offset to north of qualisys for each body

# for dynamic positioning
body_number = 0
default_heading_values =  ((+ 90 - (+ 45))) # also acts as the offset to north of qualisys for each body 

gpgga_string=["$GPGGA"]
hehdt_string=["$HEHDT"]

def qualisys_on_packet(packet):
    """ Callback function that is run when stream-data is triggered by QTM """

    # conversion of parameters to a GPS like packet (position of tank and conversion of distance (mm) to lat long)
    latitude_offset = 5056.20500	# DDDMM.MMMMM N 
    longitude_offset = 00124.34367	# DDMM.MMMMM W
    latitude_factor = 0.0000005393	# 1mm in min latitude
    longitude_factor = 0.0000008547	# 1mm in min longitude

    # read packet number and timestamp given by qualisys and generate own timestamp
#    print(output_prefix + 'Framenumber: %d\t Timestamp: %d' % (packet.framenumber, packet.timestamp))
#    print(output_prefix + 'Host time = ', time.time())

    timestamps = datetime.utcnow().strftime("%H%M%S.%f")
    timestamps = float(timestamps[0:9]) 
#    print(output_prefix + 'Host UTC time = ', timestamps)

    # header lists the names of bodies recognised by Qualisys as they appear in the project options
    # bodies gives a 2 x 3 matrix (tuple of )


    header, bodies = packet.get_6d_euler()

    print("Framenumber:", packet.framenumber, "Body count: ", header.body_count)

    names = ['dynamic_positioning', 'moored']

    print(output_prefix + 'header:', header)
    print(output_prefix + 'name:',names[0], 'x:', bodies[0][0][0], 'y:', bodies[0][0][1], 'a3:', bodies[0][1][2])
    print(output_prefix + 'name:',names[1], 'x:', bodies[1][0][0], 'y:', bodies[1][0][1], 'a3:', bodies[1][1][2])

    # decode qualisys packet

    # initially set nan_flag to 0
    nan_flag = 0

    # decode qualisys x, y and heading, handling nan cases
    # If NaN is received (body non detected) fill in with the default values
    if math.isnan(bodies[body_number][0][0]):
        longitude_value=default_latitude_values
        nan_flag+=1
    else:
        longitude_value=bodies[body_number][0][0]


    if math.isnan(bodies[body_number][0][1]):
        if right_hand_rule_thumb_up_or_down is 0:
            latitude_value=default_latitude_values
        else:
            latitude_value=default_latitude_values
        nan_flag+=1
    else:
        if right_hand_rule_thumb_up_or_down is 0:
            latitude_value=-bodies[body_number][0][1]
        else:
            latitude_value=bodies[body_number][0][1]

    if math.isnan(bodies[body_number][1][2]):
        heading_value=default_heading_values
        nan_flag+=1
    else:
        heading_value=-bodies[body_number][1][2]+default_heading_values

        # Heading angle unwrap
        if heading_value > 180:
            heading_value -= 360
        if heading_value < -180:
            heading_value += 360

    if global_or_local_m is 0:
        # send latitude and longitude in gpgga
        longitude=longitude_factor*float(longitude_value)+longitude_offset #assumes x is longitude
        latitude=latitude_factor*float(latitude_value)+latitude_offset #assumes y is latitutde
    else:
        longitude=float(longitude_value)/1000
        latitude=float(latitude_value)/1000

    heading=float(heading_value)

    # if data is good
    if nan_flag is 0:
        data_integrity = 2
    else:
        data_integrity = 0

    # GPGGA packet
    gpgga_string = "$GPGGA"+","+str(timestamps)+ "," + '{:013.8f}'.format(latitude) + ",N,"+ '{:014.8f}'.format(longitude) + ",W,"+ '{:d}'.format(data_integrity) +",12,1.5,0,M,100,M,1.0,1"

    # HEHDT packet
    hehdt_string = "$HEHDT"+","+str(heading)+",T\r\n"

    # handle nan case (send default or if on_nan_default_or_blank = 1, send blank values for latitude, longitude, and heading)
    if nan_flag is not 0 and on_nan_default_or_blank is 1:
        gpgga_string = "$GPGGA"+","+str(timestamps)+",,,,,"+'{:d}'.format(data_integrity)+",12,1.5,0,M,100,M,1.0,1"
        hehdt_string = "$HEHDT"+",,T\r\n"

    j=0
    checksum=0
    while j<len(gpgga_string):
        checksum = checksum ^ ord(gpgga_string[j])
        j+=1
    gpgga_string = gpgga_string + hex(checksum) + '\r\n'

    # print local nmea formats for the body
    print(output_prefix + 'Body number ' + '[{:d}]: '.format(body_number) + gpgga_string)
    print(output_prefix + 'Body number ' + '[{:d}]: '.format(body_number) + hehdt_string)

    # WARNING
    # host_rpi_sendCommand(gpgga_string)
    # host_rpi_sendCommand(hehdt_string)

# --------------------------------------------------------------------------------------
def host_rpi_sendCommand(cmd):
    try:
        # convert string to hex bytes
        cmd_bytes = bytes(cmd,'utf-8')
        sock.sendall(cmd_bytes)
    except:
        host_rpi_closeConnection()

def host_rpi_closeConnection():
    global isConnected
    sock.close()
    isConnected = False

def host_rpi_connect():
    global sock
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(output_prefix + "Connecting to server on " + str(BODY_IP_ADDRESS) + ":" + str(BODY_IP_PORT))
    try:
        sock.connect((BODY_IP_ADDRESS, BODY_IP_PORT))
    except:
        print(output_prefix + "failed... confirm the server is running check it can be pinged, trying again in 10s")
        return False
    print(output_prefix + "Connection established")

    return True

#-----------------------------------------------------------------------------------------

async def qualisys_setup():
    """ Main function """
    print(output_prefix + "Trying to connect to Qualisys server in: " + str(QUALISYS_IP_ADDRESS))
    connection = await qtm.connect(QUALISYS_IP_ADDRESS)

    if connection is None:
    	#TODO: Here we can try to connect to the fake server, that must meet the same qtm data structure
        print (output_prefix + "No active server found in:" + str(QUALISYS_IP_ADDRESS))
        return

    state = await connection.get_state()

    await connection.stream_frames(frames='frequencydivisor:100',components=['6dEuler'], on_packet=qualisys_on_packet)


if __name__ == "__main__":


    sock = None
    isConnected = True
    trys=1
    trys_limit=10

    while isConnected is False and trys<trys_limit:
        print(output_prefix + str(trys) + " of " + str(trys_limit) + " attempts")
        if host_rpi_connect() is True:
            isConnected = True
        trys+=1
        time.sleep(5)

    if isConnected is True:

        asyncio.ensure_future(qualisys_setup())
        asyncio.get_event_loop().run_forever()
    
    else:
        print(output_prefix + "Connection to" + BODY_IP_ADDRESS + ":" + BODY_IP_PORT + "failed")

    print(output_prefix + "done")
