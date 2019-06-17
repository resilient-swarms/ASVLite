"""
    Script that creates and sends fake Qualisys-like data to a target with a given IP address over a WiFi link

    NOTE: this doesn't require the Qualisys running, in that case use *host_qualisys_client_host_rpi_client.py* instead

    The body to which we send the data act as a server, to which this host connects to as a client.
    The body server (rpi_host_server.py) need to be running prior to running this code.
    Currently this code only supports a single body. To use multiple the host will need to act as the server to the bodies and the code modified accordingly

    Requires python 3.5 or higher

    Based on: host_qualisys_client_host_rpi_client.py by Blair Thornton, 13/10/2018 (release v1.1 in GitHub repository)

    Author: Jose Cappelletto, 12/12/2018
"""

from threading import Thread
import asyncio, socket, qtm
import math, time
from time import gmtime, strftime
from datetime import datetime

# Mandatory: prepend this to any console output (print messages)
output_prefix = "[fake_client] "

# this is the IP address of the ethernet LAN port on qualisys
# REMOVE
QUALISYS_IP_ADDRESS = "192.168.10.1"

# flag to ccontrol coordinated system orientation
right_hand_rule_thumb_up_or_down = 0

# this is the IP addres of the body, which needs to be on the same subnet as the forwarding port of the host
BODY_IP_ADDRESS = "192.168.60.201"
#BODY_IP_ADDRESS = "127.0.0.1"
BODY_IP_PORT = 50001

on_nan_default_or_blank = 1 # when qualisys is nan 0 to send default position or 1 to send a blank value
global_or_local_m = 1 # 0 sends global position,, 1 sends local position in m relative to qualisys centre

# default values to send if not seen by qualisys for each body (i.e. nan)
default_latitude_values = 0
default_longitude_values = 0

# total elapsed simulation time
elapsed_time = 0

# number of the body you want to forward and IP address: Port to forward to

# for the moored system
# body_number = 1
# default_heading_values = 110 -(-27.6) #also acts as the offset to north of qualisys for each body

# for dynamic positioning
body_number = 0
default_heading_values =  ((+ 90 - (+ 45))) # also acts as the offset to north of qualisys for each body 

gpgga_string=["$GPGGA"]
hehdt_string=["$HEHDT"]

#MODIFY
def rectangular_trajectory(width, height, time, period):
    """ Function for trajectory generation. It receives elapsed time, period, and dimensions """
    x = 0   # movement along width/longitude
    y = 0   # movement along height/latitude
    yaw = 0 # orientation referenced against NORTH
    # trajectory goes W -> H -> W -> H counter clockwise
    total_perimeter = 2*(width + height) 

    # trim excess time over period value (we could work on normalized values for trajectory parametrization)
    # built-in modulus implementation in Python can throw some unexpected values due to rounding...
    norm_time = time/period - math.floor(time/period)
    # We must figure out in which section of the trajectory are. There are 4 sections, one per side of the rectangle
    # Section 1:
    if norm_time < (width/total_perimeter):
        x = width * norm_time/(width/total_perimeter)
        y = 0
        yaw = 90
    elif norm_time < (width+height)/total_perimeter:
        x = width
        y = height * (norm_time - width/total_perimeter)/(height/total_perimeter)
        yaw = 0
    elif norm_time < (width+height+width)/total_perimeter:
        x = width * (1 - (norm_time - (width+height)/total_perimeter)/(width/total_perimeter))
        y = height
        yaw = -90
    else:
        x = 0
        y = height * (1 - norm_time)/(height/total_perimeter)
        yaw = -180

    return (x,y,yaw)

def simulation_iterate(time_step):
    """ Function that is run within the main loop, until interrupted by user """
    global elapsed_time
    # conversion of parameters to a GPS like packet (position of tank and conversion of distance (mm) to lat long)
    latitude_offset = 5056.20500	# DDDMM.MMMMM N 
    longitude_offset = 00124.34367	# DDMM.MMMMM W
    latitude_factor = 0.0000005393	# 1mm in min latitude
    longitude_factor = 0.0000008547	# 1mm in min longitude
    traj_rect_height = 1  # dimensions for the desired rectangular trajectory
    traj_rect_width = 1
    traj_period = 10    # desired trajectory period (seconds)

    # read packet number and timestamp expected from qualisys and generate own timestamp
    # SETUP SIMULATED CONFIGURATION
    packet_framenumber = 0
    packet_timestamp = 1111

    print(output_prefix + 'Framenumber: %d\t Timestamp: %d' % (packet_framenumber, packet_timestamp))
    # WARNING: time.time() returns time as float number of seconds since the epoc, in UTC
 #   print(output_prefix + 'Qualisys(?) time = ', time.time())

    # retrieve current UTC date and time 
    timestamps = datetime.utcnow().strftime("%H%M%S.%f")
    timestamps = float(timestamps[0:9]) 
    print(output_prefix + 'Host UTC time = ', timestamps)

    # header lists the names of bodies recognised by Qualisys as they appear in the project options
    # bodies gives a 2 x 3 matrix (tuple of )
    # body_number is necessary to select the correct tuple

    # NOTE: header information is not used
    # NOTE: body information contains the XYZ/RPY data


    null_body = [[0,0,0],[0,0,0]]
    bodies = [null_body, null_body]
    names = ['dynamic_positioning', 'moored']
    
    ref_x,ref_y,ref_yaw = rectangular_trajectory(traj_rect_width, traj_rect_height, elapsed_time, traj_period)
    elapsed_time = elapsed_time + time_step

    # We transfer the simulated trajectory to the boby[0]: dynamic_positioning (aka Smarty)
    bodies[0][0][0] = ref_x
    bodies[0][0][1] = ref_y
    bodies[0][1][2] = ref_yaw

#    print ("X: " + '{:02.2f}'.format(ref_x) + "Y: " + '{:02.2f}'.format(ref_y))
#    print(output_prefix + 'name:',names[0], 'x:', bodies[0][0][0], 'y:', bodies[0][0][1], 'a3:', bodies[0][1][2])
#    print(output_prefix + 'name:',names[1], 'x:', bodies[1][0][0], 'y:', bodies[1][0][1], 'a3:', bodies[1][1][2])

    # Parse decoded data

    # initially set nan_flag to 0
    nan_flag = 0

    # decode qualisys x, y and heading, handling nan cases
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
    
    # Create GPGGA packet
    gpgga_string = "$GPGGA"+","+str(timestamps)+ "," + '{:013.8f}'.format(latitude) + ",N,"+ '{:014.8f}'.format(longitude) + ",W,"+ '{:d}'.format(data_integrity) +",12,1.5,0,M,100,M,1.0,1"

    # Create HEHDT packet
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

    # print local NMEA formats for the body
    print(output_prefix + 'Body # ' + '{:d}'.format(body_number) + gpgga_string)
    print(output_prefix + 'Body # ' + '{:d}'.format(body_number) + hehdt_string)
        
    # Send prepared data strings to rpi_host_server.py socket
    host_rpi_sendCommand(gpgga_string)
    host_rpi_sendCommand(hehdt_string)


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
        print(output_prefix + "ERROR: confirm the server is running, check it can be pinged. Trying again in 10s")
        return False

    print(output_prefix + "Connection established")
    return True

#-----------------------------------------------------------------------------------------
# Entry point
if __name__ == "__main__":

    sock = None
    isConnected = False
    trys=1
    trys_limit=10
    time_step=0.1   # time step in seconds

    while isConnected is False and trys<trys_limit:
        print(output_prefix + str(trys) + "/" + str(trys_limit) + " attempts")
        if host_rpi_connect() is True:
            isConnected = True
        trys+=1
        time.sleep(10)

    if isConnected is True:
        print (output_prefix + "Successful connection with BODY at " + str(BODY_IP_ADDRESS) + ":" + str(BODY_IP_PORT))
    # We no longer need asyncio library as there won't be any other concurrent process call
    # Here, we must call our main loop function that will keep sending data until interrupted by the user
        while(True):
            simulation_iterate(time_step)
            time.sleep(time_step)

    else:
        print(output_prefix + "Connection to" + BODY_IP_ADDRESS + ":" + BODY_IP_PORT + "failed")
    
    print(output_prefix + "done")
