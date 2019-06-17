# written by Blair Thornton, 13/10/2018, receives nmea data where can treat as normal nmea (global) or locaal relative to some reference point in m 
# this needs to run on boot and the controller that receives this needs to store the nmea data
from threading import Thread
import socket
import time

import lcm

from inertial_pose.qualisys import qualisys

output_prefix = "[rpi_host_server] "

VERBOSE = False
IP_PORT = 50001

global_or_local_m = 1 #0 sends global position,, 1 sends local position in m relative to qualisys centre, needs to be read from vehicle yaml
proceed_nmea = 0 #flag to proceed to complete pose data
latitude = -9999
longitude = -9999
northings = -9999
westings = -9999
heading=-9999
data_integrity=0

def debug(text):
    if VERBOSE:
        print output_prefix + "Debug:---", text

# ---------------------- class SocketHandler ------------------------
class SocketHandler(Thread):
    def __init__(self, conn):
        Thread.__init__(self)
        self.conn = conn

    def run(self):
        global isConnected
        debug("SocketHandler started")
        while True:
            cmd = ""
            try:
                debug("Calling blocking conn.recv()")
                cmd = self.conn.recv(1024)
            except:
                debug("exception in conn.recv()") 
                # happens when connection is reset from the peer
                break
            debug("Pose fix: " + cmd + " len: " + str(len(cmd)))
            if len(cmd) == 0:
                break
            self.executeCommand(cmd)
        conn.close()
        print output_prefix + "Client disconnected. Waiting for next client..."
        isConnected = False
        debug("SocketHandler terminated")

    def executeCommand(self, cmd):
        global global_or_local_m
        global proceed_nmea, latitude, longitude, northings
        global westings, heading, data_integrity

        # Check if this is redundant for the current setup
        lc=lcm.LCM("udpm://239.255.76.67:7667?ttl=64")

        msg=qualisys()
        msg.global_or_local_m=int(global_or_local_m)

        debug("Calling executeCommand() with  cmd: " + cmd)
#        if cmd[:-1] == "go":  # remove trailing "\0"
        #data=list(cmd)
        #print cmd #"Pose fix: x=%d, y=%d, heading=%d" % (float(data[0]),float(data[2]),float(data[4]))
        #self.conn.sendall(cmd)# + "\0")
        data=cmd.split(',')
        #need to locally store too
        if data[0] == '$GPGGA' and proceed_nmea == 0:
            msg.gpgga = str(cmd)
            data_integrity=data[6]

            msg.data_integrity = int(data_integrity)

            if data[6] == '2':#checks data is valid
                latitude_str=data[2]
                
                if global_or_local_m is 0:
                    latitude = float(latitude_str[0:2])+float(latitude_str[2:13])/60#decimal degrees
                    if data[3] == 'S':
                        latitude=-latitude
                    msg.latitude = latitude
                else:
                    northings = float(latitude_str)
                    msg.northings = northings
#                    print "[lat_str]", latitude_str
#		    print "[northings]", northings 

                longitude_str=data[4]
                if global_or_local_m is 0:
                    longitude = float(longitude_str[0:3])+float(longitude_str[3:14])/60#decimal degrees
                    if data[5] == 'E':
                        longitude=-longitude
                    
                    msg.longitude = longitude
                else:
                    westings = float(longitude_str)
                    msg.westings = westings
                #next get heading
                proceed_nmea+=1
            else:
                msg.data_integrity = 0
                msg.heading_degrees = -9999
                msg.northings = -9999
                msg.westings = -9999
                msg.latitude = -9999
                msg.longitude = -9999
                lc.publish("qualisys", msg.encode())
                print output_prefix + "invalid NMEA"


        elif data[0] == '$HEHDT' and proceed_nmea == 1:
            msg.hehdt = str(cmd)
            heading=float(data[1])
#	    print "[s2] n,w", northings, westings
    	    msg.northings = northings
    	    msg.westings = westings
#	    print "[s3] m.n,m.w", msg.northings, msg.westings

            msg.heading_degrees = heading
            msg.data_integrity = 2
            proceed_nmea+=1

        elif data[0] == '$HEHDT' and proceed_nmea == 0:
            proceed_nmea=0
            msg.data_integrity = 0
            msg.heading_degrees = -9999
            msg.northings = -9999
            msg.westings = -9999
            msg.latitude = -9999
            msg.longitude = -9999
            lc.publish("qualisys", msg.encode())
            print output_prefix + "invalid NMEA"

        if proceed_nmea == 2:
            # print "[rhs] northings = %f, westings = %f, latitude = %f, longitude = %f, heading = %f" % (northings, westings, latitude,longitude,heading)
            # print "[rpi_host]", msg.northings, msg.westings, msg.heading_degrees
            proceed_nmea = 0       
            # publish lcm
            lc.publish("qualisys", msg.encode())
        
# ----------------- End of SocketHandler -----------------------

serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# close port when process exits:
serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) 
debug("Socket created")
HOSTNAME = "" # Symbolic name meaning all available interfaces
try:
    serverSocket.bind((HOSTNAME, IP_PORT))
except socket.error as msg:
    print output_prefix + "Bind failed", msg[0], msg[1]
    sys.exit()
serverSocket.listen(10)

print output_prefix + "Waiting for a connecting client..."
isConnected = False
while True:
    debug("Calling blocking accept()...")
    conn, addr = serverSocket.accept()
    print output_prefix + "Connected with client at " + addr[0]
    isConnected = True
    socketHandler = SocketHandler(conn)
    # necessary to terminate it at program termination:
    socketHandler.setDaemon(True)  
    socketHandler.start()
    t = 0
    while isConnected:
        print output_prefix + "Server connected at", t, "s"
        time.sleep(10)
        t += 10
