    ###########################################################################
#   motrack_udp_multi.py
#
#   v1.0.0
#   written by Miles Johnson, Aaron Phelps, and Cem Onglsku
#
#   reads 6-DOF on multiple trackables from a running Tracking Tools
#   (go to Streaming Pane and check the box for 'Broadcast Frame Data'
#   in NaturalPoint Streaming Engine, For "Network Interface Selection",
#   under "Local Interface" set to <opti_ip> ) and sends the data to
#   <drone_ip>, a list of IPs for laptops running drones.
#
#    Note that the order of IP addresses in drone_ip corresponds to the
#    trackable numbers.  These must match!
#
#    Copies of this code are found on the svn at
# https://subversion.cs.illinois.edu/svn/ae483/trunk/optitrack/motrack_udp_multi.py
# -------------------------------------------------------------------------
#   Change Log:
#
#   
###########################################################################

import sys
import socket
import struct
import threading
import time
import math

import serial
import Queue
import fmcore
import scipy.io
from policies import lqr_outer

from utils import *

import requests
import json
import csv

starttime=time.time()
wiser_Data = ([" wiser_y(in)   wiser_x(in)   wiser_z(in)  opti_x(m)  opti_y(m)  opti_yaw(deg)"])
timeData = []
csvfile = "NickW_posData.csv"



##################################################################
######## Configuration Options ###################################
##################################################################

# Drone Computer IP's - First IP responds to first trackable, etc.
# To increase number of drones, add IP's (run "ifconfig eth0" on Linux machines)
#drone_ip = ["192.168.0.109"]
#drone_ip = "192.168.1.51", "192.168.1.52","192.168.1.54"
# drone_ip = "192.168.0.51", "192.168.0.52","192.168.0.54"
#drone_ip = ["192.168.1.72"]

# Drone Computer UDP Port
#drone_port = 3500
udp_port = 3500
address_list = ["192.168.1.71",
                "192.168.1.72",
                "192.168.1.73",
                "192.168.1.74",
                "192.168.1.75",
                "192.168.1.76",
                "192.168.1.77",
                "192.168.1.78",
                "192.168.1.70"]

            
# OptiTrack Computer IP address
# [Start->type 'cmd' in command window, type IPconfig, IPv4]
# opti_ip ="192.168.1.99"
opti_ip ="192.168.0.99"

# Data Port Set in Optitrack Streaming Properties
opti_port = 1511

# Multicast Interface in Optitrack Streaming Properties
multicastAdd = "239.255.42.99"

##################################################################
##################################################################
# DO NOT EDIT ANYTHING BENEATH THIS LINE!!!!
##################################################################
##################################################################

def unPack(data): #{{{1
    trackableState = []
    byteorder='@'
    PacketIn = data
    major = 2
    minor = 0
    offset = 0
    # message ID, nBytes
    messageID, nBytes = struct.unpack(byteorder+'hh',PacketIn[offset:offset+4])
    offset += 4
    #print 'messageID=',messageID,' number of bytes=',nBytes
    if (messageID == 7):
        frameNumber,nMarkerSets = struct.unpack(byteorder+'ii',PacketIn[offset:offset+8])
        offset += 8
        #print 'Markersets=', nMarkerSets
        i=nMarkerSets
        while (i > 0):
            ns = PacketIn[offset:offset+255]
            szNamelen=ns.find('\0')
            #print ns, szNamelen
            szName = struct.unpack(byteorder+str(szNamelen)+'s',PacketIn[offset:offset+szNamelen])[0]
            offset += szNamelen+1 # include the C zero char
            #print 'Modelname=',szName
            # markers
            nMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            #print 'Markercount=',nMarkers
            j=nMarkers
            while (j>0):
                x,y,z = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
                offset += 12
                j=j-1
            i=i-1

        #unidentified markers
        nUMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        offset += 4
        #print 'Unidentified Markercount=',nUMarkers
        i = nUMarkers
        while (i > 0):
            ux,uy,uz = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
            offset += 12
            i=i-1

        # rigid bodies
        nrigidBodies = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        nr = nrigidBodies
        #print nr
        offset += 4
        #print 'Rigid bodies=',nrigidBodies
        while (nr > 0):
            ID = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            rbx,rby,rbz = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
            offset += 12
            rbqx,rbqy,rbqz,rbqw = struct.unpack(byteorder+'ffff',PacketIn[offset:offset+16])
            offset += 16

            trackableState.append([ID,frameNumber, rbx, rby, rbz, rbqw, rbqx, rbqy, rbqz])  # our quaternion convention is scalar part first!

            #print '\nID=',ID
            #print 'pos:',rbx,rby,rbz
            #print 'ori:',rbqx,rbqy,rbqz,rbqw
            # associated marker positions
            nRigidMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            #print 'Marker count=',nRigidMarkers
            md = []
            markerID = []
            markersize = []
            for i in range(0,nRigidMarkers):
                md.extend(struct.unpack(byteorder+'fff',PacketIn[offset:offset+12]))
                offset += 12
            if major >= 2:
                for i in range(0,nRigidMarkers):
                    markerID.append(struct.unpack(byteorder+'I',PacketIn[offset:offset+4])[0])
                    offset += 4
                for i in range(0,nRigidMarkers):
                    markersize.append(struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0])
                    offset += 4
                for i in range(0,nRigidMarkers):
                    pass
                    #print 'Marker ',i+1,' ID=',markerID[i],' markerData=',md[3*i],md[3*i+1],md[3*i+2],' markersize=',markersize[i]
            else:
                for i in range(0,nRigidMarkers):
                    pass
                    #print 'Marker ',i+1,' markerData=',md[3*i],md[3*i+1],md[3*i+2]

            # marker errors
            if major >= 2:
                markerError = struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0]
                offset += 4
                #print 'Mean marker error=',markerError

            nr = nr-1 # next rigid body

        #skeletons
        if (major==2 and minor>0) or major>2:
            nSkeletons = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
            offset += 4
            #print 'Skeletons=',nSkeletons
            ns = nSkeletons
            while (ns > 0):
                skeletonID = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                offset += 4
                #print 'SkeletonID=',skeletonID
                nsrigidBodies = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                nsr = nsrigidBodies
                offset += 4
                #print 'Rigid body count=',nsrigidBodies
                while (nsr > 0):
                    IDsr = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                    offset += 4
                    srbx,srby,srbz = struct.unpack(byteorder+'fff',PacketIn[offset:offset+12])
                    offset += 12
                    srbqx,srbqy,srbqz,srbqw = struct.unpack(byteorder+'ffff',PacketIn[offset:offset+16])
                    offset += 16
                    #print 'ID=',IDsr
                    #print 'pos:',srbx,srby,srbz
                    #print 'ori:',srbqx,srbqy,srbqz,srbqw
                    # associated marker positions
                    nsRigidMarkers = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
                    offset += 4
                    #print 'Marker count=',nsRigidMarkers
                    smd = []
                    smarkerID = []
                    smarkersize = []
                    for i in range(0,nsRigidMarkers):
                        smd.extend(struct.unpack(byteorder+'fff',PacketIn[offset:offset+12]))
                        offset += 12
                    for i in range(0,nsRigidMarkers):
                        smarkerID.append(struct.unpack(byteorder+'I',PacketIn[offset:offset+4])[0])
                        offset += 4
                    for i in range(0,nsRigidMarkers):
                        smarkersize.append(struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0])
                        offset += 4
                    for i in range(0,nsRigidMarkers):
                        pass
                        #print 'Marker ',i+1,' ID=',smarkerID[i],' markerData=',smd[3*i],smd[3*i+1],smd[3*i+2],' markersize=',markersize[i]

                    # marker errors
                    smarkerError = struct.unpack(byteorder+'f',PacketIn[offset:offset+4])[0]
                    offset += 4
                    #print 'Mean marker error=',smarkerError

                    nsr = nsr-1 # next rigidbody of skeleton

                ns = ns-1 # next skeleton

        #latency
        latency = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        offset += 4
        #print 'latency=',latency
        #end of data tag
        eod = struct.unpack(byteorder+'i',PacketIn[offset:offset+4])[0]
        offset += 4
        #print 'end of packet'
    
    return trackableState


if __name__ == '__main__': #{{{1

    # Initialize Multicast Socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
    #s.bind((opti_ip, opti_port))
    s.bind(('', opti_port)) # check if this makes a difference
    mreq = struct.pack('4sl',socket.inet_aton(multicastAdd), socket.INADDR_ANY)
    s.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
    # Initialize UDP Socket
    udpsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


    # Create Drone Sockets - DONT NEED THIS ANYMORE, replaced by address list
    #clientlist = []
    #for x in drone_ip:
    #    client = FMClientUDP(address=(x,drone_port))
    #    clientlist.append(client)
    #    client.start()

    # Receive data
        
    # Create data arrays
    mocap_state = [] 
    prev_state = []
    status = []
    rbid = [] # rigid body id
    pymsg = [] 
	
        
    print "begin recv'ing"
        
    frame_counter = 1.
        
    while (True):
                
        #data, addr = s.recvfrom(20240) #receive data and address from Optitrack Multicast
        data = s.recv(10240) #receive data and address from Optitrack Multicast
                
        if data:
                        
            # Unpack data and skip if data isn't valid
            state = unPack(data)
            if not(state):
                continue
                        
            # Find number of rigid bodies
            nRigidBodies = len(state)  # use this to check if untracked bodies count toward this sum
            #print "Number of Rigid Bodies: ", nRigidBodies
                        
            # Only use 3 trackables - need to change this to add tracked obstacles
            # TODO: Need to add some sort of variable to replace 3
            if nRigidBodies > 9:
                state = state[0:8]
            
            # Append state variables to guarantee data for 3 trackables is sent
            while nRigidBodies < 9:
                state.append([0, 0, 0, 0, 0, 0, 0, 0, 0])
                nRigidBodies = nRigidBodies + 1

            # Initialize the data structures if this is the first time through
            if len(mocap_state) == 0:
                #print "Initializing."
                for i in range(nRigidBodies): # this loops over [0,1,2,...,nRigidBodies-1]
                    mocap_state.append([])
                    status.append(0)
                    rbid.append(0)
                    pymsg.append('')
                    prev_state.append([])
                pymsg.append('')

            #print "Made it past init."
            # Loop over all rigid bodies and pack into pymsg[i]
            for i in range(nRigidBodies):

                rbid[i],fn,x,y,z,qw,qx,qy,qz = state[i] #split the state structure

                # Optitrack Sends Quaternions so convert data
                quat = Quaternion()
                quat.w, quat.x, quat.y, quat.z = qw, qx, qy, qz
                angles = optiquat2euler(quat.vec())
                ## may need to rotate the yaw to match with robot
                #angles0 = EulerAngles(0, -np.pi/2, np.pi/2)
                #yaw = angles.theta;  # check this (is this psi?)

                # Save position and orientation
                mocap_state[i] = np.array([x, y, z, (angles.phi)*180/math.pi, (angles.theta)*180/math.pi, (angles.psi)*180/math.pi])

                # If the rigid body is very close to the origin, assume it is untracked
                # For this rigid body: set status to 0 if untracked, rigid body ID if tracked
                threshold2 = 1e-9
                d = np.dot(mocap_state[i][0:3], mocap_state[i][0:3])
                if d < threshold2:
                    status[i] = 0
                else:
                    status[i] = rbid[i]

                if frame_counter>1:
                    d = np.dot(mocap_state[i][0:3]-prev_state[i][0:3],mocap_state[i][0:3]-prev_state[i][0:3])
                    if d < threshold2:
                        status[i] = 0
                    else:
                        status[i] = rbid[i]

                prev_state[i]=mocap_state[i]

                # Pack X,Z,YAW,STATUS(either 0 or RBID)
                #print "Packing rigid body number %d" % i
                pymsg[i] = struct.pack('fffff', mocap_state[i][0], mocap_state[i][2], mocap_state[i][4], status[i], frame_counter)  # sending rbid instead of status

            # SORT the states so that they are in ascending order - How does this work?
            #mydata = zip(rbid, pymsg)
            #mydata_sorted = sorted(mydata)
            #pymsg_sorted = [d[1] for d in mydata_sorted]
            #pymsg_data = ''.join(pymsg_sorted)	

            # Combine data for all rigid bodies
            #pymsg[nRigidBodies] = struct.pack('f', frame_counter)
            #pymsg_data = ''.join(pymsg)	# Will this join the data?



            # SEND filter_state over UDP
            #target = 0
            #for address in address_list:
            #print "len(pymsg_data) = ", len(pymsg_data)
            #print map(ord,pymsg_data)
            #    if status[target] != 0:
            #        udpsock.sendto(pymsg[target], (address, udp_port))
            #    target = target + 1

            #if frame_counter % 50 == 0:
            #    #print "len(pymsg_data) = ", len(pymsg_data)
            #    print "rbid       status      x          z          yaw"
            #    for i in range(nRigidBodies):
            #        print "%f         %f          %.2f       %.2f       %.2f" % (rbid[i],status[i],mocap_state[i][0], mocap_state[i][2], mocap_state[i][4])

            #frame_counter = frame_counter + 1

            # SEND filter_state over UDP
            if frame_counter % 25 == 0:
                printnow = 1
                wiser_data = requests.get("http://localhost:3101/wiser/api/activetag")
                formatted_data = json.loads(wiser_data.text)
                position_data = formatted_data[0]['location'] #saves xyz location data as a tuple
                #timestamp = formatted_data[0]['timestamp'] #saves time data]
                wiser_Data.append([position_data, mocap_state[1][0],mocap_state[1][2],mocap_state[1][4]])
                #wiser_Data.append([mocap_state[1][0],mocap_state[1][1],mocap_state[1][2]])
                print position_data
                print "rbid       status      x          y          yaw"          
             
            else:
                printnow = 0
            for i in range(nRigidBodies):
            #print "len(pymsg_data) = ", len(pymsg_data)
            #print map(ord,pymsg_data)
                if status[i] != 0:
                    udpsock.sendto(pymsg[i], (address_list[rbid[i]-1], udp_port))
                if printnow == 1:
                    print "%.0f         %.0f          %.3f       %.3f       %.3f" % (rbid[i],status[i],mocap_state[i][0], mocap_state[i][2], mocap_state[i][4])

            if frame_counter % 25000 == 0: # 25000 frames ~= 5.5 min       
                #Assuming res is a flat list
                with open(csvfile, "w") as output:
                    writer = csv.writer(output, lineterminator='\r')
                    i = 0;
                    for val in wiser_Data:
                        writer.writerow([val])
                        
            frame_counter = frame_counter + 1


