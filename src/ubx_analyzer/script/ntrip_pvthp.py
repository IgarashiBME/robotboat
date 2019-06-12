#!/usr/bin/python
# coding:utf-8

import serial
import rospy
import binascii
import struct
import numpy as np
from pyproj import Proj

# ROS message form
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from ubx_analyzer.msg import NavPVT 
from ubx_analyzer.msg import UTMHP

Header_A = "b5"  #Ublox GNSS receiver, UBX protocol header
Header_B = "62"  #Ublox GNSS receiver, UBX protocol header
NAV_Id = "01"    #UBX-NAV-PVT class header(0x01 0x07)
PVT_Id = "07"
NAV_PVT_Length = 96    #length of UBX-NAV-PVT hex data

HPPOSLLH_Id = "14"
NAV_HPPOSLLH_Length = 36

RXM_CLASS = "02"
RTCM_ID = "32"

# nmea header
HEADER_NMEA = "$"
HEADER_GNGGA = "GNGGA"

class ublox():
    def __init__(self):
        # Set up serial:
        port = rospy.get_param('~port', '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00')
        self.ser = serial.Serial(
            port,\
            baudrate=115200,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
            timeout=1)

        rospy.on_shutdown(self.shutdown)

        # ROS publisher initialize
        self.pub_gpst = rospy.Publisher('gpstime', String, queue_size = 1)
        self.pub_navsat = rospy.Publisher('RTK_NavSatFix', NavSatFix, queue_size = 1)
        self.pub_utm = rospy.Publisher('utm', Odometry, queue_size = 1)
        self.pub_navpvt = rospy.Publisher('navpvt', NavPVT, queue_size = 1)
        self.pub_utm_hp = rospy.Publisher('utm_hp', UTMHP, queue_size = 1)
        self.pub_gngga = rospy.Publisher('gngga', String, queue_size = 1)
        self.navsat = NavSatFix()
        self.utm = Odometry()
        self.navpvt_data = NavPVT()
        self.utm_hp = UTMHP()

        # ROS subscriber
        # RTCM message subscriber, messages from ntrip node
        rospy.Subscriber('/ntrip_rtcm', String, self.rtcm_subs, queue_size=1)

        # initialize the valiable
        self.fix_status = 0
        self.satellites = 0

    def loop(self):
        # searching for UBX-NAV-Class headers
        while not rospy.is_shutdown():
            Ascii = self.ser.read()
            Data = binascii.b2a_hex(Ascii)

            if Data == Header_A:
                Data = binascii.b2a_hex(self.ser.read())
 
                if Data == Header_B:
                    Data = binascii.b2a_hex(self.ser.read())

                    if Data == NAV_Id:
                        Data = binascii.b2a_hex(self.ser.read())

                        if Data == PVT_Id:
                            NAV_PVT_Data = bytearray(self.ser.read(NAV_PVT_Length))
                            #print binascii.b2a_hex(NAV_PVT_Data)
                            self.PVT_Function(NAV_PVT_Data)

                        elif Data == HPPOSLLH_Id:
                            NAV_HPPOSLLH_Data = bytearray(self.ser.read(NAV_HPPOSLLH_Length))
                            #print binascii.b2a_hex(NAV_HPPOSLLH_Data)
                            self.HPPOSLLH_Function(NAV_HPPOSLLH_Data)
                    elif Data == RXM_CLASS:
                        # UBX-RXM-RTCM checker
                        Data = binascii.b2a_hex(self.ser.read())
                        #if Data == RTCM_ID:
                            #print "RTCM"
                            

            if Ascii == HEADER_NMEA:
                #print self.ser.readline()
                nmea = "$" + self.ser.readline()
                if nmea[1:6] == HEADER_GNGGA:
                    self.pub_gngga.publish(nmea)

    def HPPOSLLH_Function(self, Data):
        #print binascii.b2a_hex(Data)
        if Data.__len__() == NAV_HPPOSLLH_Length:
            gpst = float(struct.unpack('I', struct.pack('BBBB', Data[6], Data[7],
                                       Data[8], Data[9]))[0])
            longitude = float(struct.unpack('i', struct.pack('BBBB', Data[10],
                                            Data[11], Data[12], Data[13]))[0])/10000000.0
            latitude = float(struct.unpack('i', struct.pack('BBBB', Data[14], 
                                            Data[15], Data[16], Data[17]))[0])/10000000.0
            height = float(struct.unpack('i', struct.pack('BBBB', Data[18], Data[19], 
                                         Data[20], Data[21]))[0])/1000.0

            # High Precision
            longitudeHp = float(struct.unpack('b', struct.pack('B', Data[26]))[0])/1000000000.0
            latitudeHp  = float(struct.unpack('b', struct.pack('B', Data[27]))[0])/1000000000.0
            heightHp    = float(struct.unpack('b', struct.pack('B', Data[28]))[0])/10000.0

            longitudeHp = longitude + longitudeHp
            latitudeHp  = latitude + latitudeHp
            heightHp    = height + heightHp

            # accuracy
            hAcc = float(struct.unpack('I', struct.pack('BBBB', Data[30], Data[31],
                                                         Data[32], Data[33]))[0])/10000.0
            vAcc = float(struct.unpack('I', struct.pack('BBBB', Data[30], Data[31],
                                                         Data[32], Data[33]))[0])/10000.0

            # UTM
            utmzone = int((longitude + 180)/6) +1   # If you are on the specific location, can't be calculated. 
            convertor = Proj(proj='utm', zone=utmzone, ellps='WGS84')
            x, y = convertor(longitudeHp, latitudeHp)
            
            #publish navsatfix
            self.navsat.header.stamp = rospy.Time.now()
            self.navsat.header.frame_id = "gnss_link"
            self.navsat.status.status = self.fix_status
            self.navsat.latitude = latitudeHp
            self.navsat.longitude = longitudeHp
            self.navsat.altitude = heightHp
            self.navsat.position_covariance = np.array([(hAcc*hAcc)/2,           0.0,           0.0,
                                                                  0.0, (hAcc*hAcc)/2,           0.0,
                                                                  0.0,           0.0, (vAcc*vAcc)/3])
            self.pub_navsat.publish(self.navsat)
            
            # Publish utm_hp
            self.utm_hp.iTOW = gpst
            self.utm_hp.numSV = self.satellites
            self.utm_hp.fix_status = self.fix_status
            self.utm_hp.lonHp = longitudeHp
            self.utm_hp.latHp = latitudeHp
            self.utm_hp.utm_easting = x
            self.utm_hp.utm_northing = y
            self.utm_hp.heightHp = heightHp
            self.utm_hp.hAcc = hAcc
            self.utm_hp.vAcc = vAcc
            self.pub_utm_hp.publish(self.utm_hp)

            # publish odometry message (UTM coodinate)
            self.utm.header.stamp = rospy.Time.now()
            self.utm.pose.pose.position.x = x
            self.utm.pose.pose.position.y = y
            self.utm.pose.pose.position.z = heightHp
            self.pub_utm.publish(self.utm)

            #print "HPPOSLLH"
            #print gpst
            #print "lat:"+str(latitudeHp), "lon:"+str(longitudeHp), "alt:"+ str(heightHp)
            #print "east:" + str(x), "north:" + str(y)
            #print
    
    def PVT_Function(self, NAV_PVT_Data):
        #print binascii.b2a_hex(NAV_PVT_Data)

        if NAV_PVT_Data.__len__() == NAV_PVT_Length:
            # decode Time data
            gpst = float(struct.unpack('I', struct.pack('BBBB', NAV_PVT_Data[2], NAV_PVT_Data[3],
                                       NAV_PVT_Data[4], NAV_PVT_Data[5]))[0])
            year = int(struct.unpack('h', struct.pack('BB', NAV_PVT_Data[6], NAV_PVT_Data[7]))[0])
            month = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[8]))[0])
            day = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[9]))[0])
            hour = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[10]))[0])
            minute = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[11]))[0])
            second = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[12]))[0])
            
            # decode RTK fix flag
            fix_flag = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[23]))[0])
            fix_flag = bin(fix_flag)[2:].zfill(8)
            if fix_flag[0:2] == "10":
                self.fix_status = 2    # when rtk was fixed, publish 2
                fix_str = "RTK fixed solution"
            elif fix_flag[0:2] == "01":
                self.fix_status= 1     # when rtk was float, publish 1
                fix_str = "RTK float solution"
            else:
                self.fix_status= 0
                fix_str = "No carrier phase renge solution"

            # decode Number of satellites used in Nav Solution
            self.satellites = int(struct.unpack('B', struct.pack('B', NAV_PVT_Data[25]))[0])

            # decode Coordinate data
            longitude = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[26],
                                            NAV_PVT_Data[27], NAV_PVT_Data[28], 
                                            NAV_PVT_Data[29]))[0])/10000000.0
            latitude = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[30], 
                                            NAV_PVT_Data[31], NAV_PVT_Data[32], 
                                            NAV_PVT_Data[33]))[0])/10000000.0
            height = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[34], NAV_PVT_Data[35], 
                                         NAV_PVT_Data[36], NAV_PVT_Data[37]))[0])/1000.0

            # sea level
            hMSL = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[38], NAV_PVT_Data[39], 
                                                             NAV_PVT_Data[40], NAV_PVT_Data[41]))[0])/1000.0

            # accuracy
            hAcc = float(struct.unpack('I', struct.pack('BBBB', NAV_PVT_Data[42], NAV_PVT_Data[43],
                                                              NAV_PVT_Data[44], NAV_PVT_Data[45]))[0])
            vAcc = float(struct.unpack('I', struct.pack('BBBB', NAV_PVT_Data[46], NAV_PVT_Data[47],
                                                              NAV_PVT_Data[48], NAV_PVT_Data[49]))[0])
            pDOP = int(struct.unpack('h', struct.pack('BB', NAV_PVT_Data[78], NAV_PVT_Data[79]))[0])

            # velocity
            velN = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[50], NAV_PVT_Data[51], 
                                         NAV_PVT_Data[52], NAV_PVT_Data[53]))[0])/1000.0
            velE = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[54], NAV_PVT_Data[55], 
                                         NAV_PVT_Data[56], NAV_PVT_Data[57]))[0])/1000.0
            velD = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[58], NAV_PVT_Data[59], 
                                         NAV_PVT_Data[60], NAV_PVT_Data[61]))[0])/1000.0
            gSpeed = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[62], NAV_PVT_Data[63], 
                                         NAV_PVT_Data[64], NAV_PVT_Data[65]))[0])/1000.0

            # heading
            headMot = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[66], NAV_PVT_Data[67], 
                                         NAV_PVT_Data[68], NAV_PVT_Data[69]))[0])/1000.0            
            sAcc = float(struct.unpack('I', struct.pack('BBBB', NAV_PVT_Data[70], NAV_PVT_Data[71],
                                                              NAV_PVT_Data[72], NAV_PVT_Data[73]))[0])
            headAcc = float(struct.unpack('I', struct.pack('BBBB', NAV_PVT_Data[74], NAV_PVT_Data[75],
                                                              NAV_PVT_Data[76], NAV_PVT_Data[77]))[0])
            headVeh = float(struct.unpack('i', struct.pack('BBBB', NAV_PVT_Data[86], NAV_PVT_Data[87], 
                                         NAV_PVT_Data[88], NAV_PVT_Data[89]))[0])/1000.0   

            # publish rostime and gpstime
            self.pub_gpst.publish(str(rospy.Time.now()) +"," +str(gpst))

            # publish GNSS status
            self.navpvt_data.iTOW = gpst
            self.navpvt_data.year = year
            self.navpvt_data.month = month
            self.navpvt_data.day = day
            self.navpvt_data.hour = hour
            self.navpvt_data.min = minute
            self.navpvt_data.sec = second
            self.navpvt_data.numSV = self.satellites
            self.navpvt_data.fix_status = self.fix_status
            self.navpvt_data.lon = longitude
            self.navpvt_data.lat = latitude
            self.navpvt_data.height = height
            self.navpvt_data.hMSL = hMSL
            self.navpvt_data.hAcc = hAcc
            self.navpvt_data.vAcc = vAcc
            self.navpvt_data.pDOP = pDOP

            self.navpvt_data.velN = velN
            self.navpvt_data.velE = velE
            self.navpvt_data.velD = velD
            self.navpvt_data.headMot = headMot
            self.navpvt_data.sAcc = sAcc
            self.navpvt_data.headAcc = headAcc
            self.navpvt_data.headAcc = headVeh
            self.pub_navpvt.publish(self.navpvt_data)            

            # print section
            #print gpst
            time_data = str(year) + "," + str(month) + "," + str(day) + "," + str(hour) \
                        + "," + str(minute) + "," + str(second)
            coordinate_data = str(latitude) + "," + str(longitude) + "," + str(height)
            #print "NavPVT"
            #print gpst
            #print time_data
            #print fix_str, self.fix_status, fix_flag[0:2]
            #print coordinate_data
            #print "east:" + str(x), "north:" + str(y)
            #print

    def rtcm_subs(self, msg):
        # subscribe RTCM and write serial
        rtcm = binascii.a2b_hex(msg.data)
        self.ser.write(rtcm)

    def shutdown(self):
        rospy.loginfo("ublox analyzer node was terminated") 

if __name__ == '__main__':
    rospy.init_node("ublox_analyzer_node")
    u = ublox()
    u.loop()
        
