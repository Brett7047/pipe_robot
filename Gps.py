#! /usr/bin/env python3
# Date Created : Spring 2021
# Author : Brett Ryan, ryanbm@sunypoly.edu
# Language: Python3.7.3
import serial
import pynmea2
import math
import csv
import sys

class GPS:
    def __init__(self):
        try:
            self.gps = serial.Serial(port="/dev/ttyACM0",baudrate=9600,timeout=1)
            if self.gps.isOpen() == False:
                self.gps.open()
        except serial.SerialException as e:
            print(e)
            sys.exit()
        self.data = 0
        self.initial_lat = 0
        self.initial_lon = 0
        self.last_lat = 0
        self.last_lon = 0
        self.acquired_initial_gps = False
        self.file = open("gps_points.txt","w")
        self.writer = csv.writer(self.file)

    def get_initial_point(self,angle=0):
        gpsfile_header_string =['type','latitude','longitude','angle','name','desc'] # format for csv file header for gpsvisualizer.com
        print("Acquiring GPS Lock . . .")
        while self.acquired_initial_gps == False:
            try:
                self.gps_data = pynmea2.parse(self.gps.readline().decode())
            except pynmea2.nmea.ParseError as e:
                print(e)
            if self.gps_data.sentence_type != "GGA":
                pass
            else:
                self.initial_lat = round(self.gps_data.latitude,6)
                self.initial_lon = round(self.gps_data.longitude,6)
                #self.last_lat = self.initial_lat # for use in compute_latlon
                #self.last_lon = self.initial_lon # for use in compute_latlon
                self.writer.writerow((gpsfile_header_string)) # prints gps string header to csv file
                self.writer.writerow(("T",self.initial_lat,self.initial_lon,angle)) # writes to csv file
                break
        print("Acquired GPS lock. Currently at:\n",self.initial_lat," N\n",self.initial_lon," W")

    def compute_latlon(self,bearing,distance,leak=False,angle=0):
        """
        bearing - degree from 0-360 with 0 being north and 180 being south
        distance - linear distance in millimeters
        leak - bool if detected leak or not. Will output gps point as waypoint, default=False
        """
        # https://stackoverflow.com/questions/7222382/get-lat-long-given-current-point-distance-and-bearing
        R = 6378.1 #Radius of the Earth
        brg = ((bearing * math.pi) / 180)  #brg is bearing converted to radians(its our robot heading)
        dst = (distance /1000000) #convert from mm to km
        #lat1 = math.radians(self.last_lat) #Current lat point converted to radians
        #lon1 = math.radians(self.last_lon) #Current long point converted to radians
        lat1 = math.radians(self.initial_lat)
        lon1 = math.radians(self.initial_lon)

        lat2 = math.asin( math.sin(lat1)*math.cos(dst/R) +
            math.cos(lat1)*math.sin(dst/R)*math.cos(brg))
        lon2 = lon1 + math.atan2(math.sin(brg)*math.sin(dst/R)*math.cos(lat1),
                    math.cos(dst/R)-math.sin(lat1)*math.sin(lat2))

        lat2 = round(math.degrees(lat2),6)
        lon2 = round(math.degrees(lon2),6)
        if leak==False:
            self.writer.writerow(("T",lat2,lon2,angle)) # writes to csv file
        else:
            distance_detected = round(((distance*0.0393701)/12),3)
            print("distance is: ",distance)
            print("distance_detected is: ",distance_detected)
            leak_csv_string = str(distance_detected)+" feet down the pipe"
            self.writer.writerow(("W",lat2,lon2,angle,"Leak",leak_csv_string)) # writes to csv file            
        #self.last_lat = lat2 # for next calc
        #self.last_lon = lon2 # for next calc

    def __del__(self):
        try:
            if self.gps.isOpen() == True:
                self.gps.close() 
            self.file.close()
        except AttributeError:
            pass

if __name__ == "__main__":
    # for testing this script. The script itself is mainly used in main.py
    gps = GPS()
    gps.get_initial_point()
    gps.compute_latlon(0,6000)
