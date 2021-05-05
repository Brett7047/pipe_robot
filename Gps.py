#! /usr/bin/env python3

import serial
import pynmea2
import math

class GPS:
    def __init__(self):
        self.gps = serial.Serial(port="/dev/ttyACM0",baudrate=9600,timeout=1)
        self.data = 0
        self.initial_lat = 0
        self.initial_lon = 0
        self.last_lat = 0
        self.last_lon = 0
        self.acquired_initial_gps = False
        self.file = open("gps_points.txt","w")

    def get_initial_point(self):
        print("Acquiring GPS Lock . . .")
        while self.acquired_initial_gps == False:
            self.gps_data = pynmea2.parse(self.gps.readline().decode())
            if self.gps_data.sentence_type != "GGA":
                pass
            else:
                self.initial_lat = round(self.gps_data.latitude,6)
                self.initial_lon = round(self.gps_data.longitude,6)
                self.last_lat = self.initial_lat # for use in compute_latlon
                self.last_lon = self.initial_lon # for use in compute_latlon
                self.file.write(self.initial_lat + "," + self.initial_lon + "\n")
                break
        print("Acquired GPS lock. Currently at:\n",self.initial_lat," N\n",self.initial_lon," W")

    def compute_latlon(self,bearing,distance):
        # https://stackoverflow.com/questions/7222382/get-lat-long-given-current-point-distance-and-bearing
        R = 6378.1 #Radius of the Earth
        brg = ((bearing * math.pi) / 180)  #brg is bearing converted to radians(its our robot heading)
        dst = (distance /1000000) #convert from mm to km
        lat1 = math.radians(self.last_lat) #Current lat point converted to radians
        lon1 = math.radians(self.last_lon) #Current long point converted to radians

        lat2 = math.asin( math.sin(lat1)*math.cos(dst/R) +
            math.cos(lat1)*math.sin(dst/R)*math.cos(brg))
        lon2 = lon1 + math.atan2(math.sin(brg)*math.sin(dst/R)*math.cos(lat1),
                    math.cos(dst/R)-math.sin(lat1)*math.sin(lat2))

        lat2 = round(math.degrees(lat2),6)
        lon2 = round(math.degrees(lon2),6)
        self.file.write(lat2 + "," + lon2 + "\n")
        self.last_lat = lat2
        self.last_lon = lon2

    def __del__(self):
        if self.gps.isOpen() == True:
            self.gps.close()


if __name__ == "__main__":
    gps = GPS()
    gps.get_initial_point()
    gps.compute_latlon(0,6000)
