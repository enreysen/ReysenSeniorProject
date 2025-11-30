#!/usr/bin/env python3 
from dronekit import connect, Vehicle

from pymavlink import mavutil

import time

import numpy as np
import scipy
from scipy.integrate import cumtrapz

drone_vehicle = connect("tcp:127.0.0.1:5763", wait_ready = True)


x_accels = [0] # holds x accelerations
times = [0] # holds seconds passed from the start time

start_time = time.time()

@drone_vehicle.on_message('RAW_IMU') # define a listener for the RAW_IMU data
# recieve IMU data from ardupilot with mavlink
def get_imu(self, name, message):
    #print('IMU DATA')
    # the accelerometer tells us the linear acceleration (change in velocity)
    #print(f"  Accelerometer (x, y, z): ({message.xacc}, {message.yacc}, {message.zacc})")
    #print(f"  Gyroscope (x, y, z): ({message.xgyro}, {message.ygyro}, {message.zgyro})")

    
    times.append(time.time() - start_time) # append seconds from start time to times list
    x_accels.append(message.xacc) # append x acceleration


def calculate_displacement(x_accels):
    if(len(times) == len(x_accels)):
        x_vel = cumtrapz(np.array(x_accels), np.array(times), initial=0)
        print('\n\nx_vel', x_vel[-1])
    #y_vel = cumtrapz(accels[1], time.time() - start_time, initial=0)
    #z_vel = cumtrapz(accels[2], time.time() - start_time, initial=0)
    
        x_displacement = cumtrapz(np.array(x_vel), np.array(times),  initial =  0)
    #y_displacement = cumtrapz(y_vel, time,  initial =  0)
    #z_displacement = cumtrapz(z_vel, time,  initial =  0)

        print('x_displacement: ', x_displacement[-1])

end_time = time.time() + 3
while (time.time() < end_time):

    #print("x accel", x_accels)
    #print("times", times)

    calculate_displacement(x_accels)

    time.sleep(0.5)
    

# I need to calculate distance
# the accelerometer gives us linear acceleration for x, y, and z

# get velocity of each axis --> we need acceleration from each axis and time

# calculate the distance along each axis --> velocity from each axis and time

# find the total distance
  


# https://dronekit-python.readthedocs.io/en/latest/guide/mavlink_messages.html