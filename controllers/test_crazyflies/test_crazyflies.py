# -*- coding: utf-8 -*-
#
#  ...........       ____  _ __
#  |  ,-^-,  |      / __ )(_) /_______________ _____  ___
#  | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  | / ,..´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#     +.......   /_____/_/\__/\___/_/   \__,_/ /___/\___/

# MIT License

# Copyright (c) 2023 Bitcraze


"""
file: crazyflie_py_wallfollowing.py

Controls the crazyflie and implements a wall following method in webots in Python

Author:   Kimberly McGuire (Bitcraze AB)
"""


from controller import Robot
from controller import Keyboard

from math import cos, sin

from pid_controller import pid_velocity_fixed_height_controller
from wall_following import WallFollowing

FLYING_ATTITUDE = 4

class Maya(Robot):

    def __init__(self):
        super().__init__()

        timestep = int(self.getBasicTimeStep())

        # Initialize motors
        self.m1_motor = self.getDevice("m1_motor")
        self.m1_motor.setPosition(float('inf'))
        self.m1_motor.setVelocity(-1)
        self.m2_motor = self.getDevice("m2_motor")
        self.m2_motor.setPosition(float('inf'))
        self.m2_motor.setVelocity(1)
        self.m3_motor = self.getDevice("m3_motor")
        self.m3_motor.setPosition(float('inf'))
        self.m3_motor.setVelocity(-1)
        self.m4_motor = self.getDevice("m4_motor")
        self.m4_motor.setPosition(float('inf'))
        self.m4_motor.setVelocity(1)

        # Initialize Sensors
        self.imu = self.getDevice("inertial_unit")
        self.imu.enable(timestep)
        self.gps = self.getDevice("gps")
        self.gps.enable(timestep)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(timestep)
        self.camera = self.getDevice("camera")
        self.camera.enable(timestep)
        self.range_front = self.getDevice("range_front")
        self.range_front.enable(timestep)
        self.range_left = self.getDevice("range_left")
        self.range_left.enable(timestep)
        self.range_back = self.getDevice("range_back")
        self.range_back.enable(timestep)
        self.range_right = self.getDevice("range_right")
        self.range_right.enable(timestep)

        # Initialize variables
        self.past_x_global = 0
        self.past_y_global = 0
        self.past_time = 0
        self.first_time = True

        # Crazyflie velocity PID controller
        self.PID_crazyflie = pid_velocity_fixed_height_controller()
        self.PID_update_last_time = self.getTime()
        self.sensor_read_last_time = self.getTime()

        self.height_desired = FLYING_ATTITUDE

        # Initialize values
        self.desired_state = [0, 0, 0, 0]
        self.forward_desired = 0
        self.sideways_desired = 0
        self.yaw_desired = 0
        self.height_diff_desired = 0


    def run(self):

        dt = self.getTime() - self.past_time
        actual_state = {}

        if self.first_time:
            self.past_x_global = self.gps.getValues()[0]
            self.past_y_global = self.gps.getValues()[1]
            self.past_time = self.getTime()
            self.first_time = False

        # Get sensor data
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        yaw = self.imu.getRollPitchYaw()[2]
        yaw_rate = self.gyro.getValues()[2]
        x_global = self.gps.getValues()[0]
        v_x_global = (x_global - self.past_x_global)/dt
        y_global = self.gps.getValues()[1]
        v_y_global = (y_global - self.past_y_global)/dt
        altitude = self.gps.getValues()[2]

        # Get body fixed velocities
        cos_yaw = cos(yaw)
        sin_yaw = sin(yaw)
        v_x = v_x_global * cos_yaw + v_y_global * sin_yaw
        v_y = - v_x_global * sin_yaw + v_y_global * cos_yaw



        self.height_desired += self.height_diff_desired * dt

        camera_data = self.camera.getImage()

        # get range in meters
        range_front_value = self.range_front.getValue() / 1000
        range_right_value = self.range_right.getValue() / 1000
        range_left_value = self.range_left.getValue() / 1000

        # # Choose a wall following direction
        # # if you choose direction left, use the right range value
        # # if you choose direction right, use the left range value
        # direction = WallFollowing.WallFollowingDirection.LEFT
        # range_side_value = range_right_value

        # PID velocity controller with fixed height
        motor_power = self.PID_crazyflie.pid(dt, self.forward_desired, self.sideways_desired,
                                        self.yaw_desired, self.height_desired,
                                        roll, pitch, yaw_rate,
                                        altitude, v_x, v_y)

        self.m1_motor.setVelocity(-motor_power[0])
        self.m2_motor.setVelocity(motor_power[1])
        self.m3_motor.setVelocity(-motor_power[2])
        self.m4_motor.setVelocity(motor_power[3])

        self.past_time = robot.getTime()
        self.past_x_global = x_global
        self.past_y_global = y_global


    def up(self,value):
        self.height_diff_desired = value
    def down(self,value):
        self.height_diff_desired = -value
    def forward(self,value):
        self.forward_desired = value
    def sideways(self,value):
        self.yaw_desired = value

if __name__ == '__main__':

    robot = Maya()
    timestep = int(robot.getBasicTimeStep())
    flag_X=False
    flag_y=False
    Coordinate=[(-50.8656,73.0636,4),(-56.8528,79.5066,4)]
    i=0
    # Main loop:
    while robot.step(timestep) != -1:
        print(robot.gps.getValues())
        
        robot.run()
        if(robot.gps.getValues()[2]<Coordinate[i][2]):
            robot.up(0.1)
            flag=False
        else:
            robot.up(0)
            flag=True
        if(robot.gps.getValues()[2]>Coordinate[i][2]+0.5):
            robot.down(0.1)
            flag=False
        else:
            robot.down(0)
            flag=True
        if((robot.gps.getValues()[2]<Coordinate[i][2]+0.5) and (robot.gps.getValues()[2]>Coordinate[i][2]-0.5)):
            print("ici",Coordinate[i][2],robot.gps.getValues()[0]<Coordinate[i][2]+0.5,robot.gps.getValues()[2]>Coordinate[i][2]-0.5)
            if(robot.gps.getValues()[0]<Coordinate[i][0]-0.2):
                robot.forward(0.1)
            elif(robot.gps.getValues()[0]>Coordinate[i][0]+0.2):
                robot.forward(-0.1)
            else:
                robot.forward(0)
                flag_X=True
                
            if(robot.gps.getValues()[1]<Coordinate[i][1]-0.2 ):
                robot.sideways(0.2)
            elif(robot.gps.getValues()[1]>Coordinate[i][1]+0.2 ):
                robot.sideways(-0.2)
                
            else:
                robot.sideways(0)
                flag_Y=True
        else:
            robot.forward(0)
            robot.sideways(0)
        if(flag_X and flag_Y):
            flag_X=False
            flag_Y=False
            print(Coordinate[i])
            i+=1
                

        
