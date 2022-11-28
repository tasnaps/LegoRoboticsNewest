#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor
from pybricks.nxtdevices import LightSensor

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.

ev3 = EV3Brick()

# Wheels on b and c for correct lego calibration
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# DriveBase constructor values might need adjustments
base = DriveBase(left_motor, right_motor, 60, 150)


left_ir_sensor = Ev3devSensor(Port.S3)
right_ir_sensor = Ev3devSensor(Port.S4)

#Center IR sensor for detecting the black line used in goal tracking
center_ir_sensor = Ev3devSensor(Port.S2)

left_ir_sensor_value = left_ir_sensor.read("DC")
right_ir_sensor_value = right_ir_sensor.read("DC")
center_ir_sensor_value = center_ir_sensor.read("DC")

#Light sensor optional for goal tracking instead of black line and ir sensor
#lightSensor = LightSensor(Port.S2)

# when we set the robot in arena, must face forward we update this in degrees turned
heading = 0
locationToLine = 0


#Optional function for approaching goal
def getGoalDir():
    print("Getting goal direction")
    lightValue = lightSensor.ambient()


    if(lightValue>=12):
        #Goal in front based on light value
        base.straight(200)
        movement(left_ir_sensor, right_ir_sensor)

    if(lightValue<12):
        print("Light value less than 12: " + str(lightValue))
        base.turn(40)


def getRotation():
    #Cleaning data
    newHeading = heading/360
    while(newHeading>1 or newHeading<-1):
        if newHeading>1:
            newHeading =-1
        if newHeading<-1:
            newHeading =+1
    #cleaned heading returning a decimal doesnt matter if positive or negative for now
    #newHeading = newHeading * 360
    return abs(newHeading)


def updateLocation(distance):
    tempHeading = getRotation
    #Update on the location based on which side we have moved. 
    if tempHeading<0.5:
        locationToLine += distance
    if tempHeading>0.5:
        locationToLine -= distance

        
def moveToLine():
    tempHeading = getRotation() * 360

    #we are on the left side of the line
    rotation = -tempHeading + 90
    if locationToLine<0:
        base.turn(rotation)
        readAndMove()
        base.turn(-90)
        base.straight(10000)
    
    #we are on the right side of the line
    if locationToLine>0:
        base.turn(rotation+180)
        readAndMove()
        base.turn(90)
        base.straight(10000)

    #read the third sensor value and move forward until we hit the black line
    #once we hit the line we turn depending on approach and move forward.

def readAndMove():
    irValue = center_ir_sensor.read("DC")
    #TODO Need to check what value we get on black line 
    if irValue < 255:
        base.straight(50)
        readAndMove()

    

def detectColors():
    color_sensor_value = color_sensor.read("RGB")
    
    if(color_sensor_value[0]==255):
        return "red"
    if(color_sensor_value[1]==255):
        return "green"
    if(color_sensor_value[2]==255):
        return "blue"
    else:
        return "none"





# one option is to only use one sensor, if we install it on the middle. Then the values should be ok when ball is in the middle.
def movement(left_ir_sensor, right_ir_sensor):
    left_ir_sensor_value = left_ir_sensor.read("DC")
    right_ir_sensor_value = right_ir_sensor.read("DC")

    ## ball detected behind on both sensors
    if(right_ir_sensor_value[0] == 0 and left_ir_sensor_value[0] == 0):
        print("ball behind")
        base.turn(40)
        heading =+40

    if right_ir_sensor_value[0]>5:#6
        print("ball slightly to the right")
        base.turn(20)
        heading =+20
        movement(left_ir_sensor, right_ir_sensor)

    if left_ir_sensor_value[0] < 5:#4
        print("ball slightly to the left")
        base.turn(-25)
        heading =-20
        movement(left_ir_sensor, right_ir_sensor)
    else:
        base.straight(150)
        updateLocation(150)
        movement(left_ir_sensor, right_ir_sensor)

movement(left_ir_sensor, right_ir_sensor)

