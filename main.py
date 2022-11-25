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
#compass sensor not in use
compass_sensor = Ev3devSensor(Port.S1)
#color_sensor = Ev3devSensor(Port.S1)

left_ir_sensor_value = left_ir_sensor.read("DC")
right_ir_sensor_value = right_ir_sensor.read("DC")
lightSensor = LightSensor(Port.S2)

#compass_value = compass_sensor.read("GYRO")
#light_sensor_value = light_sensor.read("REFLECT")#Available modes reflect and ambient

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

    if right_ir_sensor_value[0]>5:#6
        print("ball slightly to the right")
        base.turn(20)
        movement(left_ir_sensor, right_ir_sensor)

    if left_ir_sensor_value[0] < 5:#4
        print("ball slightly to the left")
        base.turn(-25)
        movement(left_ir_sensor, right_ir_sensor)
    else:
        #base.straight(150)
        getGoalDir()
        movement(left_ir_sensor, right_ir_sensor)

movement(left_ir_sensor, right_ir_sensor)

