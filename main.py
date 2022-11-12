#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.iodevices import Ev3devSensor

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()

# Wheels to b and c for correct lego calibration
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)


# drivebase args l motor, r motor, w diameter, w distance
# left_motor = Motor(portL, positive_direction=Direction.CLOCKWISE, gears=None)
# right_motor = Motor(portR, positive_direction=Direction.CLOCKWISE, gears=None)
base = DriveBase(left_motor, right_motor, 60, 150)
left_ir_sensor = Ev3devSensor(Port.S4)
right_ir_sensor = Ev3devSensor(Port.S3)

left_ir_sensor_value = left_ir_sensor.read("DC")
right_ir_sensor_value = right_ir_sensor.read("DC")
def updateIrValue():
    new_ir_sensor_value = ir_sensor.read("DC")
    return new_ir_sensor_value

def movement(left_ir_sensor, right_ir_sensor):
    left_ir_sensor_value = left_ir_sensor.read("DC")
    right_ir_sensor_value =right_ir_sensor.read("DC")
    if(right_ir_sensor_value[0] == 0 and left_ir_sensor_value[0] == 0):
        base.turn(90)

    if(right_ir_sensor_value[0] <3 and left_ir_sensor_value[0] > 7):'
    #Method to look for the goal
        base.straight(1000)

    if right_ir_sensor_value[0] >5:
        base.turn(30)
        movement(ir_)

    if ir_sensor_value[0] < 5:
        base.turn(-30)
        movement(ir_sensor)
    
movement(ir_sensor)

