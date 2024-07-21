from pybricks.hubs import PrimeHub # type: ignore
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor # type: ignore
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop # type: ignore
from pybricks.robotics import DriveBase # type: ignore
from pybricks.tools import wait, StopWatch # type: ignore


hub = PrimeHub(observe_channels=[1])

LF_MOVE = Motor(Port.A)
LB_MOVE = Motor(Port.E)
RF_MOVE = Motor(Port.B)
RB_MOVE = Motor(Port.F)

def moveAtSpeed(LFspeed,LBspeed,RFspeed,RBspeed):
    LF_MOVE.run(LFspeed)
    LB_MOVE.run(LBspeed)
    RF_MOVE.run(RFspeed)
    RB_MOVE.run(RBspeed)


while True:
    data = hub.ble.observe(1)
    if data is not None:
        print(data)
        moveAtSpeed(data,data,data,data)