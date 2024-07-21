
from pybricks.hubs import PrimeHub # type: ignore
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor, ForceSensor # type: ignore
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop # type: ignore
from pybricks.robotics import DriveBase # type: ignore
from pybricks.tools import wait, StopWatch # type: ignore
from pybricks.iodevices import XboxController # type: ignore
import umath as math # type: ignore
import  math
hub = PrimeHub(broadcast_channel=1)
controller = XboxController()

LF_TURN = Motor(Port.A)
LB_TURN = Motor(Port.E)
RF_TURN = Motor(Port.B)
RB_TURN = Motor(Port.F)

def moveToAngle(LFangle,LBangle,RFangle,RBangle):
    LF_TURN.track_target(LFangle)
    LB_TURN.track_target(LBangle)
    RF_TURN.track_target(RFangle)
    RB_TURN.track_target(RBangle)

def arctan(x,y):
    if x>0:
        return -1*(math.atan(y/x)-math.pi/2)
    if y<0 and x<0:
        return 3*math.pi/2 - math.atan(y/x)
    if y>=0 and x<0:
        return 3*math.pi/2 - math.atan(y/x)
    if y>0 and x==0:
        return 0
    if y<0 and x==0:
        return math.pi
    if y==0 and x==0:
        return 0

def calculateVector(cords):
    angle = arctan(cords[0],cords[1])/math.pi*180
    dist = math.sqrt(cords[0]**2 + cords[1]**2)
    return angle,dist
def signum(x):
    return 1 if x>0 else -1 if x<0 else 0

def closestAngle(current,angle):
    dirc = angle%360 - current%360
    if(abs(dirc)>180):
        dirc = -1*(signum(dirc)*360)+dirc
    return dirc

def calculateAngle(current,target_angle):
    setpointAngle  = closestAngle(current,target_angle)
    setpointAngleFlipped = closestAngle(current,target_angle+180)
    if abs(setpointAngle)<=abs(setpointAngleFlipped):
        angleToSet = (setpointAngle+current)%360
    else:
        angleToSet = (setpointAngleFlipped+current)%360
    return angleToSet   

def translate(angle,speed):
    moveToAngle(angle,angle,angle,angle)
    hub.ble.broadcast(speed)

def inPlaceTurn(speed):
    moveToAngle(135,45,-45,-135)
    hub.ble.broadcast(speed)

def traslateTurn(angle,translatePower, turnPower):
    turnAngle = turnPower*45
    if closestAngle(angle,135)>=90:
        LF  = angle+turnAngle
    else:
        
        LF = angle-turnAngle
    if closestAngle(angle,225)>=90:
        LB = angle+turnAngle
    else:
        LB = angle-turnAngle
    if closestAngle(angle,45)>=90:
        RF = angle+turnAngle
    else:
        RF = angle-turnAngle
    if closestAngle(angle,315)>=90:
        RB = angle+turnAngle
    else:
        RB=angle-turnAngle
    moveToAngle(LF,LB,RF,RB)
    hub.ble.broadcast(translatePower)

def setSwerveDrive(angle,translatePower, turnPower):
    if translatePower == 0 and turnPower != 0:
        inPlaceTurn(turnPower)
    else:
        traslateTurn(angle,translatePower, turnPower)


while True:
    if hub.imu.ready():
        hub.light.on(Color.GREEN)
        strafe_control = controller.joystick_left()
        turn_control = controller.joystick_right()[0]
        strafe_angle,strafe_dist = calculateVector(strafe_control)
        # for the angle to be correct, we need to subtract the current heading
        strafe_angle-=hub.imu.heading()
        setSwerveDrive(strafe_angle,strafe_dist,turn_control)
        # angle is from 0 to 360 with 0 and 360 at the top
        # dist is 0 to 120
        last = calculateAngle(last,strafe_angle)
        print(last,strafe_angle)
    else:
        hub.light.off()