#!/usr/bin/env python3
from bluedot import BlueDot
from signal import pause
import unipolarDirect

class bluemotor():
    def __init__(self):
        self.motor=unipolarDirect.StartMotor( name='unitest')
        self.speed = 0
        self.speedlimit=self.motor['settings/maxrpm'].getCurrent()

    def dpad(self, pos):
        if pos.top:
            print("up")
        elif pos.bottom:
            print("down")
        elif pos.left:
            olds=self.speed
            print("left")
            if round(self.speed) == 1:
                self.speed=0
            elif round(self.speed) == 0:
                self.speed=-1
            elif self.speed > 0:
                self.speed /= 2
            else:
                self.speed *= 2
                if abs(self.speed) > self.speedlimit:
                    self.speed = -self.speedlimit
            print('left: was %s, now %s' % (olds, self.speed))
            self.motor.setspeed(speed=self.speed)
        elif pos.right:
            olds=self.speed
            if round(self.speed)==-1:
                self.speed=0
            elif round(self.speed) == 0:
                self.speed=1
            elif self.speed < 0:
                self.speed /= 2
            else:
                self.speed *=2
                if self.speed > self.speedlimit:
                    self.speed=self.speedlimit
            print('rite: was %s, now %s' % (olds, self.speed))
            self.motor.setspeed(speed=self.speed)
        elif pos.middle:
            print("stopped")
            self.motor.setspeed(speed=0)
        
    def stop(self):
        self.motor.close()

if __name__=='__main__':
    motor=bluemotor()
    bd = BlueDot()
    bd.when_pressed = motor.dpad
    try:
        pause()
    except KeyboardInterrupt:
        print('stop')
        motor.stop()