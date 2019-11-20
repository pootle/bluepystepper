#!/usr/bin/env python3
"""
base module for driving unipolar stepper via simple darlington drivers.

For example the el cheapo box of 10 28BYJ-48 steppers with little ULN2003A based driver boards
"""

import pigpio
import threading, queue
import time

import treedict
import appregs

def StartMotor(**kwargs):
    t=SimpleUni(**kwargs)
    mthread=threading.Thread(target=t.run)
    mthread.start()
    return t

# the pins that control the 4 outputs in the proper order!
defaultpins=(17, 23, 22,27)

class appregPosn(appregs.appreg):
    """
    converts position regs to a meaningful value
    """
    def getCurrent(self):
        return self.drivereg.getCurrent()/self['../uStepsPerRev'].getCurrent()

    def setVal(self, value):
        self.drivereg.setValue(round(value*self['../uStepsPerRev'].getCurrent()))

motor28BYJ_48=(
    {'_cclass': appregs.appval, 'name': 'stepsPerRev'   , 'value': 2048/12},    # motor and gearbox with 12:1 speedup to second hand
    {'_cclass': appregs.appval, 'name': 'maxrpm'        , 'value': 100},        # 1 rpm is 1 rotation of the second hand per minute
    {'_cclass': appregs.appval, 'name': 'uSteps'        , 'value': 2},          # microsteps per full step - 2 for now
    {'_cclass': appregs.uStepsPR,'name': 'uStepsPerRev'},                       # calculated from stepsPerRev and uStepsPerRev
    {'_cclass': appregPosn,     'name': 'posn', 'drivereg': '../../driveregs/uStepPos'},
#    {'_cclass': appregPosn,     'name': 'target', 'drivereg':'../../driveregs/XTARGET'}, 
)

mregs=(
    {'_cclass': appregs.appval, 'name': 'uStepPos',         'value': 0},        # records current position in microsteps (integer)
    {'_cclass': appregs.appval, 'name': 'reverse',          'value': True},     # set True to invert motor direction if +ve speed is wrong direction
    {'_cclass': appregs.appval, 'name': 'PWM frequency',    'value': 10000},    # frequency to request for pwm (but see pigpio docs for details) 
    {'_cclass': appregs.appval, 'name': 'actual frequency', 'value': None},     # actual pwm frequency reported by pigpio
    {'_cclass': appregs.appval, 'name': 'hold power',       'value': .1},       # power reduction used when motor is stopped
    {'_cclass': appregs.appval, 'name': 'slow power',       'value': .6},       # power reduction used when motor is at slow speed
    {'_cclass': appregs.appval, 'name': 'slow limit',       'value': 20},       # rpm below which slow power factor used
    {'_cclass': appregs.appval, 'name': 'fast power',       'value': 1},        # power reduction used when speed above slow limit
    
)

class SimpleUni(treedict.Tree_dict):
    """
    simple controller for a direct from pi (well via driver transistors) unipolar stepper.
    
    This class is instantiated by StartMotor above and then runs as a new thread, independent of the
    thread that created it,
    
    State can be read from the original thread, but all 'commands' are passed via a queue to avoid
    concurrent update problems.
    """
    def __init__(self, pins=defaultpins, settings=motor28BYJ_48, pio=None, **kwargs):
        """
        pins:   list (like) of the 4 pins to drive the 4 outputs
        
        settings:dict with setup settings - see below
        
        pio     : an existing instance of pigpio to use, or None, in which case
                  a new pigpio instance is setup and will be closed on exit

        kwargs  : further params passed to Tree_dict constructor
        
        setting
        =======
        
        """
        assert len(pins) == 4
        for i in pins:
            assert isinstance(i, int) and 0<i<32
        if not pio is None:
            ptest=pigpio.pi()
            if not ptest.connected:
                raise ValueError('no pigpio connection available')
            ptest.stop()
        self.pins=pins
        self.pio=pio
        self.running=True
        self.commandqueue=queue.Queue()
        super().__init__(parent=None, app=None, **kwargs)
        self.makeChild(_cclass=treedict.Tree_dict, name='driveregs', childdefs=mregs                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    )
        self.makeChild(_cclass=treedict.Tree_dict, name='settings', childdefs=settings)

    def run(self):
        self.stepinterval=None   # time between (micro) steps - None if motor is stationary
        self.targetspeed=0
        if self.pio is None:
            self.mypio=True
            self.pio=pigpio.pi()
        else:
            self.mypio=False
        if not self.pio.connected:
            raise ValueError('no pigpio connection available')
            return
        for p in self.pins:
            self.pio.set_PWM_frequency(p, round(self['driveregs/PWM frequency'].getCurrent()))
        actual=self.pio.get_PWM_frequency(self.pins[0])
        print('setup pwm on pins %s, requested frequency %d Hz, actual frequency is %d Hz' % (self.pins, round(self['driveregs/PWM frequency'].getCurrent()), actual))
        self['driveregs/actual frequency'].setVal(actual)
        nextsteptime=time.time() + 1 if self.stepinterval is None else self.stepinterval
        ticktable='two'
        stepindex=0
        usetable = self._maketable(ticktable)
        stepfactor=StepTables[ticktable]['factor']
        thisentry=usetable[stepindex]
        self.uStepPosReg=self['driveregs/uStepPos']
        stoptime=None
        while self.running:
            if not stoptime is None and time.time() > stoptime:
                for p in self.pins:
                    self.pio.set_PWM_dutycycle(p,0)
            delay=nextsteptime-time.time()
            if delay > .25:
                time.sleep(.25)
            else:
                if delay > .0001:
                    time.sleep(delay)
                if self.stepinterval is None:
                    nextsteptime += 1
                else:
                    nextsteptime += self.stepinterval
                    stepchange=1 if self.targetspeed > 0 else -1
                    stepindex += stepchange
                    if stepindex >= len(usetable):
                        stepindex=0
                    elif stepindex < 0:
                        stepindex=len(usetable)-1
                    self.uStepPosReg.changeVal(stepchange*stepfactor)
                    lastentry=thisentry
                    thisentry=usetable[stepindex]
                    for i in range(len(lastentry)):
                        if thisentry[i]!=lastentry[i]:
                            self.pio.set_PWM_dutycycle(self.pins[i],thisentry[i])
            try:
                qcommand=self.commandqueue.get_nowait()
            except queue.Empty:
                qcommand=None
            if not qcommand is None:
                command, params=qcommand
                if command=='setspeed':
                    self.targetspeed=params
                    if abs(params) < .00001:
                        self.stepinterval=None
                        usetable = self._maketable(ticktable)
                        print('stopped')
                        stoptime=time.time()+3
                    else:
                        tps=abs(params)*self['settings/uStepsPerRev'].getCurrent()/60   # calculate steps per second and convert to interval in seconds
                        self.stepinterval=1/tps
                        usetable = self._maketable(ticktable)
                        print('step interval now set to %2.5f' % self.stepinterval)
                        stoptime=None
                elif command=='setmicrostep' and params in StepTables and params != ticktable:
                    newstepfactor=StepTables[params]['factor']
                    if newstepfactor > stepfactor:
                        ratio=newstepfactor/stepfactor
                        stepindex = round(stepindex/ratio)
                    elif newstepfactor < stepfactor:
                        ratio=stepfactor/newstepfactor
                        stepindex=round(stepindex*ratio)
                    ticktable=params
                    usetable = self._maketable(ticktable)
                    stepfactor=newstepfactor
                elif command=='setreverse' and params != self['driveregs/reverse'].getCurrent():
                    print(' change direction')
                    self['driveregs/reverse'].setVal(params)
                    usetable = self._maketable(ticktable)
        print('motor thread shutting down')
        for p in self.pins:
            self.pio.set_PWM_dutycycle(p,0)
        if self.mypio:
            self.pio.stop()
            self.mypio=False
        self.pio=None

    def _maketable(self, tablename):
        if self.targetspeed is None or abs(self.targetspeed) < .0001:
            pfact=self['driveregs/hold power'].getCurrent()
        elif abs(self.targetspeed) < self['driveregs/slow limit'].getCurrent():
            pfact=self['driveregs/slow power'].getCurrent()
        else:
            pfact=self['driveregs/fast power'].getCurrent()
        newtable=[[int(oneval*pfact) for oneval in coilvals] for coilvals in StepTables[tablename]['table']]
        if not self.targetspeed is None and self['driveregs/reverse'].getCurrent():
            newtable=list(reversed(newtable))
#        print('new power table:\n  ', '\n   '.join(['%3d, %3d, %3d, %3d' % tuple(cvals) for cvals in newtable]))
        return newtable

    def setspeed(self, speed):
        """
        Sets motor target speed, can be positive or negative.
        
        For now this immediately sets the step interval
        """
        self.commandqueue.put(('setspeed', float(speed)))

    def setreverse(self, isreversed):
        self.commandqueue.put(('setreverse', isreversed == True))
        print('reverse requested')

    def close(self):
        self.running=False

StepTables={
    'single'    : {'factor':4, 'table':((255, 0, 0, 0), (0, 255, 0, 0), (0, 0, 255, 0), (0, 0, 0, 255))},
    'singled'   : {'factor':4, 'table':((255, 255, 0, 0), (0, 255, 255, 0), (0, 0, 255, 255), (255, 0, 0, 255))},
    'two'       : {'factor':2, 'table': ((255, 0, 0, 0), (128,128, 0, 0), (0,255, 0, 0), (0, 128, 128, 0), (0, 0, 255, 0), (0, 0, 128, 128), (0, 0, 0, 255), (128, 0, 0, 128))},
    'four'      : {'factor':1, 'table': ((255, 0, 0, 0),
                                         (192, 64, 0, 0),
                                         (128, 128, 0, 0),
                                         (64, 192,0,0),
                                         (0,255,0,0),
                                         (0,192, 64, 0),
                                         (0, 128, 128, 0),
                                         (0, 64, 192, 0),
                                         (0, 0, 255,0),
                                         (0,0,192,64),
                                         (0,0,128,128),
                                         (0,0,64,192),
                                         (0,0,0,255),
                                         (64, 0, 0, 192),
                                         (128,0,0,128),
                                         (192, 0, 0, 64),
                   )},
}
print('tables are:\n', '\n'.join(['%12s: %d' % (name, len(tx['table'])) for name, tx in StepTables.items()]))