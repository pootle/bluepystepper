#!/usr/bin/env python3

"""
application registers are managed variables used in the user interface to control settings or report state
"""
import treedict

class appreg(treedict.Tree_dict):
    """
    This class (and those inheriting from it) are used to present values to the users that are indirectly used by the motor driver.
    
    These can be used where the underlying driver value is always in direct correspondence to the value here, so the chip
    register IS the reference value - the value in this class is always computed from the chip register value.
    
    """
    def __init__(self, drivereg, logacts = ('constructors', 'content'), **kwargs):
        """
        drivereg:  path to the chip register this value is based on
        """
        super().__init__(**kwargs)
        self.drivereg=self[drivereg]

class appval(treedict.Tree_dict):
    """
    a base class for settings / states that need to be processed to do things to the chip. 
    
    e.g. a value in rpm is converted to / from a chip register value using info about the clock frequency and the motor

    This simplest case just records a value
    """
    def __init__(self, value=None, **kwargs):
        super().__init__(**kwargs)
        self.setVal(value)

    def getCurrent(self):
        return self.curval

    def setVal(self, value):
        self.curval=value

    def changeVal(self, change):
        self.curval += change

class uStepsPR(appval):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.setVal(self['../stepsPerRev'].getCurrent()*self['../uSteps'].getCurrent())
