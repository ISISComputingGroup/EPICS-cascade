import threading
import numpy
import time
import random
import os
import sys
import numpy
from pcaspy import Driver, SimpleServer, Severity

## set TESTING = False if PyTango is installed and PyTango.DeviceProxy below is correct
## PyTango is available from pip on Linux, but should already be installed on Cascade computer

TESTING = True

if TESTING:
    DEVSTATE_ON = 0
    DEVSTATE_MOVING = 1
    DEVSTATE_FAULT = 2
else:
    import PyTango
    DEVSTATE_ON = PyTango.DevState.ON
    DEVSTATE_MOVING = PyTango.DevState.MOVING
    DEVSTATE_FAULT = PyTango.DevState.FAULT

## process variables will be e.g. "IN:LARMOR:CASCADE:STATE" 
## write to :SP values, readbacks are values without :SP
## FILENAME and FILENAME:SP are character arrays rather than strings (to allow > 40 characters), so use caput -S and caget -S from epics command line
## SAVEDATA:SP is an aynchronous PV, if you wish to wait for file write to complete request a 
##             completion callback e.g.    caput -c -w 100    (the -w is max seconds to wait)
## on an IBEX developer PC you will need to set   EPICS_CAS_INTF_ADDR_LIST=127.0.0.1    but not on cascasde machine
## more exception handlong needed? Not sure what Tango returns/throws
## 

## determine PV global prefix
if os.environ.get('MYPVPREFIX') is not None:
    prefix = os.environ['MYPVPREFIX'] + 'CASCADE:'
else:
    prefix = 'IN:LARMOR:CASCADE:'

## EPICS pvs we will serve, above prefix will be added to these
pvdb = {
        'STATE' : { 'type' : 'enum', 'enums': ['Idle', 'Counting', 'Fault']},
        'STATESTR' : { 'type': 'string', 'value' : '' },
        'FILENAME:SP' : { 'type': 'char', 'count': 512, 'value': [0] },
        'FILENAME' : { 'type': 'char', 'count': 512, 'value': [0] },
        'BINX:SP' : { 'type' : 'int', 'value' : 0 },
        'BINX' : { 'type' : 'int', 'value' : 0 },
        'BINY:SP' : { 'type' : 'int', 'value' : 0 },
        'BINY' : { 'type' : 'int', 'value' : 0 },
        'TIMECHANS:SP' : { 'type' : 'int', 'value' : 0 },
        'TIMECHANS' : { 'type' : 'int', 'value' : 0 },
        'PRESELECTION:SP' : { 'type' : 'float', 'value' : 0 },
        'PRESELECTION' : { 'type' : 'float', 'value' : 0, 'prec' : 1 },
        'SAVEDATA:SP' : { 'type' : 'int', 'value' : 0, 'asyn' : True }, # completes asynchronously with optional callback when done
        'CLEAR:SP' : { 'type' : 'int', 'value' : 0 },
        'START:SP' : { 'type' : 'int', 'value' : 0 },
        'STOP:SP' : { 'type' : 'int', 'value' : 0 }
}

## DummyProxy used when TESTING == True
class DummyProxy:
    def __init__(self):
        self.binning = [1, 1]
        self.timeChannels = 128
        self.preselection = 10
        self.value = numpy.array([ 1, 2, 3])
        self.states = ['ON','MOVING','FAULT']
        
    def Start(self):
        print "Start"

    def Stop(self):
        print "Stop"

    def Clear(self):
        print "Clear"
 
    def state(self):
        return random.randint(0,2)

    def status(self):
        return self.states[random.randint(0,2)]

class myDriver(Driver):
    def __init__(self):
        Driver.__init__(self)
        self.saveData = False
        self.filename = ''

        if TESTING:
            self.proxy = DummyProxy()
        else:
            # The server will be running without a Tango database, so the connection is made using
            self.proxy = PyTango.DeviceProxy('tango://localhost:12345/mira/cascade/tofchannel#dbase=no')
            # The "mira/cascade/tofchannel" part depends on what is configured in the Entangle resource file.
        
        self.tid = threading.Thread(target = self.doWork)
        self.tid.setDaemon(True)
        self.tid.start()

    def write(self, reason, value):
        status = True
        if reason == 'BINX:SP':
            old_binning = self.proxy.binning
            self.proxy.binning = [ value, old_binning[1] ]
        elif reason == 'BINY:SP':
            old_binning = self.proxy.binning
            self.proxy.binning = [ old_binning[0], value ]
        elif reason == 'START:SP' and value == 1:
            self.proxy.Start()
        elif reason == 'STOP:SP' and value == 1:
            self.proxy.Stop()
        elif reason == 'CLEAR:SP' and value == 1:
            self.proxy.Clear()
        elif reason == 'PRESELECTION:SP':
            self.proxy.preselection = value
        elif reason == 'TIMECHANS:SP':
            self.proxy.timeChannels = value
        elif reason == 'FILENAME:SP':
            self.filename = value
        elif reason == 'SAVEDATA:SP' and value == 1:
            self.saveData = True
        if status:
            self.setParam(reason, value)
        return status

    def doWork(self):
        while True:
            tango_state = self.proxy.state()
            epics_state = 2
            if tango_state == DEVSTATE_ON: # PyTango.DevState.ON
                epics_state = 0 # Idle
            elif tango_state == DEVSTATE_MOVING: # PyTango.DevState.MOVING
                epics_state = 1 # Counting
            elif tango_state == DEVSTATE_FAULT: # PyTango.DevState.FAULT
                epics_state = 2 # Fault
            self.setParam('STATE', epics_state)
            self.setParam('STATESTR', self.proxy.status())
            self.setParam('PRESELECTION', self.proxy.preselection)
            self.setParam('BINX', self.proxy.binning[0])
            self.setParam('BINY', self.proxy.binning[1])
            self.setParam('TIMECHANS', self.proxy.timeChannels)
            self.setParam('FILENAME', self.filename)
            if self.saveData:
                print "Saving data to {}".format(self.filename)
                try:  
                    self.proxy.value.astype('<u4').tofile(self.filename)
                except:
                    print("Unexpected error saving to file: {}".format(sys.exc_info()[0]))
                print "Save complete"
                # as a save may take a while, we have set the pv as asyn above so need to 
                # signal done to allow completion callbacks to be used
                self.callbackPV('SAVEDATA:SP') 
                self.saveData = False
            # trigger EPICS monitors for changed values
            self.updatePVs() 
            time.sleep(0.1)

if __name__ == '__main__':
    server = SimpleServer()
    server.createPV(prefix, pvdb)
    driver = myDriver()

    # process CA transactions
    while True:
        server.process(0.1)
