#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
import os
import time
import math
import datetime
from optparse import OptionParser

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'MAVProxy/MAVProxy'))
from mavproxy import *
import mavproxy


DELIMITER_GROUP = str(unichr(29))
DELIMITER_RECORD = str(unichr(30))
#PROCENTZONE = 4 # If feedback is below this procentage, do nothing
MIN_SURENESS = 0.4 #Don't use values if they are not more than 40 % sure
#goalAltitude=-1
#sweetSpot=0.5
# Altitude adjustment is moved to MAVProxy module
DESIRED_DISTANCE_Z = 200
SWEETSPOT_Y = 0.5

class Regulator(object):
    """Reads input from stream (standard implementation is meant for stdin)
    and converts relative positions into a command sent to MAVProxy
    for quadcopter control"""
    def __init__(self, stream=sys.stdin, kp=0.2, ki=0):
        super(Regulator, self).__init__()
        self.stream = stream
        self.kp = kp
        self.ki = ki

        # X, Y, Z positions in 3D space
        self.lastPosition = [None, None, None]

        #Maximum allowed speed of the Quadcopter
        self.boundary = 10

    def toFloats(self, array):
        for index, s in enumerate(array):
            array[index] = float(s)

        return array

    def parse(self, line):
        groups = line.split(DELIMITER_GROUP)

        # Scream for help, lol
        if (len(groups) < 1):
            raise Exception('Invalid input sent to regulator, no groups found')

        # Read 3D position
        position = self.toFloats(groups.pop(0).split(DELIMITER_RECORD))

        # Read angle if it exists
        if (len(groups) > 0):
            angle = self.toFloats(groups.pop(0).split(DELIMITER_RECORD))
        else:
            # Set default value null
            angle = None
            
        # Read sureness
        if (len(groups) > 0):
            sureness = float(groups.pop(0))
            #self.toFloats(groups.pop(0).split(DELIMITER_RECORD))
        else:
            # Set default value null
            sureness = None
            raise Exception('WARNING: Couldnt parse sureness, is now set to None')
        
        #print(sureness)
        # Scream for help, someone is drowning us in input
        if (len(groups) > 0):
            raise Exception('Invalid input sent to regulator, \
                            too many groups found')

        return (position, angle, sureness)

    def regulateDistance(self, distance, prevDistance, prevOutput, ref, deltaT):
        error = distance - ref
        prevError = prevDistance - ref
        output = self.kp * error + self.ki * deltaT * (error + prevError)

        if (abs(output) > self.boundary):
            return math.copysign(self.boundary, output)
        else:
            return output
            
    def regulateHeight(self, distance, ref):
        output = 0
        if(distance >= ref):
            output = -50
        elif(distance <= -ref) :
            output = 50
        
        return output

    
def median(list):
    list = sorted(list)
    if len(list)%2 == 0:
        b = len(list)/2
        a = b - 1
        return (list[a] + list[b])/2
    return list[len(list)/2]

def getData(positionArray, anglesArray, timesArray):
    position = None
    angle = None
    deltaT = None
    if(positionArray):
        posXs = []
        posYs = []
        posZs = []

        for i in positionArray:
            posXs += [i[0]]
            posYs += [i[1]]
            posZs += [i[2]]
            
        position = [median(posXs), median(posYs), median(posZs)]
            
    if(anglesArray):
        angX = []
        angY = []
        angZ = []

        for i in anglesArray:
            angX += [i[0]]
            angY += [i[1]]
            angZ += [i[2]]
            
        angle = [median(angX), median(angY), median(angZ)]

    if(timesArray):
        deltaT = median(timesArray)
    
    return (position, angle, deltaT)
    
def init():
    # While true read stdin
    regulator = Regulator()
    
    oldPosition = None
    oldRegulatedX = 0
    oldRegulatedZ = 0
    oldRegulatedY = 0
    
    positions = []
    angles = []
    times = []
    
    startTime = time.time()
    
    # COMPUTER CONTROL STARTS HERE, microcontroller no longer has any effect on the channels which are in use
    
    time.sleep(7) # Wait for MAVProxy to parse 'alt' correctly from the APM

    # SET ALTITUDE HERE.
    mavproxy.mpstate.functions.process_stdin("startalt") # Initialize altitude-hold method in MAVProxy module
    time.sleep(1) # Wait for it to get ready
    mavproxy.mpstate.functions.process_stdin("setalt 2") # Set altitude to 2 meters

    mavproxy.mpstate.functions.process_stdin("hover") # Initialize by setting all sticks to middle and enter alt_hold mode
    #time.sleep(1) 

    while True:
        try:
            for i in range(0, 5):
                (position, angle, sureness) = regulator.parse(regulator.stream.readline())
            
                newTime = time.time()
                deltaT = startTime - newTime
                #sureness=100 
                if sureness >= MIN_SURENESS:
                    positions += [position]
                    angles += [angle]
                    times += [deltaT]
                
        
            #If we have any values on "oldPosition" and we have
            #  at least three positions
            if (oldPosition and len(positions) >= 3):
            
                (position, angle, deltaT) = getData(positions, angles, times)
                ts=time.time() 
                x = position[0]
                y = position[1]
                z = position[2]
                lastX = oldPosition[0]
                lastY = oldPosition[1]
                lastZ = oldPosition[2]
                deltaT = startTime - newTime
                regulatedX = regulator.regulateDistance(x, lastX,
                                                        oldRegulatedX, 0, deltaT)
                regulatedY = regulator.regulateHeight(y, SWEETSPOT_Y)
                #Borde vara en konstant istället, alternativt en parameter in ttill programmet
                regulatedZ = regulator.regulateDistance(z, lastZ,
                                                        oldRegulatedZ, DESIRED_DISTANCE_Z, deltaT)
                st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
                print('[{4}]   X: {0}, Y: {1}, Z: {2}, S: {3}'.format(regulatedX, regulatedY, regulatedZ, sureness, st))
           

                #Send data for X to the copter
                mavproxy.mpstate.functions.process_stdin("strafe %d" % regulatedX)
                #Send data for Y to the copter
                #mavproxy.mpstate.functions.process_stdin("movey %d" % regulatedY)
                #Send data for Z to the copter
                mavproxy.mpstate.functions.process_stdin("movez %d" % regulatedZ) 
    

                oldRegulatedX = regulatedX
                oldRegulatedZ = regulatedZ
                oldRegulatedY = regulatedY
            else: 
                (position, angle, deltaT) = getData(positions, angles, times)
                mavproxy.mpstate.functions.process_stdin("strafe 0")
                #mavproxy.mpstate.functions.process_stdin("movey 0")
                mavproxy.mpstate.functions.process_stdin("movez 0")
    
            #Clear any old data
            positions = []
            angles = []
            times = []
            
            oldPosition = position
            startTime = newTime
        except KeyboardInterrupt:
            # This happends when Ctrl-C is pressed
            mavproxy.mpstate.functions.process_stdin("setalt -1") # Set altitude to -1 to disable altitude control            
            print("Giving control back to microcontroller...")
            mavproxy.mpstate.functions.process_stdin("rc all 0") # Give back all control to controller
            mavproxy.mpstate.functions.process_stdin("mode stabilize") # Set mode to stabilize 
            time.sleep(0.6)
            mavproxy.mpstate.functions.process_stdin("rc all 0")
            mavproxy.mpstate.functions.process_stdin("mode stabilize")            
            print("Exiting...")


def initMAVProxy():
    from optparse import OptionParser
    parser = OptionParser("mavproxy.py [options]")

    parser.add_option("--master", dest="master", action='append',
                      metavar="DEVICE[,BAUD]", help="MAVLink master port and optional baud rate",
                      default=[])
    parser.add_option("--out", dest="output", action='append',
                      metavar="DEVICE[,BAUD]", help="MAVLink output port and optional baud rate",
                      default=[])
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="default serial baud rate", default=115200)
    parser.add_option("--sitl", dest="sitl",  default=None, help="SITL output port")
    parser.add_option("--streamrate",dest="streamrate", default=4, type='int',
                      help="MAVLink stream rate")
    parser.add_option("--source-system", dest='SOURCE_SYSTEM', type='int',
                      default=255, help='MAVLink source system for this GCS')
    parser.add_option("--target-system", dest='TARGET_SYSTEM', type='int',
                      default=1, help='MAVLink target master system')
    parser.add_option("--target-component", dest='TARGET_COMPONENT', type='int',
                      default=1, help='MAVLink target master component')
    parser.add_option("--logfile", dest="logfile", help="MAVLink master logfile",
                      default='mav.tlog')
    parser.add_option("-a", "--append-log", dest="append_log", help="Append to log files",
                      action='store_true', default=False)
    parser.add_option("--quadcopter", dest="quadcopter", help="use quadcopter controls",
                      action='store_true', default=False)
    parser.add_option("--setup", dest="setup", help="start in setup mode",
                      action='store_true', default=False)
    parser.add_option("--nodtr", dest="nodtr", help="disable DTR drop on close",
                      action='store_true', default=False)
    parser.add_option("--show-errors", dest="show_errors", help="show MAVLink error packets",
                      action='store_true', default=False)
    parser.add_option("--speech", dest="speech", help="use text to speach",
                      action='store_true', default=False)
    parser.add_option("--num-cells", dest="num_cells", help="number of LiPo battery cells",
                      type='int', default=0)
    parser.add_option("--aircraft", dest="aircraft", help="aircraft name", default=None)
    parser.add_option("--cmd", dest="cmd", help="initial commands", default=None)
    parser.add_option("--console", action='store_true', help="use GUI console")
    parser.add_option("--map", action='store_true', help="load map module")
    parser.add_option(
        '--load-module',
        action='append',
        default=[],
        help='Load the specified module. Can be used multiple times, or with a comma separated list')
    parser.add_option("--mav09", action='store_true', default=False, help="Use MAVLink protocol 0.9")
    parser.add_option("--auto-protocol", action='store_true', default=False, help="Auto detect MAVLink protocol version")
    parser.add_option("--nowait", action='store_true', default=False, help="don't wait for HEARTBEAT on startup")
    parser.add_option("--continue", dest='continue_mode', action='store_true', default=False, help="continue logs")
    parser.add_option("--dialect",  default="ardupilotmega", help="MAVLink dialect")
    parser.add_option("--rtscts",  action='store_true', help="enable hardware RTS/CTS flow control")

    (opts, args) = parser.parse_args()
    mavproxy.opts=opts

    if opts.mav09:
        os.environ['MAVLINK09'] = '1'
    from pymavlink import mavutil, mavparm
    mavutil.set_dialect(opts.dialect)

    # global mavproxy state
    mpstate = MPState()
    mavproxy.mpstate=mpstate
    mpstate.status.exit = False
    mpstate.command_map = command_map
    mpstate.continue_mode = opts.continue_mode

    if opts.speech:
        # start the speech-dispatcher early, so it doesn't inherit any ports from
        # modules/mavutil
        load_module('speech')

    if not opts.master:
        serial_list = mavutil.auto_detect_serial(preferred_list=['*FTDI*',"*Arduino_Mega_2560*", "*3D_Robotics*", "*USB_to_UART*"])
        if len(serial_list) == 1:
            opts.master = [serial_list[0].device]
        else:
            print('''
Please choose a MAVLink master with --master
For example:
    --master=com14
    --master=/dev/ttyUSB0
    --master=127.0.0.1:14550

Auto-detected serial ports are:
''')
            for port in serial_list:
                print("%s" % port)
            sys.exit(1)

    # container for status information
    mpstate.status.target_system = opts.TARGET_SYSTEM
    mpstate.status.target_component = opts.TARGET_COMPONENT

    mpstate.mav_master = []

    # open master link
    for mdev in opts.master:
        if ',' in mdev and not os.path.exists(mdev):
            port, baud = mdev.split(',')
        else:
            port, baud = mdev, opts.baudrate

        m = mavutil.mavlink_connection(port, autoreconnect=True, baud=int(baud))
        m.mav.set_callback(master_callback, m)
        if hasattr(m.mav, 'set_send_callback'):
            m.mav.set_send_callback(master_send_callback, m)
        if opts.rtscts:
            m.set_rtscts(True)
        m.linknum = len(mpstate.mav_master)
        m.linkerror = False
        m.link_delayed = False
        m.last_heartbeat = 0
        m.last_message = 0
        m.highest_msec = 0
        mpstate.mav_master.append(m)
        mpstate.status.counters['MasterIn'].append(0)

    # log all packets from the master, for later replay
    open_logs()

    # open any mavlink UDP ports
    for p in opts.output:
        if ',' in p and not os.path.exists(p):
            port, baud = p.split(',')            
        else:
            port, baud = p, opts.baudrate

        mpstate.mav_outputs.append(mavutil.mavlink_connection(port, baud=int(baud), input=False))

    if opts.sitl:
        mpstate.sitl_output = mavutil.mavudp(opts.sitl, input=False)

    mpstate.settings.numcells = opts.num_cells
    mpstate.settings.streamrate = opts.streamrate
    mpstate.settings.streamrate2 = opts.streamrate

    mavproxy.msg_period = mavutil.periodic_event(1.0/15)
    mavproxy.heartbeat_period = mavutil.periodic_event(1)
    mavproxy.battery_period = mavutil.periodic_event(0.1)
    mavproxy.heartbeat_check_period = mavutil.periodic_event(0.33)

    mpstate.rl = rline.rline("MAV> ", mpstate)
    if opts.setup:
        mpstate.rl.set_prompt("")

    if 'HOME' in os.environ and not opts.setup:
        start_script = os.path.join(os.environ['HOME'], ".mavinit.scr")
        if os.path.exists(start_script):
            run_script(start_script)

    if opts.aircraft is not None:
        start_script = os.path.join(opts.aircraft, "mavinit.scr")
        if os.path.exists(start_script):
            run_script(start_script)
        else:
            print("no script %s" % start_script)

    if not opts.setup:
        # some core functionality is in modules
        standard_modules = ['log','rally','fence','param','relay',
                            'tuneopt','arm','mode','calibration','rc','wp','auxopt','quadcontrols','test']
        for m in standard_modules:
            load_module(m, quiet=True)

    if opts.console:
        process_stdin('module load console')

    if opts.map:
        process_stdin('module load map')

    for module in opts.load_module:
        modlist = module.split(',')
        for mod in modlist:
            process_stdin('module load %s' % mod)

    if opts.cmd is not None:
        cmds = opts.cmd.split(';')
        for c in cmds:
            process_stdin(c)

# run main loop as a thread
    mpstate.status.thread = threading.Thread(target=main_loop)
    mpstate.status.thread.daemon = True
    mpstate.status.thread.start()


if __name__ == '__main__':
    initMAVProxy()
    init()
