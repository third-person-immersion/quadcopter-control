#!/usr/bin/python
import sys
import os
import time
import math
from optparse import OptionParser

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'MAVProxy/MAVProxy'))
from mavproxy import *
import mavproxy


DELIMITER_GROUP = str(unichr(29))
DELIMITER_RECORD = str(unichr(30))


class Regulator(object):
    """Reads input from stream (standard implementation is meant for stdin)
    and converts relative positions into a command sent to MAVProxy
    for quadcopter control"""
    def __init__(self, stream=sys.stdin, kp=0.2, ki=0.2):
        super(Regulator, self).__init__()
        self.stream = stream
        self.kp = kp
        self.ki = ki

        # X, Y, Z positions in 3D space
        self.lastPosition = [None, None, None]

        self.boundary = 100

    def toFloats(self, array):
        for index, s in enumerate(array):
            array[index] = float(s)

        return array

    def parse(self, line):
        groups = line.split(DELIMITER_GROUP)

        # Scream for help
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

        # Scream for help, someone is drowning us in input
        if (len(groups) > 0):
            raise Exception('Invalid input sent to regulator, \
                            too many groups found')

        return (position, angle)

    def regulateDistance(self, distance, prevDistance, prevOutput, ref, deltaT):
        error = distance - ref
        prevError = prevDistance - ref
        output = self.kp * error + self.ki * deltaT * (error + prevError)

        if (abs(output) > self.boundary):
            return math.copysign(self.boundary, output)
        else:
            return output


def init():
    # While true read stdin
    regulator = Regulator()

    oldPosition = None
    oldRegulatedX = 0
    oldRegulatedZ = 0
    startTime = time.time()

    while True:
        (position, angle) = regulator.parse(regulator.stream.readline())

        newTime = time.time()
        deltaT = startTime - newTime
        if (position and oldPosition):
            x = position[0]
            # y = position[1]
            z = position[2]
            lastX = oldPosition[0]
            # lastY = oldPosition[1]
            lastZ = oldPosition[2]
            deltaT = startTime - newTime
            regulatedX = regulator.regulateDistance(x, lastX,
                                                    oldRegulatedX, 10, deltaT)
            regulatedZ = regulator.regulateDistance(z, lastZ,
                                                    oldRegulatedZ, 100, deltaT)
            oldRegulatedX = regulatedX
            oldRegulatedZ = regulatedZ
            print('X: {0}, Z: {1}'.format(regulatedX, regulatedZ))
            # mavproxy.cmd_strafe([50])
            # THIS IS WHERE WE CONTROL SHIT

        oldPosition = position
        startTime = newTime


def initMAVProxy():
    parser = OptionParser("mavproxy.py [options]")

    parser.add_option("--master",dest="master", action='append', help="MAVLink master port", default=[])
    parser.add_option("--baudrate", dest="baudrate", type='int',
                      help="master port baud rate", default=115200)
    parser.add_option("--out",   dest="output", help="MAVLink output port",
                      action='append', default=[])
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
    mavproxy.opts = opts

    if opts.mav09:
        os.environ['MAVLINK09'] = '1'
    from pymavlink import mavutil, mavwp, mavparm
    mavutil.set_dialect(opts.dialect)

    # global mavproxy state
    mpstate = MPState()
    mavproxy.mpstate = mpstate
    mpstate.status.exit = False
    mpstate.command_map = command_map
    mpstate.continue_mode = opts.continue_mode

    if opts.speech:
        # start the speech-dispatcher early, so it doesn't inherit any ports from
        # modules/mavutil
        say('Startup')

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
        m = mavutil.mavlink_connection(mdev, autoreconnect=True, baud=opts.baudrate)
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

    if mpstate.continue_mode and mpstate.status.logdir != None:
        parmfile = os.path.join(mpstate.status.logdir, 'mav.parm')
        if os.path.exists(parmfile):
            mpstate.mav_param.load(parmfile)
            for m in mpstate.mav_master:
                m.param_fetch_complete = True
        waytxt = os.path.join(mpstate.status.logdir, 'way.txt')
        if os.path.exists(waytxt):
            mpstate.status.wploader.load(waytxt)
            print("Loaded waypoints from %s" % waytxt)

    # open any mavlink UDP ports
    for p in opts.output:
        mpstate.mav_outputs.append(mavutil.mavlink_connection(p, baud=opts.baudrate, input=False))

    if opts.sitl:
        mpstate.sitl_output = mavutil.mavudp(opts.sitl, input=False)

    mpstate.settings.numcells = opts.num_cells
    mpstate.settings.speech = opts.speech
    mpstate.settings.streamrate = opts.streamrate
    mpstate.settings.streamrate2 = opts.streamrate

    msg_period = mavutil.periodic_event(1.0/15)
    param_period = mavutil.periodic_event(1)
    log_period = mavutil.periodic_event(2)
    heartbeat_period = mavutil.periodic_event(1)
    battery_period = mavutil.periodic_event(0.1)
    if mpstate.sitl_output:
        mpstate.override_period = mavutil.periodic_event(20)
    else:
        mpstate.override_period = mavutil.periodic_event(1)
    heartbeat_check_period = mavutil.periodic_event(0.33)

    mpstate.rl = rline.rline("MAV> ")
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
        standard_modules = ['log','rally','fence']
        for m in standard_modules:
            process_stdin('module load %s' % m)

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
    # initMAVProxy()
    init()
