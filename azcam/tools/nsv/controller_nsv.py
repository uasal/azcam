"""
Contains the ControllerNSV class
Parts taken from example in azcam for controller_archon.py which was originally written by Grzegorz Zareba
DRAFTING / NOT DONE / MAINLY PIECING STRUCTURE TOGETHER- SFR
Contains notes to self / ignore
"""

import socket
import time
import threading
# Note- do we also want to use pyrav412 like in camera.py with the sockets or just have it similar to azcam? - SFR
import azcam
import azcam.utils
import azcam.exceptions
import azcam.sockets
from azcam.tools.controller import Controller

# ------- power management (adapted from NSV code) --------
# From camera.py for reference

cmd = {'on':0x90, 'off':0x80}
bus = {'1':0x08, '2': 0x08,
        '3':0x02, '4':0x02,
        '5':0x01, '6':0x01}
addr = {'1':0x02bf, '2':0x02b0,
        '3':0x02bf, '4':0x02b0,
        '5':0x02bf, '6':0x02b0}

# Mainly based / pulled from archon example which uses sockets.
class ControllerNSV(Controller):
    """
    The controller class for cameras using NSV.
    """

    # Exposure states
    EXP_UNKNOWN = 0
    EXP_IDLE = 1
    EXP_EXPOSE = 2
    EXP_READY = 3
    EXP_FETCH = 4
    EXP_DONE = 5
    DEFAULT_MODE = 'ROI'


    def __init__(self, tool_id="controller", description=None):
        super().__init__(tool_id, description)
        self.controller_class = "NSV"
        self.controller_type = "NSV"

        # video gain flag
        self.video_gain = 1

        # True if image data from ControllerServer is binary (and not a file)
        self.binary_image_data = 1

        self.shutter_stat = 0

        # Pulled from camera.py
        self.gain_ctrl = None
        self.exp_ctrl = None
        self.fps_ctrl = None
        self.blacklevel_ctrl = None
        self.cam = None
        self.width = None
        self.height = None
        self.gain = None
        self.exptime = None
        self.blacklevel = None
        self.vcropoffset = None
        self.is_connected = False
        self.dpath = dpath

        # power on
        self.power_on()
        # set mode
        self.set_mode(mode)
        # do not initialize this controller type <Potentially will remove this- SFR>
        azcam.db.tools_init.pop("controller", None)
        # azcam connected to the controller
        self.connected_controller = 0
        # reset flag
        self.reset_flag = 0  # 0 for soft reset, 1 to upload code
        # receiving raw data
        self.rawdata_enable = 0
        # Raw channel - in the config dictionary raw channel numbers start from 0
        self.rawdata_channel = 1
        # Raw data received from the nsv controller
        self.rawdata = 0

        #: lock for threads
        self.lock = threading.Lock()

        # controller server <Need to make adjustments potentially- SFR>
        self.camserver = azcam.sockets.SocketInterface()
        self.camserver.host = ""
        self.camserver.port = 4242
        self.camserver.cmd_id = 0x00
        self.camserver.lastcmd_id = 0x00

    # Keeping similar as controller_archon.py for now. SFR
    def initialize(self):
        """
        Initializes the NSV controller.
        """

        if self.is_initialized:
            return

        # connect to controller
        self.connect()

        # load configuration file
        self.update_config_data(self.reset_flag)

        # set pars to stop exposure cycle and then load them
        self.set_continuous_exposures(0)
        self.set_exposures(0)
        self.set_int_ms(0)
        self.set_no_int_ms(0)
        self.load_params()
        self.resettiming()

        # check power status
        if self.verbosity > 1:
            azcam.log("Setting controller power status")
        self.get_power_status()

        if self.power_status in ["OFF", "NOT_CONFIGURED"]:
            azcam.log("Power status: ", self.power_status, level=2)
            self.power_on(1)
        elif self.power_status != "ON":
            raise azcam.exceptions.AzcamError(f"Bad power status: {self.power_status}")

        self.is_initialized = 1

        return

    # Keeping same as controller_archon.py
    def connect(self):
        """
        Connects NSV to the controller.
        """

        if self.camserver.open():
            self.connected_controller = 1
            self.camserver.set_timeout(5)
        else:
            self.connected_controller = 0

        return

    # Keeping same as controller_archon.py
    def disconnect(self):
        """
        Disconnects NSV from the controller.
        """

        self.camserver.close()
        self.connected_controller = 0

        return

    # Keeping same as controller_archon.py
    def nsv_command(self, Command):
        """
        Send a command to the NSV controller.
        """

        with self.lock:
            if not self.camserver.open():
                raise azcam.exceptions.AzcamError(
                    "Could not open connection to controller."
                )

            self.camserver.lastcmd_id = self.camserver.cmd_id
            self.camserver.cmd_id = (self.camserver.cmd_id + 1) & 0xFF
            preCmd = ">%02X" % (self.camserver.cmd_id)
            preResp = "<%02X" % (self.camserver.cmd_id)
            cmd = preCmd + Command
            if self.verbosity > 2:
                print("===>", cmd)

            self.camserver.send(cmd, "\r\n")

            if Command not in ["WARMBOOT", "REBOOT"]:
                reply = self.camserver.recv(-1)
                if self.verbosity > 2:
                    print("<===", reply[:40])
                status = reply.split(" ")[0]

                # check if the reply is synchronized
                if status[0:3] == preResp:
                    return reply[3:]
                else:
                    if reply[0] == "?":
                        raise azcam.exceptions.AzcamError("NSV response not valid")
                    else:
                        raise azcam.exceptions.AzcamError("NSV response out of sync")

        return None  # no nsv response is OK

    def power_on(self):
        """
        :return:
        """

        # Checks to see if already on
        status = self.get_power_status()
        if status == "ON":
            return
        cmd = "POWERON"

        self.nsw_command(cmd)

        # Checks power status and waits up to 10 seconds
        cnt = 0
        power = 0
        while cnt < 10:
            if self.get_power_status =="ON":
                # Exits
                cnt = 10
                power = 1
                return
            else:
                # Waits
                cnt += 1
                time.sleep(1)
        azcam.exceptions.warning("Controller power is OFF")
        return

    def power_off(self):
        """
        Turns off power
        :return:
        """
        cmd = "POWEROFF"
        self.nsv_command(cmd)
        return

    def set_exposuretime(self, time):
        """
        Sets the exposure time (seconds) to the controller.
        :param time: exposure time (secs) value
        :return:
        """
        return


