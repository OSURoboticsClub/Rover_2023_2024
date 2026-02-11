from typing import List
from copy import deepcopy
import datetime

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from comms_control.RemoteWirelessControl import WirelessInterface
from comms_control.RemoteWirelessControl import InterfaceStatus

from rover2_control_interface.msg import GPSStatusMessage
from rover2_control_interface.msg import CommsStatusMessage
from std_msgs.msg import Float32

from os import getcwd

MONITOR_NODE_PREFIX = 'comms_monitor_'

class WirelessInterfaceMonitor(Node):

    lat:     float = 0.0
    long:    float = 0.0
    heading: float = 0.0
    interface: WirelessInterface
    logDirectory = ""
    logFilename = ""
    name = ""
    status = InterfaceStatus(connected=False, syncing=False)

    def __init__(self, name: str, interface: WirelessInterface, logging: bool = False, logPeriod: float = 5.0, updatePeriod = 5.0, logDirectory: str = ""): 
        self.logDirectory = logDirectory
        self.name = name
        self.interface = interface

        super().__init__(MONITOR_NODE_PREFIX + self.name)
        
        if (logging): #Create gps and imu subscriptions and logging timer if logging is enabled #TODO dynamically start and stop logging (ROS Service?)
            self.gpsSubscription = self.create_subscription(
                GPSStatusMessage,
                'tower/status/gps',
                self.gpsCallback,
                10
            )
        
            self.imuSubscription = self.create_subscription(
                Float32,
                'imu/heading',
                self.imuCallback,
                10
            )
        
            self.logTimer = self.create_timer(
                logPeriod,
                self.logCallback
            )
        
        self.statusPublisher = self.create_publisher(
            CommsStatusMessage,
            f'rover2/status/comms/{self.name}',
            10
        )

        self.statusPublisherTimer = self.create_timer(
            updatePeriod,
            self.statusPublisherCallback
        )

    def statusPublisherCallback(self):
        def noneToZero(x: object):
            if x is None:
                return 0
            else:
                return x
        
        def noneToFloatZero(x: object):
            if x is None:
                return 0.0
            else:
                return x
            
        self.status = self.interface.getStatus()
            
        msg = CommsStatusMessage()
        msg.signal_level = noneToZero(self.status.signalLevel)
        msg.noise_level = noneToZero(self.status.noiseLevel)
        msg.frequency = noneToZero(self.status.frequency)
        msg.channel = noneToZero(self.status.channel)
        msg.rxmcs = str(self.status.rxmcs)
        msg.txmcs = str(self.status.txbitrate)
        msg.rxbitrate = noneToFloatZero(self.status.rxbitrate)
        msg.txbitrate = noneToFloatZero(self.status.txbitrate)
        msg.txpower = noneToFloatZero(self.status.txpower)
        msg.channel_width = noneToZero(self.status.channelWidth)
        msg.connected_to_target = self.status.connected and self.status.syncing
        self.statusPublisher.publish(
            msg
        )

        if self.status.connected == False:
            self.get_logger().warning(f"Failed to connect to target {self.name} at {self.interface.remoteAddr}")
        elif self.status.syncing == False:
            self.get_logger().warning(f"Failed to run commands on target {self.name} at {self.interface.remoteAddr}")
        self.get_logger().info(f"Published status for node {self.name}.", )
        
    def gpsCallback(self, msg: GPSStatusMessage):
        if (type(msg.rover_latitude) is float) and (type(msg.rover_longitude) is float):
            self.lat = msg.rover_latitude
            self.long = msg.rover_longitude
        else:
            self.lat = 0
            self.long = 0
    
    def imuCallback(self, msg: Float32):
        if (type(msg.data) is float):
            self.heading = msg.data
        else:
            self.heading = 0

    def logCallback(self):
        currentStatus = self.status
        if self.logFilename == "": # if the log file-name is empty, begin logging by generating a new file name
            self.logFilename = f"{self.name}_{datetime.datetime.now().replace(microsecond=0).isoformat().replace(':','-')}.csv"
            try:
                with open(self.logDirectory+self.logFilename, "w") as f:
                    f.write(f"datetime,latitude,longitude,heading,signalLevel,noiseLevel,frequency,channel,rxbitrate,txbitrate,txpower,channelWidth,connected,syncing\n")
            except(OSError, NotADirectoryError) as e:
                self.get_logger().error(f"Error attempting to write to {self.logDirectory+self.logFilename} + {e.strerror}")
        try:
            with open(self.logDirectory+self.logFilename, "a") as f:
                f.write(
                    f"{datetime.datetime.now().replace(microsecond=0).isoformat().replace(':','-')},"
                    f"{self.lat},"
                    f"{self.long},"
                    f"{self.heading},"
                    f"{currentStatus.signalLevel},"
                    f"{currentStatus.noiseLevel},"
                    f"{currentStatus.frequency},"
                    f"{currentStatus.channel},"
                    f"{currentStatus.rxbitrate},"
                    f"{currentStatus.txbitrate},"
                    f"{currentStatus.txpower},"
                    f"{currentStatus.channelWidth},"
                    f"{currentStatus.connected},"
                    f"{currentStatus.syncing}\n"
                )

        except(OSError, NotADirectoryError) as e:
            self.get_logger().error(f"Error attempting to write to {self.logDirectory+self.logFilename}: + {e.strerror}")
            

# Read the config file and generate nodes to monitor each device.
def generateNodesFromConfig(cfgpath: str) -> List[WirelessInterfaceMonitor]:
    nodes = list[WirelessInterfaceMonitor]()
    try:
        with open(cfgpath, "r") as f:
            currentInferface = WirelessInterface()
            loggingEnabled = False
            loggingPeriod = 5.0
            updatePeriod = 5.0
            loggingDirectory = ""
            deviceName = ""
            nodeIndex = 0

            def createMonitorNode() -> WirelessInterfaceMonitor:
                print(f"Creating Comms Monitor Node: {deviceName} ({MONITOR_NODE_PREFIX+deviceName})")
                print(f"update_period = {updatePeriod}")
                print(f"logging_period = {loggingPeriod}")
                print(f"logging_enabled = {loggingEnabled}")
                print(f"logging_path = {loggingDirectory}")
                print(f"ip = {currentInferface.remoteAddr}")
                print(f"interface_name = {currentInferface.interfaceName}")
                print(f"username = {currentInferface.username}")
                print(f"password = {currentInferface.password}")
                print(f"interface_type = {currentInferface.type}")
                print()
                return WirelessInterfaceMonitor(deviceName, deepcopy(currentInferface), loggingEnabled, loggingPeriod, updatePeriod, loggingDirectory)

            for l in f:
                tokens = l.split('=')
                try:
                    if tokens[0].strip() == "device_name":
                        if nodeIndex != 0: #use the current data to create a node, so long as this is not the first instance of device_name
                            nodes.append(createMonitorNode())
                        deviceName = tokens[1].strip()
                        nodeIndex += 1
                    elif tokens[0].strip() == "ip":
                        currentInferface.remoteAddr = tokens[1].strip()
                    elif tokens[0].strip() == "interface_name":
                        currentInferface.interfaceName = tokens[1].strip()
                    elif tokens[0].strip() == "username":
                        currentInferface.username = tokens[1].strip()
                    elif tokens[0].strip() == "password":
                        currentInferface.password = tokens[1].strip()
                    elif tokens[0].strip() == "interface_type":
                        if tokens[1].strip() == "GENERIC_IW":
                            currentInferface.type = WirelessInterface.InterfaceType.GENERIC_IW
                        elif tokens[1].strip() == "MM_HALOW":
                            currentInferface.type = WirelessInterface.InterfaceType.MM_HALOW
                        elif tokens[1].strip() == "UBIQUITI_AIROS" or tokens[1].strip() == "AIROS":
                            currentInferface.type = WirelessInterface.InterfaceType.UBIQUITI_AIROS
                    elif tokens[0].strip() == "update_period":
                        updatePeriod = float(tokens[1])
                        currentInferface.syncTimeoutSec = float(tokens[1])
                    elif tokens[0].strip() == "logging_period":
                        loggingPeriod = float(tokens[1])
                    elif tokens[0].strip() == "logging_enabled":
                        loggingEnabled = tokens[1].strip().lower() == "true"
                    elif tokens[0].strip() == "logging_path":
                        loggingDirectory = tokens[1].strip()
                    elif tokens[0].strip() == "":
                        pass #ignore empty lines
                    else:
                        print(f"Wireless_Logger: parameter {tokens[0]} in {cfgpath} is unkown.")
                except(IndexError, ValueError):
                    print(f"Format of line '{l}' in {cfgpath} is invalid. Script Terminated.")
                    exit(1)

            nodes.append(createMonitorNode()) 

            return nodes
        
    except(FileNotFoundError):
        print(f"Failed to open config file: '{cfgpath}'")
        exit(2) 


def main(args=None):

    print(getcwd())

    try: 
        rclpy.init(args=args)

        nodeExecutor = MultiThreadedExecutor()

        for node in generateNodesFromConfig("./comms_control/comms_control/comms_monitor.cfg"):
            nodeExecutor.add_node(node)

        nodeExecutor.spin()

    except (KeyboardInterrupt, ExternalShutdownException):
        rclpy.shutdown()
        pass

    
if __name__ == '__main__':
    main()
    

