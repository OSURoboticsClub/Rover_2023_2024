import typing
import subprocess
from enum import Enum

#web packages, used for AIR_OS
import requests
from requests.packages.urllib3.exceptions import InsecureRequestWarning
import urllib3
requests.packages.urllib3.disable_warnings()
requests.packages.urllib3.util.ssl_.DEFAULT_CIPHERS += ':HIGH:!DH:!aNULL'
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

class InterfaceStatus:
    signalLevel: typing.Optional[int] #Recieved signal level dBm
    noiseLevel: typing.Optional[int] #noise level dBm
    frequency: typing.Optional[int] #channel freq in MHz
    channel: typing.Optional[int] #channel #
    rxmcs: typing.Optional[str] #download modulation coding scheme
    txmcs: typing.Optional[str] #upload modulation coding scheme
    rxbitrate: typing.Optional[float] #download bitrate Mbps
    txbitrate: typing.Optional[float] #upload bitrate Mbps
    txpower: typing.Optional[float] #transmit power dBm
    channelWidth: typing.Optional[int] #channel width MHz
    connected: bool #is there a working ssh connection to the target
    syncing: bool #were monitoring commands executed successfully on the target

    def __init__(
                self,
                signalLevel: typing.Optional[int] = None,
                noiseLevel: typing.Optional[int] = None,
                frequency: typing.Optional[int] = None,
                channel: typing.Optional[int] = None,
                rxmcs: typing.Optional[str] = None,
                txmcs: typing.Optional[str] = None,
                rxbitrate: typing.Optional[float] = None,
                txbitrate: typing.Optional[float] = None,
                txpower: typing.Optional[float] = None,
                channelWidth: typing.Optional[int] = None,
                connected: bool = False,
                syncing: bool = False
                 ):
        self.signalLevel = signalLevel
        self.noiseLevel = noiseLevel
        self.frequency = frequency
        self.channel = channel
        self.channelWidth = channelWidth
        self.rxmcs = rxmcs
        self.txmcs = txmcs
        self.rxbitrate = rxbitrate
        self.txbitrate = txbitrate
        self.txpower = txpower
        self.channelWidth = channelWidth
        self.connected = connected
        self.syncing = syncing

    def __str__(self) -> str:
        return \
        f"Signal Level (dBm): {self.signalLevel}\n" + \
        f"Noise Level (dBm): {self.noiseLevel}\n" + \
        f"Frequency MHz (dBm) {self.frequency}\n" + \
        f"Channel: {self.channel}\n" + \
        f"Channel Width (MHz): {self.channelWidth}\n" + \
        f"RX MCS: {self.rxmcs}\n" + \
        f"TX MCS: {self.txmcs}\n" + \
        f"RX Bitrate: {self.rxbitrate}\n" + \
        f"TX Bitrate: {self.txbitrate}\n" + \
        f"TX Power (dBm): {self.txpower}"



class WirelessInterface:

    class InterfaceType(Enum):
        GENERIC_IW = 0
        MM_HALOW = 1
        UBIQUITI_AIROS = 2

    remoteAddr: str
    password: str
    username: str
    interfaceName: str
    type: InterfaceType
    syncTimeoutSec: float
    airOSSession = None
    airOS_login_url = 'https://{0}/login.cgi'

    def __init__(self, remoteAddr: str = "", username: str = "", interfaceName: str = "", password: str = "", type: InterfaceType = InterfaceType.GENERIC_IW, syncTimeoutSec: float = 4.5):
        self.remoteAddr = remoteAddr
        self.interfaceName = interfaceName
        self.username = username
        self.password = password
        self.type = type  
        self.syncTimeoutSec = syncTimeoutSec

    #Gets status information about this interface.
    def getStatus(self) -> InterfaceStatus:
        result = InterfaceStatus()
        try:
            def sshRun(command: str) -> subprocess.CompletedProcess[str]:
                    return subprocess.run(['ssh', f'{self.username}@{self.remoteAddr}', '"{command}"'], text=True, capture_output=True, check=True, timeout=self.syncTimeoutSec)

            def intOrNone(input: str) -> typing.Optional[int]:
                try:
                    return int(input)
                except ValueError:
                    return None

            def floatOrNone(input: str) -> typing.Optional[float]:
                try:
                    return float(input)
                except ValueError:
                    return None

            if self.type == WirelessInterface.InterfaceType.GENERIC_IW:
                #use iw to get interface info. This should contain the channel, frequency, channel width, and tx power.
                output = sshRun(f"iw dev {self.interfaceName} info")

                if output.returncode == 0:
                    tokens = output.stdout.split()
                    for i in range(0, len(tokens)):
                        if (tokens[i] == 'channel'):
                            i += 1
                            result.channel = intOrNone(tokens[i])
                            i += 1
                            result.frequency = intOrNone(tokens[i].removeprefix('('))
                        elif (tokens[i] == 'width:'):
                            i += 1
                            result.channelWidth = intOrNone(tokens[i])
                        elif (tokens[i] == 'txpower'):
                            i += 1
                            result.txpower = floatOrNone(tokens[i])
                else:
                    print(f"Failed to connect to {self.username}@{self.remoteAddr} over ssh.")

                #use iw to get link info. Should contain signal level, bitrates, and mcs info
                output = sshRun(f"iw dev {self.interfaceName} link")

                if output.returncode == 0:
                    lines = output.stdout.splitlines()
                    for l in lines:
                        tokens = l.split()
                        if (tokens[0] == 'signal:'):
                            result.channel = intOrNone(tokens[1])
                        elif (tokens[0] == 'rx bitrate:'):
                            result.rxbitrate = floatOrNone(tokens[1])
                            result.rxmcs = str.join(' ', tokens[1:len(tokens)])
                        elif (tokens[0] == 'tx bitrate:'):
                            result.txbitrate = floatOrNone(tokens[1])
                            result.txmcs = str.join(' ', tokens[1:len(tokens)])
                else:
                    print(f"Failed to connect to {self.username}@{self.remoteAddr} over ssh.")

                #attempt to get noise level from /proc/net/wireless
                output = sshRun(f"cat /proc/net/wireless")

                if output.returncode == 0:
                    lines = output.stdout.splitlines()
                    for l in lines:
                        tokens = l.split()
                        if (tokens[0] == self.interfaceName+':'):
                            result.noiseLevel = intOrNone(tokens[4])
                else:
                    print(f"Failed to connect to {self.username}@{self.remoteAddr} over ssh.")

            elif self.type == WirelessInterface.InterfaceType.MM_HALOW:

                output = sshRun(f"morse_cli -i {self.interfaceName} stats")

                lines = output.stdout.splitlines()

                for l in lines:
                    tokens = l.split(':')
                    if tokens[0] == "Received Power (dBm)":
                        result.signalLevel = intOrNone(tokens[1])
                    elif tokens[0] == "Noise dBm":
                        result.noiseLevel = intOrNone(tokens[1])
                    elif tokens[0] == "Current RF frequency Hz":
                        result.frequency = intOrNone(tokens[1])
                        if type(result.frequency) == int:
                            result.frequency //= 1000000
                    elif tokens[0] == "Current Operating BW MHz":
                        result.channelWidth = intOrNone(tokens[1])

                output = sshRun(f"iwinfo {self.interfaceName} info") 

                for l in lines:
                    tokens = l.split(':')
                    if tokens[0].strip() == "Tx-Power":
                        result.txpower = intOrNone(tokens[1])
                    elif tokens[0].strip() == "Mode":
                        result.channel = intOrNone(tokens[3])
                    elif tokens[0].strip() == "Bit Rate":
                        result.txbitrate = floatOrNone(tokens[1])  

            elif self.type == WirelessInterface.InterfaceType.UBIQUITI_AIROS:
                ubiquiti_properties = {"wireless" : {"channel":None, "frequency":None, "opmode":None, "signal":None, "noisef":None, "rssi":None, "txpower":None, "distance":None, "ccq":None, "rxrate":None, "txrate":None}}

                #This function logs in to the Ubiquiti device and returns a logged in session.
                # Inputs: None
                # Outputs:
                #   Requests Session s
                def airOSLogin():
                    
                    s = requests.Session()
                    
                    #This seems unecessary but things breaks when it's removed...
                    s.get(
                        url = self.airOS_login_url.format(self.remoteAddr),
                        verify=False
                    )
                
                    # conduct the login
                    r = s.post(
                        url=self.airOS_login_url.format(self.remoteAddr),
                        data={
                            'username': self.username,
                            'password': self.password
                        },
                        verify=False,
                        timeout=self.syncTimeoutSec,
                    )
                
                    # Error if the login fails
                    if r.status_code > 201:
                        #Return None to be handled externally
                        return None
                
                    #Return the logged-in session so we can use it to get data
                    return s

                #This function requests the data from the rocket m2 and returns it as a dict of only the items specified in ubiquiti_properties
                # Inputs:
                #   request session s
                # Outputs:
                #   Data as a flattened dictionary
                def request_data(s: requests.Session):

                        url = f"https://{self.remoteAddr}/status.cgi"

                        try:
                            r = s.get(
                                url=url,
                                verify=False,
                                timeout=self.syncTimeoutSec
                            )
                            
                            r_dict = r.json()

                            output = {}

                            #Get the desired entries from the returned json file:
                            output = extract_flat_dict(ubiquiti_properties, r_dict)
                            #print(output)

                            return output

                        except:
                            raise
                    
                #This function recursviely runs to flatten the Initial defined dictionary (It's kinda dumb to have two of these but whatever):
                # Inputs: 
                #    properties (dict, with each entry having a value of None or another dict based on the structure)
                # Outputs:
                #    Array of Flattened key names (used for data headers and retrieving extracted data)
                def extract_flat_keys(properties):
                
                    output = []

                    for key, value in properties:
                        #Check if it has any child properties:
                        if value == None:
                            output.append(key)
                        else:
                            #If it has children, recursively call to flatten:
                            inner_output = extract_flat_keys(properties[key])
                            #Iterate over the returned dictionary, and rewrite the keys with an underscore in between:
                            for i_key in inner_output:
                                output.append(key + "_" + i_key)

                    return output

                #This function recursviely runs to flatten the returned dictionary:
                # Inputs: 
                #    properties (dict, with each entry having a value of None or another dict)
                #    data (dict, containing data corresponding to the keys in properties)
                # Outputs:
                #    Flattened dictionary with flattened key names and desired fields filled in from the data dict
                def extract_flat_dict(properties,data):

                    output = dict()
                    for key, value in properties.items():
                        #Check if it has any child properties:
                        if value == None:
                            output[key] = data[key]
                        else:
                            #If it has children, recursively call to flatten:
                            inner_output = extract_flat_dict(properties[key],data[key])
                            #Iterate over the returned dictionary, and rewrite the keys with an underscore in between:
                            for i_key, i_value in inner_output.items():
                                output[ key + "_" + i_key] = inner_output[i_key]

                    return output
                
                try:
                    if self.airOSSession == None:
                        #print(f"Attempting Login {self.remoteAddr}")
                        self.airOSSession = airOSLogin() #TODO Fix hang on login attempt when on a conflicting network
                    if self.airOSSession != None:
                        #print(f"Attempting Data Request {self.remoteAddr}")
                        airOSdata = request_data(self.airOSSession)
                except (requests.exceptions.HTTPError):
                    result.connected = True
                    result.syncing = False
                    return result
                except (requests.exceptions.ConnectionError, requests.exceptions.ReadTimeout, requests.exceptions.RequestException):
                    result.connected = False
                    result.syncing = False
                    return result
                
                result.channel     =   int(airOSdata.get("wireless_channel"))
                result.frequency   =   int(airOSdata.get("wireless_frequency").split(' ')[0])
                result.signalLevel =   int(airOSdata.get("wireless_signal"))
                result.noiseLevel  =   int(airOSdata.get("wireless_noisef"))
                result.txpower     = float(airOSdata.get("wireless_txpower"))
                result.rxbitrate   = float(airOSdata.get("wireless_rxrate"))
                result.txbitrate   = float(airOSdata.get("wireless_txrate"))


            result.connected = True
            result.syncing = True

        except ChildProcessError as e:
            if e.errno == 255:
                result.connected = True
                result.syncing = False
                return result
            else:
                result.connected = False
                result.syncing = False
                return result
        except subprocess.TimeoutExpired as e:
            result.connected = False
            result.syncing = False
            return result

        return result


    