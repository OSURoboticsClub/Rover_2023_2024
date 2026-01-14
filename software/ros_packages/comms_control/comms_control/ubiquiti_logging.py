#ROS Imports
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

#GPS Message Type (TODO: Change to propper type)
from rover2_control_interface.msg import GPSStatusMessage

#Web Imports
# Requirements:
# requests package (pip install requests), using 2.32.5
# urllib3, Specifically needs a pre-2.0 version, using 1.26.20

import requests
from requests.packages.urllib3.exceptions import InsecureRequestWarning
import urllib3
import datetime
import time

requests.packages.urllib3.disable_warnings()
requests.packages.urllib3.util.ssl_.DEFAULT_CIPHERS += ':HIGH:!DH:!aNULL'
requests.packages.urllib3.disable_warnings(InsecureRequestWarning)

#Global Constants:
####################################################################################
USERNAME = 'rover'
PASSWORD = 'makemorerobot'
ip = '192.168.1.25'
login_url = 'https://{0}/login.cgi'

#List of URLs we can request from on the Ubiquiti Rocket
urls = [
    'http://{0}/status.cgi', #Json format-able, has most of what we want

    # 'http://{0}/sta.cgi', #Empty but might be a json-able form of stalist.cgi?
    # 'http://{0}/stalist.cgi', #Not Json-able, I think this is a list of stations/transcievers?
    # 'http://{0}/getcfg.sh', #Empty?
    # 'http://{0}/getboardinfo.sh', #Not json-able, simple plain text but mostly hw config stuff

    # 'http://{0}/brmacs.cgi?brmacs=y', #Json format-able, shows mac address/hw adapters connected?
    # 'http://{0}/arp.cgi', #Not Json format-able, same as brmacs...

    # 'http://{0}/sroutes.cgi', #Not Json format-able, seems to show routes between devices somehow? might be useful for testing with a relay...
    # 'http://{0}/ifstats.cgi',  #Json format-able, but not super useful
    # 'http://{0}/iflist.cgi', #Json format-able, unsure if useful. Lists adapters/devices? 
    # 'http://{0}/log.cgi', #Not Json format-able, unsure if useful, need to check what the log shows when connected.
]


#List of GPS and Ubiquiti Properties
gps_properties = ["Python datetime","latitude","longitude"]
ubiquiti_properties = [{"wireless" : {"channel":None, "frequency":None, "opmode":None, "signal":None, "noisef":None, "rssi":None, "txpower":None, "distance":None, "ccq":None,}}]

file_dir = "" #TODO: Make this save to a directory that makes sense?

#Class Defining our Node:
####################################################################################
class UbiquitiLogger(Node):

    def __init__(self):
        #Initialize the Node:
        super().__init__('ubiquiti_logger')
                
        #flattened ubiquiti keys
        self.ubiquiti_keys = extract_flat_keys(ubiquiti_properties[0])

        #Start the session and login:
        while True:
            self.s = login()

            #Handle if there's a request error or no connection
            if self.s == None:
                self.get_logger().warn("No Connection or Failed to login")
            else:
                self.get_logger().info("Login Successful")
                break
            #Rate limit login attempts
            time.sleep(1)

        #Open the CSV file and write the headers 
        self.file = open(file_dir + get_filename(),'x')
        write_headers(self.file,gps_properties,self.ubiquiti_keys)

        #Initialize the GPS topic subscription
        #Do this after starting the session and opening the file in case it matters/to avoid a race condition?
        self.subscription = self.create_subscription(
            GPSStatusMessage,
            'tower/status/gps',
            self.gps_callback,
            10)

    #Upon recieving a GPS message, request Ubiquiti Data and log both:
    def gps_callback(self, msg):
        #Request le Data
        ubiquiti_data_dict = request_data(self.s)

        if ubiquiti_data_dict == None:
            self.get_logger().warn("Failed to Read Ubiquiti data")
            return #TODO: Have it restart the session/reattempt login if it fails ideally?

        gps_data = [datetime.datetime.now().time(), msg.rover_latitude, msg.rover_longitude] #Placeholder, replace with ros message data

        #Actually write to the file :)
        write_data(self.file,gps_data,self.ubiquiti_keys,ubiquiti_data_dict)


#Function definitions:
####################################################################################
#This function requests the data from the rocket m2 and returns it as a dict of only the items specified in ubiquiti_properties
# Inputs:
#   request session s
# Outputs:
#   Data as a flattened dictionary
def request_data(s):
    
    #Scrape the URLs
    for i,url in enumerate(urls):
        
        url = url.format(ip)

        try:
            r = s.get(
                url=url,
                verify=False
            )

            r_dict = r.json()

            output = {}

            #Get the desired entries from the returned json file:
            output = extract_flat_dict(ubiquiti_properties[i], r_dict)
            #print(output)
            
            return output
        
        except:
            #Return None so the error can be handled externally
            return None

#This function recursviely runs to flatten the Initial defined dictionary (It's kinda dumb to have two of these but whatever):
# Inputs: 
#    properties (dict, with each entry having a value of None or another dict based on the structure)
# Outputs:
#    Array of Flattened key names (used for data headers and retrieving extracted data)
def extract_flat_keys(properties):

    output = []

    for key, value in properties.items():
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
    
    output = {}
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

#This function returns a filename to store the data in
# Inputs: None
# Outputs:
#   file name as string
def get_filename():

    t_stamp = datetime.datetime.now().replace(microsecond=0).isoformat()

    t_stamp = t_stamp.replace(":","-")

    f_name = "ubiquiti_data_" + t_stamp + ".csv"

    return f_name

#This function writes the data headers to the csv file
# Inputs:
#   file handle
#   gps headers as a list of strings
#   ubiquiti headers as a list of strings 
# Outputs:
#   None
def write_headers(fhandle,gps_headers,ubiquiti_headers):
    
    #Write headers as a csv
    headers = ""
    
    #Add the gps data first
    for header in gps_headers:
        headers = headers + header + ","

    #and add the ubiquiti headers second
    for header in ubiquiti_headers:
        headers = headers + header + ","

    #End of line
    headers = headers + "\n"

    fhandle.write(headers)

#This function writes the data to the csv file
# Inputs:
#   file handle
#   timestamped gps data as a list
#   ordered list of keys for the ubiquiti data
#   ubiquiti data as an array 
# Outputs:
#   None
def write_data(fhandle, gps_data, ubiquiti_keys, ubiquiti_data):

    #Write data as a csv line
    line = ""

    #Add the gps data:
    for datum in gps_data:
        line = line + str(datum) + ","

    #Add the ubiquiti data:
    for key in ubiquiti_keys:
        line = line + str(ubiquiti_data[key]) + ","

    line = line + "\n"

    fhandle.write(line)

#This function logs in to the Ubiquiti device and returns a logged in session.
# Inputs: None
# Outputs:
#   Requests Session s
def login():
    
    s = requests.Session()
    
    #This seems unecessary but things breaks when it's removed...
    try:
        s.get(
            url = login_url.format(ip),
            verify=False
        )
    except:
        return None

    # conduct the login
    r = s.post(
        url=login_url.format(ip),
        data={
            'username': USERNAME,
            'password': PASSWORD
        },
        verify=False,
        timeout=500,
    )

    # Error if the login fails
    if r.status_code > 201:
        #Return None to be handled externally
        return None

    #Return the logged-in session so we can use it to get data
    return s


#Node Main Entry point function: 
####################################################################################
def main(args=None):
    try:
        rclpy.init(args=args)
        ubiquiti_logger = UbiquitiLogger()

        rclpy.spin(ubiquiti_logger)

        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        rclpy.shutdown()
        pass

