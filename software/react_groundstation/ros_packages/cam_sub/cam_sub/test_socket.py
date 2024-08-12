import requests
import json
from datetime import datetime
url1 = "http://localhost:5000/img1"


headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
while(True):
    now = datetime.now()
    data1 = {'msg': now.strftime( "%-d %b %Y %H:%M:%S.%f" )}
    try:
        r1 = requests.post(url1, data=json.dumps(data1), headers=headers,timeout = 0.03)
        
    except requests.exceptions.ReadTimeout: 
        pass
    except requests.exceptions.ConnectionError:
        print("Backend server is offline, please check it's status. Exiting...")
        break

    
