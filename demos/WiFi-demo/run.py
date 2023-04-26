import sys
sys.path.append( '../../')
from modules.WiFi import WiFi

address_url = 'http://13.56.207.97:5000'
data = {"HB":0,"IO":1,"WO":0,"DM":"D","CMD":[0,0]}
rover_comms = WiFi(address_url)

while True:
    incoming_data = rover_comms.read_data()
    print(incoming_data)
    rover_comms.write_data(data)