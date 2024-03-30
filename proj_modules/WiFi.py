import requests
import time

class WiFi:
    def __init__(self, address_url):
        self.web_server_url = address_url
        self.data = None
        self.response = None

    def read_data(self, max_retries=3, wait_time=1):
        """Gets data from mission control. Expected response: {"HB":int, "IO":int, "WO":int, "DM":char, "CMD":list}

        PARAMS:
            max_retries [int]: max number of requests we send to web server before giving up 
        RETURNS:
            JSON data from mission control (if successful. Otherwise None).
        """
        for retry_count in range(max_retries):
            try:
                self.response = requests.get(f'{self.web_server_url}/drive/status', timeout=5)
                if self.response.status_code == 200:
                    data_received = self.response.json()
                    print('Data received:', data_received)
                    return data_received
                else:
                    print('Failed to get data: status code', self.response.status_code)
            except requests.exceptions.RequestException as e:
                print(f'Error getting data (retry {retry_count + 1}/{max_retries}):', e)
            time.sleep(wait_time)  # wait for 1 second before retrying
        print('Max retries exceeded, giving up.')


    def write_data(self, data):
        """Sends data to mission control using an http post request

        PARAMS:
            data [dict]: something
        RETURNS:
            None. Just prints out whether or not data was successfully sent or not
        """
        try:
            self.response = requests.post(f'{self.web_server_url}/drive', json=data, timeout=5)
            if self.response.status_code == 200:
                print('Data sent successfully')
            else:
                print('Failed to send data: status code', self.response.status_code)
        except requests.exceptions.RequestException as e:
            print('Error sending data:', e)
    def send_command(self, data):
        """Alias for write_data(..) | Send command to the rover"""
        self.write_data(data)
    def get_status(self, max_retries=3, wait_time=1):
        """Gets data from mission control. Expected response: {"HB":int, "IO":int, "WO":int, "DM":char, "CMD":list}

        PARAMS:
            max_retries [int]: max number of requests we send to web server before giving up 
        RETURNS:
            JSON data from mission control (if successful. Otherwise None).
        """
        for retry_count in range(max_retries):
            try:
                self.response = requests.get(f'{self.web_server_url}/drive/status', timeout=5)
                if self.response.status_code == 200:
                    data_received = self.response.json()
                    return data_received
                else:
                    return None
            except requests.exceptions.RequestException as e:
                print(f'Error getting data (retry {retry_count + 1}/{max_retries}):', e)
            time.sleep(wait_time)  # wait for 1 second before retrying
        # print('Max retries exceeded, giving up.')

class Modes:
    DRIVE="D"
    SPIN="S"
    TRANSLATE="T"
def make_drive_command(mode=None, speed_percent=None, angle_degrees=None):
    """
    Create a command to be written to the rover

    Parameters
    ----------
    mode: str
        Specify the mode in which to interpret command
        D: Drive, S: Spin, T: Translate
        Default to "D"
    speed_percent: int
        Specify the speed as a % of max speed. Can be +-
    angle_degree: int
        Specify angle the rover should deviate by
        Look forward to this parameter being modified in the future"""
    command = {
        "HB": 0,        # Heart Beat
        "IO": 1,        # Is Operational
        "WO": 0,        # Wheel Orientation
        "DM": "D",      # Drive mode {D: Drive, S: Spin, T: Translate}
        "CMD": [0,0]    # Command [Speed, Angle in Degrees]
    }
    if mode is not None:
        command["DM"] = mode
    if speed_percent is not None:
        command['CMD'][0] = speed_percent
    if angle_degrees is not None:
        command["CMD"][1] = angle_degrees
    return command