import requests
import time

class WiFi:
    def __init__(self, address_url):
        self.web_server_url = address_url
        self.data = None
        self.response = None

    def read_data(self, max_retries=3):
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
            time.sleep(1)  # wait for 1 second before retrying
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
