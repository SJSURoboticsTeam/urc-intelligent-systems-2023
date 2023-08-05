import random
import time

class MockWiFi:
    def __init__(self, address_url):
        self.web_server_url = address_url
        self.data = None
        self.response = None

    def read_data(self, max_retries=3):
        for retry_count in range(max_retries):
            try:
                # Simulate a response with random data
                simulated_data = self._generate_random_data()
                print('Simulated data received:', simulated_data)
                return simulated_data
            except Exception as e:
                print(f'Error getting data (retry {retry_count + 1}/{max_retries}):', e)
            time.sleep(1)  # wait for 1 second before retrying
        print('Max retries exceeded, giving up.')

    def write_data(self, data):
        try:
            # Simulate sending data
            self._simulate_data_sent(data)
        except Exception as e:
            print('Error sending data:', e)

    def _generate_random_data(self):
        """Generate random data as if it's received from the server."""
        return {"HB": random.randint(0,10),"IO":random.randint(0,1),"WO":0,"DM":random.choice(["D","S","R","T"]),"CMD":[0,0]}

    def _simulate_data_sent(self, data):
        """Simulate sending data to the server."""
        # Replace this with any logic that would handle data sending to the server
        print('Simulated data sent successfully')

