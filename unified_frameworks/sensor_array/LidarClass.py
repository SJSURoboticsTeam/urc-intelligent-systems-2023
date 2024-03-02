from abc import ABC, abstractmethod
class LidarDisconnectException(Exception):
    pass
class Lidar(ABC):
    @abstractmethod
    def connect(self, max_attempts=3, wait_seconds=1, verbose_attempts=False) -> bool:
        """Make attempts to connect to the Lidar

        Parameters
        -----------
        max_attempts: Number of attempts to make
        wait_seconds: Seconds to wait between each attempt 

        Returns
        _______
        True if connection is made
        """
        pass
    @abstractmethod
    def disconnect(self):
        """Disconnect from the Lidar"""
        pass
    @abstractmethod
    def get_measures(self):
        """Get measures from the lidar as a list of 3-tuples (quality, angle degrees, distance meter)"""
        pass
    def test_Lidar(lidar, connection_attempts=3):
        import sys, time
        if not lidar.connect(connection_attempts, verbose_attempts=True):
            print("failed to connect")
            sys.exit(0)
        print("Connected")
        start_time = time.time()
        while time.time() - start_time < 10:
            print(len(lidar.get_measures()))
            time.sleep(1)
        lidar.disconnect()
        print("disconnected")