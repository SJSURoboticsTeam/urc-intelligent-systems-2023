class Lidar:
    def iter_scans(self):
        """
        Get an Iterable that iteratively returns a list of scanned points
        Each Point is a 3 tuple of (quality, angle_degrees, distance_millimeters)
        """
        pass
    def stop(self):
        pass
    def stop_motor(self):
        pass
    def disconnect(self):
        pass