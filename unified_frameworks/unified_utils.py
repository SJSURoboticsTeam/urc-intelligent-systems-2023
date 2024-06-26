"""File to hold helpful util functions/classes"""
from threading import Thread
import time, json
from math import sqrt, cos, atan2, sin, pi
import numpy as np
from abc import ABC, abstractmethod

config = {
    'time_analysis': True,
    'decimal_precision': 5,
}

class _Abstract_Service(ABC):
    @abstractmethod
    def start_service(self):
        pass
    @abstractmethod
    def stop_service(self):
        pass

class Service(_Abstract_Service):
    def __init__(self, service_func, name) -> None:
        """
        Create an easy to start/stop service from a function. 
        The function must accept a callable which returns if the service is still running. 
        That callable must be respected to prevent the service from running amok.
        """
        self._running = False
        self._service_func = service_func
        self._thread = Thread(target=service_func, args=(lambda: self._running,), name=name)
    def start_service(self):
        self._running=True
        self._thread.start()
    def stop_service(self):
        self._running=False
        self._thread.join()
        self._thread = Thread(target=self._service_func, args=(lambda: self._running,), name=self._thread.name)
    def is_running(self):
        return self._running

def printLog(TAG, log):
    print(f"{TAG}: {log}")

exe_times = {}
track_count = 10
def track_time(func: callable):
    def inner(*args, **kwargs):
        begin = time.time()
        res = func(*args, **kwargs)
        end = time.time()
        m = func.__module__
        n = func.__name__
        default_val = {
            "calls": 0,
            "times": [0]*track_count
        }
        exe_times.setdefault(m, {}).setdefault(n, default_val)['calls']+=1
        exe_times[m][n]['times'] = exe_times[m][n]['times'][1:] + [end-begin]
        return res
    return inner if config['time_analysis'] else func
def keep_track(is_active):
    while is_active():
        for m in exe_times:
            for f in exe_times[m]:
                avg_time = sum(exe_times[m][f]['times'])
                exe_times[m][f]['runtime'] = avg_time/10
                exe_times[m][f]['frequency'] = 10/avg_time if avg_time > 0 else float('inf')
                exe_times[m][f]['total_time'] = exe_times[m][f]['calls']*avg_time/10
        with(open("frequency_analysis.json", "w")) as f:
            f.write(json.dumps(exe_times, indent=4))
        time.sleep(1)
time_tracking_service = Service(keep_track, "Time Tracking Service")

@track_time
def polar_dis(p1, p2):
    """Distance between polar coordinates p1 and p2"""
    if p1 is None or p2 is None: return float('inf')
    return sqrt(abs(p1[1]**2 + p2[1]**2 - 2*p1[1]*p2[1]*cos(p1[0]-p2[0])))
@track_time
def same_polar_point(p1, p2, thresh=0.1):
    # return False
    return polar_dis(p1, p2) < thresh
@track_time
def polar_to_cart(p) -> np.ndarray:
    if p is None: return None
    return np.round(p[1]*np.array([cos(p[0]), sin(p[0])]), config['decimal_precision'])
@track_time
def cart_to_polar(coord):
    if coord is None: return None
    x, y = coord
    res = np.round((atan2(y, x), np.linalg.norm(coord)),config['decimal_precision'])
    # print(f"cart_to_polar: {res}")
    return res
@track_time
def polar_sum(*polar_points):
    cart_points = [polar_to_cart(p) for p in polar_points if p is not None]
    return cart_to_polar(sum(cart_points, np.array([0,0])))
@track_time
def three_point_deviation(polar1, polar2, polar3):
    if polar1 is None: 
        a = (pi/2,0)
    else:
        a = polar_sum(polar2, (polar1[0], -polar1[1]))
    b = polar_sum(polar3, (polar2[0], -polar2[1]))
    return b[0]-a[0]


if __name__=="__main__":
    def func(service_running):
        while service_running():
            print("Still runnning")
            time.sleep(1)
    service = Service(func, "TestService")
    service.start_service()
    time.sleep(10)
    service.stop_service()