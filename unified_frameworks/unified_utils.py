"""File to hold helpful util functions/classes"""
from threading import Thread
import time, json

config = {
    'time_analysis': True
}

class Service():
    def __init__(self, service_func, name) -> None:
        """
        Create an easy to start/stop service from a function. 
        The function must accept a callable which returns if the service is still running. 
        That callable must be respected to prevent the service from running amok.
        """
        self._running = False
        self._thread = Thread(target=service_func, args=(lambda: self._running,), name=name)
    def start_service(self):
        self._running=True
        self._thread.start()
    def stop_service(self):
        self._running=False
        self._thread.join()
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


if __name__=="__main__":
    def func(service_running):
        while service_running():
            print("Still runnning")
            time.sleep(1)
    service = Service(func, "TestService")
    service.start_service()
    time.sleep(10)
    service.stop_service()