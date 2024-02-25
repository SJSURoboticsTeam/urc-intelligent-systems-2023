"""File to hold helpful util functions/classes"""
from threading import Thread
import time

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

def printLog(TAG, log):
    print(f"{TAG}: {log}")


if __name__=="__main__":
    def func(service_running):
        while service_running():
            print("Still runnning")
            time.sleep(1)
    service = Service(func, "TestService")
    service.start_service()
    time.sleep(10)
    service.stop_service()