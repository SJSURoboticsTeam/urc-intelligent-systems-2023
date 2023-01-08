import RPi.GPIO as GPIO

class IR_Sensor:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pin,GPIO.IN)

    def DetectObject(self):
        if GPIO.input(self.pin):
            return 1
        else:
            return 0