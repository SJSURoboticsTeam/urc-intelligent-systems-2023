import math

class trajectory:
    
    def object_detected(self, d):
            angle = math.atan(d, d)
            self.commands[5] = angle  
            trajectory_commands = self.jsonify_commands(self.commands)
            return trajectory_commands