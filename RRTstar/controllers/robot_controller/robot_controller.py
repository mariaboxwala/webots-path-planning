"""robot_controller controller."""

from controller import Robot
import random

class Controller(Robot):

    NB_SENSORS = 8
    RANGE = 512
    MATRIX = [[11, 12, 8, -2, -3, -5, -7, -9], [-9, -8, -5, -1, -2, 6, 12, 11]]
    timeStep = 64
    
    def __init__(self):
        super(Controller, self).__init__()
        # Initialize sensors
        self.ps = []
        for i in range(self.NB_SENSORS):
            self.ps.append(self.getDevice('ds' + str(i)))
            self.ps[i].enable(self.timeStep)
        
        # Initialize motors
        self.left_motor = self.getDevice("left wheel motor")
        self.right_motor = self.getDevice("right wheel motor")
        
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
    
    
    def run(self):
        while self.step(self.timeStep) != -1:
            wander_speed = random.uniform(0, 10)
            sensor_value = []
            speed = [0,0]
            for i in range(self.NB_SENSORS):
                sensor_value.append(self.ps[i].getValue())
            for i in range(2):
                for j in range(self.NB_SENSORS):
                    speed[i] += self.MATRIX[i][j] * (1 - (sensor_value[j] / self.RANGE))
            # Set the motor speeds
            self.left_motor.setVelocity(0.2 * speed[0])
            self.right_motor.setVelocity(0.2 * speed[1])
            


controller = Controller()
controller.run()