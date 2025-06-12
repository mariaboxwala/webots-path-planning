from controller import Robot, Camera, AnsiCodes
import os
import random

if os.name == 'nt':
    from ctypes import create_unicode_buffer, windll

class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        
        # Camera
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timeStep)
        
        # Motors
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Bumper Sensor
        self.bumper = self.getDevice("bumper")
        self.bumper.enable(self.timeStep)
        
        # Initialize FSA variables
        self.can_states = [False, False, False]  # Boolean array to track cans found
        self.current_state = "Wander" #intial state
        
    def move_forward(self, speed): #to move the robot forward
        self.left_motor.setVelocity(speed)
        self.right_motor.setVelocity(speed)
    
    def move_backward(self, speed): #this is to move the robot backward while changing orientation once the robot bumps in an obstacle
        self.left_motor.setVelocity(-speed)
        self.right_motor.setVelocity(-0.5*speed)
        current_time_1 = self.getTime()
        current_time_2 = self.getTime()
        while current_time_2 < (current_time_1 + 4):
            current_time_2 = self.getTime()
            self.step(1)
   
    def wander(self, speed): #move with a random rotation
        noise = random.random()*0.25
        self.left_motor.setVelocity(speed-noise)
        self.right_motor.setVelocity(speed+noise)

    def stop(self): #Stop the robot
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def update_can_state(self, index): #to update the can state once a unique can is detected
        if index != -1 and index != 3:
            self.can_states[index] = True
        print("----------------------------")
        print("red", self.can_states[0])
        print("green", self.can_states[1])
        print("yellow", self.can_states[2])
        print("----------------------------")
    
    def check_state(self): # check if all the cans were detected
        done = True
        for i in range(len(self.can_states)):
            if(self.can_states[i]) == False:
                done = False;
        return done
    
    def detect_can(self, image, width, height): # this is the function that returns the color detected
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        image = self.camera.getImage()
        red = 0
        green = 0
        blue = 0
        
        for i in range(int(width / 3), int(2 * width / 3)):
            for j in range(int(height / 4), int(3 * height / 4)):
                red += Camera.imageGetRed(image, width, i, j)
                green += Camera.imageGetGreen(image, width, i, j)
                blue += Camera.imageGetBlue(image, width, i, j)
                # If a component is much more represented than the other ones,
                # a blob is detected
                if red < 100 and green < 100 and blue < 100:
                    current_blob = -1
                elif red > 3 * green and red > 3 * blue:
                    current_blob = 0  # red
                elif green > 3 * red and green > 3 * blue:
                    current_blob = 1  # green
                elif red > 3 * blue and green > 3 * blue:
                    current_blob = 2  # yellow
                elif blue > 3 * red and blue > 3 * green:
                    current_blob = 3  # blue
                else:
                    current_blob = -1
        return current_blob

    def run(self):
        speed = 1
        left_speed = 0
        right_speed = 0
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        
        while self.step(self.timeStep) != -1:
            image = self.camera.getImage()
            
            if self.current_state == "Wander": # Wander state, Start state
                self.wander(speed) # to keep wandering
                done = self.check_state()
                color = self.detect_can(image, width, height)
                if done:
                   self.current_state = "Wander for Base Station" #if all cans are detected then the robot changes to this state
                elif self.bumper.getValue() > 0:
                   self.move_backward(speed) 
                   self.current_state = "Wander" #if the robot hits anything then it adjusts and goes to wander state
                elif  color != -1 and color !=3:
                    detected = self.detect_can(image, width, height)
                    self.update_can_state(detected)
                    self.current_state = "Move forward" #if the robot detects any can using the camera then the robot will go to this state
                    
            elif self.current_state == "Move forward": # Move forward state
                self.move_forward(speed) # to keep moving forward
                if self.bumper.getValue() > 0:
                       self.move_backward(speed) 
                       self.current_state = "Wander" #if the robot hits anything the robot adjusts and goes to wander state

            elif self.current_state == "Wander for Base Station":# Wander for Base Station state
                self.wander(speed) # to keep wandering
                color = self.detect_can(image, width, height)
                if self.bumper.getValue() > 0: 
                     self.move_backward(speed) 
                     self.current_state = "Wander for Base Station" # if the robot detects an obstacle it adjusts and returns to same state
                elif color == 3:
                    self.current_state = "Move forward to Base Station" #if the base station is detected then we change to this state
            
            elif self.current_state == "Move forward to Base Station":
                self.move_forward(speed) # to keep moving forward
                if self.bumper.getValue() > 0:
                   self.current_state = "Stop" #we stop once the robot hits the base station or in other words arrives at the base station
                      
            elif self.current_state == "Stop": # Stop state, the Final state
                    self.stop()
                    exit()

controller = Controller()
controller.run()
