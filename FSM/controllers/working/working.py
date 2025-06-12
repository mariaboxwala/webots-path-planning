from controller import Robot, Camera, AnsiCodes

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
        self.bumper = self.getDevice("touch sensor")
        self.bumper.enable(self.timeStep)
        
        # Initialize FSA variables
        self.can_states = [False, False, False]  # Boolean array to track cans found
        self.current_state = "Wander"

    def move_forward(self, speed):
        self.left_motor.setVelocity(speed)
        self.right_motor.setVelocity(speed)

    def stop(self):
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def update_and_check_can_state(self):
        # Update and check if all cans are found
        if all(self.can_states):
            self.current_state = "Wander for Base Station"
        else:
            self.current_state = "Wander"

    def run(self):
        speed = 2
        left_speed = 0
        right_speed = 0
        width = self.camera.getWidth()
        height = self.camera.getHeight()

        while True:
            image = self.camera.getImage()

            if self.current_state == "Wander":
                # Wander state: Move forward
                self.move_forward(speed)

                # Check for can detection
                if self.detect_can(image, width, height):
                    self.current_state = "Move forward"
                    
            elif self.current_state == "Move forward":
                # Move forward state: Check if can touched
                if self.bumper.getValue() > 0:
                    self.update_and_check_can_state()
            
            elif self.current_state == "Wander for Base Station":
                # Wander for Base Station state: Move forward
                self.move_forward(speed)

                # Check for base station detection
                if self.detect_base_station():
                    self.current_state = "Stop"

            elif self.current_state == "Stop":
                # Stop state: Stop robot
                self.stop()

    def detect_can(self, image, width, height):
        # Detect can using camera image
        # Implement your can detection logic here
        # For example, check for specific colors
        return False  # Placeholder, replace with actual detection logic

    def detect_base_station(self):
        # Detect base station using bumper sensor
        # Implement your base station detection logic here
        # For example, check if bumper is triggered by the base station
        return False  # Placeholder, replace with actual detection logic


controller = Controller()
controller.run()
