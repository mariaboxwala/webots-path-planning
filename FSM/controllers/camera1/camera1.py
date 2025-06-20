from controller import Robot, Camera, AnsiCodes
import os

if os.name == 'nt':
    from ctypes import create_unicode_buffer, windll


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        
        #Camera
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timeStep)
        
        #Motors
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        #Bumper Sensor
        self.bumper = self.getDevice("touch sensor")
        self.bumper.enable(self.timeStep)
        

    def run(self):
        speed = 2
        pause_counter = 0
        left_speed = 0
        right_speed = 0
        red = 0
        green = 0
        blue = 0
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        color_names = ['red', 'green', 'blue']
        ansi_colors = [AnsiCodes.RED_FOREGROUND, AnsiCodes.GREEN_FOREGROUND, AnsiCodes.BLUE_FOREGROUND]

        while True:
            image = self.camera.getImage()
            if pause_counter > 0:
                pause_counter -= 1
            # Case 1
            # A blob was found recently
            # The robot waits in front of it until pause_counter
            # is decremented enough
            if pause_counter > 640 / self.timeStep:
                left_speed = 0
                right_speed = 0
            # Case 2
            # A blob was found quite recently
            # The robot begins to turn but don't analyse the image for a while,
            # otherwise the same blob would be found again
            elif pause_counter > 0:
                left_speed = -speed
                right_speed = speed
            # Case 3
            # The robot turns and analyse the camera image in order
            # to find a new blob
            elif not image:  # image may be NULL if Robot.synchronization is FALSE
                left_speed = 0
                right_speed = 0
            else:  # pause_counter == 0
                # Here we analyse the image from the camera. The goal is to detect a
                # blob (a spot of color) of a defined color in the middle of our
                # screen.
                # In order to achieve that we simply parse the image pixels of the
                # center of the image, and sum the color components individually
                image = self.camera.getImage()
                red = 0
                green = 0
                blue = 0
                for i in range(int(width / 3), int(2 * width / 3)):
                    for j in range(int(height / 2), int(3 * height / 4)):
                        red += Camera.imageGetRed(image, width, i, j)
                        green += Camera.imageGetGreen(image, width, i, j)
                        blue += Camera.imageGetBlue(image, width, i, j)
                # If a component is much more represented than the other ones,
                # a blob is detected
                if red > 3 * green and red > 3 * blue:
                    current_blob = 0  # red
                elif green > 3 * red and green > 3 * blue:
                    current_blob = 1  # green
                elif blue > 3 * red and blue > 3 * green:
                    current_blob = 2  # blue
                else:
                    current_blob = None
                # Case 3a
                # No blob is detected: the robot continues to turn
                if current_blob is None:
                    left_speed = 0
                    right_speed = 0
                # Case 3b
                # A blob is detected
                # the robot stops, stores the image, and changes its state
                else:
                    left_speed = 0
                    right_speed = 0
                    print('Looks like I found a ' + ansi_colors[current_blob] +
                          color_names[current_blob] + AnsiCodes.RESET + ' blob.')
                    pause_counter = 1280 / self.timeStep
            # Set the motor speeds.
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)


controller = Controller()
controller.run()
