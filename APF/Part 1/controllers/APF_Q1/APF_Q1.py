"""APF_Q1 controller."""
from controller import Robot
import math

class Controller(Robot):
    timestep = 32
    
    def __init__(self):
        super(Controller, self).__init__()
        
        # Motors
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # GPS
        self.gps = self.getDevice('gps')
        self.gps.enable(self.timestep)
        
        #imu
        self.imu = self.getDevice('inertial unit')
        self.imu.enable(self.timestep)
        
        # Pen
        self.pen = self.getDevice('pen')
        

    def calculate_motor_speeds(self, theta, imu_value, d):
        
        max_speed = 6.28  # Maximum speed for each motor
        k_orientation = 0.5  # Proportional gain for orientation control
        k_distance = 1 # Proportional gain for distance control to slow down as the robot approaches the goal
        
        #here we have this if statement to deal with the scenario when the error oscillates between negative and positive
        #So by doing this when the robot is facing away it will not be stuck and can rotate with error being positive only
        #When it leaves the threshold it will behave normally and move left or right based on the sign of the error
        if(abs(theta-imu_value[2]) >= math.pi):
            error = abs(theta-imu_value[2])
        else:
            error = theta-imu_value[2]
        # Implement a deadzone for orientation correction, so if the robot is slightly not alligned you do not have to change the two speeds
        orientation_deadzone = 0.01  # Adjust this value as needed
        if abs(error) < orientation_deadzone:
            error = 0  # Ignore minor orientation errors within the deadzone
            
        #reduce proportional gain as the robot gets closer to the goal
        if d < 0.05:
            k_orientation = 0
            k_distance = 0
        elif d < 0.1:
            k_orientation *= d
            k_distance = 0.05

        # Calculate turning speed based on orientation error
        # Negative error implies turn right, positive error implies turn left
        turn_speed = k_orientation * error
        
        # Calculate forward speed based on distance to the goal
        # Optionally reduce speed as the robot approaches the goal
        forward_speed = min(max_speed, k_distance * d)
    
        # Calculate individual motor speeds
        # Left motor speed is reduced by turn_speed to turn left, increased to turn right
        # Right motor speed is increased by turn_speed to turn left, reduced to turn right
        left_motor_speed = max(-max_speed, min(max_speed, forward_speed - turn_speed))
        right_motor_speed = max(-max_speed, min(max_speed, forward_speed + turn_speed))
        
        return left_motor_speed, right_motor_speed
    
    def get_robot_coordinates(self):
        gps_value = self.gps.getValues()
        XR = gps_value[0]
        YR = gps_value[1]
        return XR, YR
    
    def get_attractive_force(self, XR, YR, XG, YG):
        #Calculate distance to goal
        d = math.sqrt(pow((XR-XG),2)+pow((YR-YG),2))
        #Calculate angle to goal
        theta = math.atan2((YG-YR),(XG-XR))
        return d, theta
    
    def run(self):
        max_speed = 6.28
        XG=-0.335 # Goal coordinates
        YG=-0.335
        
        while self.step(self.timestep) != -1:
            
            self.pen.write(True)
            
            XR, YR = self.get_robot_coordinates()
            d, theta = self.get_attractive_force(XR, YR, XG, YG)
            imu_value = self.imu.getRollPitchYaw()
            
            left_speed, right_speed = self.calculate_motor_speeds(theta, imu_value, d)

            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
                
controller = Controller()
controller.run()
