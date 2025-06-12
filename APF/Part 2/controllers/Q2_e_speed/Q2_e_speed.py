"""APF_Q2c controller."""
from controller import Robot
import math        

class Controller(Robot):
    timestep = 32
    XG = -0.355
    YG = -0.355
    
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
        
        #proximity sensors
        self.proximity_sensors = []
        sensor_names = ['ps' + str(i) for i in range(8)]
        for name in sensor_names:
            sensor = self.getDevice(name)
            sensor.enable(self.timestep)
            self.proximity_sensors.append(sensor)
            
    def get_robot_coordinates(self):
        gps_value = self.gps.getValues()
        XR = gps_value[0]
        YR = gps_value[1]
        return XR, YR

    def calculate_repulsion(self):
        sensor_orientations = [0.314, 0.785, 1.57, 2.62, -2.62, -1.57, -0.785, -0.314]
        repulsive_force_x = 0
        repulsive_force_y = 0
        for i, sensor in enumerate(self.proximity_sensors):
            reading = sensor.getValue()
            angle = sensor_orientations[i]
            if (reading > 75):
                 repulsive_force_x += math.cos(angle) * 3 
                 repulsive_force_y += math.sin(angle)* 3
            if (reading > 100):
                 repulsive_force_x += math.cos(angle) * 3.5
                 repulsive_force_y += math.sin(angle)* 3.5

        return repulsive_force_x, repulsive_force_y
        
    def calculate_attraction(self, XR, YR):
        d = math.sqrt(pow((math.fabs(XR-self.XG)),2)+pow((math.fabs(YR-self.YG)),2))
        theta = math.atan2((self.YG-YR),(self.XG-XR))
        attraction_x = math.cos(theta) * 6 #increase force magnitude for attraction
        attraction_y = math.sin(theta) * 6
        return d, attraction_x, attraction_y

    def calculate_motor_speeds(self, d, attract_force_x, attract_force_y, repulsive_force_x, repulsive_force_y):
        max_speed = 6.28  
        #Increasing contants for faster movement
        k_orientation = 1  # Proportional gain for orientation control
        k_distance = 1  # Proportional gain for distance control
        
        #Remove drop-ff function for distance
        #drop off function for k_distance
        #k_distance *= d
        
         # Calculate the magnitude of the repulsion force and adjust k_orientation accordingly
        repulsion_magnitude = math.sqrt(repulsive_force_x**2 + repulsive_force_y**2)
        repulsion_threshold = 4
        if repulsion_magnitude > repulsion_threshold:
            k_orientation += repulsion_magnitude #Will make robot turn away from obstacles faster
        
        #Completely stop moving the robot when close enough to the goal
        at_goal_threshold = 0.01  # 4cm radius within the goal
        if d < at_goal_threshold:
            return 0, 0
            
        # Summing up attractive and repulsive forces
        resultant_force_x = attract_force_x - repulsive_force_x
        resultant_force_y = attract_force_y - repulsive_force_y

        # Calculating the angle and magnitude of the resultant force
        resultant_angle = math.atan2(resultant_force_y, resultant_force_x)
        force_magnitude = math.sqrt(resultant_force_x**2 + resultant_force_y**2)
        imu_value = self.imu.getRollPitchYaw()
        if(abs(math.atan2(math.sin(resultant_angle - imu_value[2]), math.cos(resultant_angle - imu_value[2]))) >= math.pi):
            error = abs(math.atan2(math.sin(resultant_angle - imu_value[2]), math.cos(resultant_angle - imu_value[2])))
        else:
            error = math.atan2(math.sin(resultant_angle - imu_value[2]), math.cos(resultant_angle - imu_value[2]))
       
        #oreintation deadzone
        orientation_deadzone = 0.01  # Adjust this value as needed
        if abs(error) < orientation_deadzone:
            error = 0 
            
            
        # Adjusting the motor speeds based on the resultant vector
        turn_speed = k_orientation * error
        forward_speed = force_magnitude * k_distance 
        
        left_motor_speed = forward_speed - turn_speed
        right_motor_speed = forward_speed + turn_speed
        
        # Ensure the speeds are within bounds
        left_motor_speed = max(-max_speed, min(max_speed, left_motor_speed))
        right_motor_speed = max(-max_speed, min(max_speed, right_motor_speed))
        
        return left_motor_speed, right_motor_speed


    def run(self):

        while self.step(self.timestep) != -1:
            self.pen.write(True)
            
            XR, YR = self.get_robot_coordinates()

            repulsive_force_x, repulsive_force_y = self.calculate_repulsion()
            d, attract_force_x, attract_force_y = self.calculate_attraction(XR, YR)
            
            left_speed, right_speed = self.calculate_motor_speeds(d, attract_force_x, attract_force_y, repulsive_force_x, repulsive_force_y)
            self.left_motor.setVelocity(left_speed)
            self.right_motor.setVelocity(right_speed)
            
                
controller = Controller()
controller.run()
