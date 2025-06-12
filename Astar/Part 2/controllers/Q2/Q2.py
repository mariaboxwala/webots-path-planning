"""APF_Q2c controller."""
from controller import Robot
import math        

class Controller(Robot):
    timestep = 64
    XG = -0.490
    YG = -0.490
    
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
        
        self.number_of_waypoint = 0
        
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
    
    def get_smoothed_sensor_value(self, sensor):
        readings = [sensor.getValue() for _ in range(5)]  # Take 5 readings
        return sum(readings) / len(readings)  # Return the average
               
    def calculate_repulsion(self):
        repulsive_force_x = 0
        repulsive_force_y = 0
        
        threshold = 80
        strength = 0.002  # Lowered strength to manage sensitivity
        
        reading_front_left = self.proximity_sensors[7].getValue()
        reading_front_right = self.proximity_sensors[0].getValue()
        reading_right = self.proximity_sensors[1].getValue()
        reading_left = self.proximity_sensors[6].getValue()

        # Calculate symmetric repulsion from front sensors
        if reading_front_left > threshold or reading_front_right > threshold:
            repulsive_force_x -= strength * (reading_front_left + reading_front_right)

        # Handle side sensors with additional checks for front-sensor activation
        if reading_right > threshold:
            if reading_front_left > threshold or reading_front_right > threshold:
                repulsive_force_y += strength * reading_right * 0.5  # Reduce influence when front is also blocked
            else:
                repulsive_force_y += strength * reading_right
        if reading_left > threshold:
            if reading_front_left > threshold or reading_front_right > threshold:
                repulsive_force_y -= strength * reading_left * 0.5  # Reduce influence when front is also blocked
            else:
                repulsive_force_y -= strength * reading_left

        # Rotate to global coordinate system
        yaw = self.imu.getRollPitchYaw()[2]
        global_repulsive_force_x = repulsive_force_x * math.cos(yaw) - repulsive_force_y * math.sin(yaw)
        global_repulsive_force_y = repulsive_force_x * math.sin(yaw) + repulsive_force_y * math.cos(yaw)

        return global_repulsive_force_x, global_repulsive_force_y
        
    def calculate_attraction(self, XR, YR):
        d = math.sqrt(pow((math.fabs(XR-self.XG)),2)+pow((math.fabs(YR-self.YG)),2))
        theta = math.atan2((self.YG-YR),(self.XG-XR))
        attraction_x = math.cos(theta)*0.3#org 0.5
        attraction_y = math.sin(theta)*0.3
        return d, attraction_x, attraction_y

    def calculate_motor_speeds(self, d, attract_force_x, attract_force_y, repulsive_force_x, repulsive_force_y):
        max_speed = 6.28  # Maximum speed for each motor
        k_orientation = 0.3  # Proportional gain for orientation control
        k_distance = 0.25  # Proportional gain for distance control
        
        #drop off function for k_distance
        #k_distance *= d
        
        # Calculate the magnitude of the repulsion force and adjust k_orientation accordingly
        repulsion_magnitude = math.sqrt(repulsive_force_x**2 + repulsive_force_y**2)
        repulsion_threshold = 0.02
        if repulsion_magnitude > repulsion_threshold:
           k_orientation += 1.5*repulsion_magnitude #Will make robot turn away from obstacles faster
        
        #Completely stop moving the robot when close enough to the goal
        at_goal_threshold = 0.05  # 5cm radius within the goal
        if d < at_goal_threshold:
            return 0, 0
        
        # Summing up attractive and repulsive forces
        resultant_force_x = attract_force_x + repulsive_force_x
        resultant_force_y = attract_force_y + repulsive_force_y

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
        turn_speed = k_orientation * error * 2
        forward_speed = force_magnitude * k_distance   #Reduce speed as distance decreases
        
        left_motor_speed = forward_speed - turn_speed
        right_motor_speed = forward_speed + turn_speed
        
        # Ensure the speeds are within bounds
        left_motor_speed = max(-max_speed, min(max_speed, left_motor_speed))
        right_motor_speed = max(-max_speed, min(max_speed, right_motor_speed))

        return left_motor_speed, right_motor_speed

    def run(self):
        XR, YR = self.get_robot_coordinates()
        
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