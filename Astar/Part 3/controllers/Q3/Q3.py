###############
###############
# Kindly note that the A* algorithm we used and modified its functions according to our needs was from: https://rosettacode.org/wiki/A*_search_algorithm#:~:text=The%20A*%20search%20algorithm%20is,aka%20vertices)%20of%20a%20graph.

from __future__ import print_function
import math
import re

class AStarGraph(object):
    def __init__(self):
        self.waypoints = {
            'A': (-0.426, 0.426),
            'B': (-0.213, 0.497),
            'C': (-0.426, 0.213),
            'D': (-0.497, 0),
            'E': (-0.497, -0.213),
            'F': (-0.426, -0.426),
            'G': (-0.142, -0.426),
            'H': (0.142, -0.426),
            'I': (0.142, -0.142),
            'J': (-0.142, -0.071),
            'K': (-0.071, 0.071),
            'L': (-0.142, 0.213),
            'M': (0.142, 0.213),
            'N': (0.284, 0.426),
            'O': (0.426, 0.213),
            'P': (0.497, 0),
            'Q': (0.497, -0.284),
            'R': (0.426, -0.497),
        }
        self.connections = {
            'A': ['B', 'C'],
            'B': ['A'],
            'C': ['A', 'D', 'L'],
            'D': ['C','E'],
            'E': ['D','F'],
            'F': ['E','G'],
            'G': ['F','H'],
            'H': ['I', 'G', 'R'],
            'I': ['J', 'H'],
            'J': ['K', 'I'],
            'K': ['L', 'J'],
            'L': ['K', 'M', 'C'],
            'M': ['N','L', 'O'],
            'N': ['M','O'],
            'O': ['N','P', 'M'],
            'P': ['Q', 'O'],
            'Q': ['P','R'],
            'R': ['H', 'Q'],
        }

    def heuristic(self, current, goal):
        x1, y1 = self.waypoints[current]
        x2, y2 = self.waypoints[goal]
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_vertex_neighbours(self, pos):
        return self.connections[pos]

    def move_cost(self, a, b):
        x1, y1 = self.waypoints[a]
        x2, y2 = self.waypoints[b]
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return distance

def AStarSearch(start, end, graph):

	G = {} #Actual movement cost to each position from the start position
	F = {} #Estimated movement cost of start to end going via this position

	#Initialize starting values
	G[start] = 0
	F[start] = graph.heuristic(start, end)

	closedVertices = set()
	openVertices = set([start])
	cameFrom = {}

	while len(openVertices) > 0:
		#Get the vertex in the open list with the lowest F score
		current = None
		currentFscore = None
		for pos in openVertices:
			if current is None or F[pos] < currentFscore:
				currentFscore = F[pos]
				current = pos

		#Check if we have reached the goal
		if current == end:
			#Retrace our route backward
			path = [current]
			while current in cameFrom:
				current = cameFrom[current]
				path.append(current)
			path.reverse()
			return path, F[end] #Done!

		#Mark the current vertex as closed
		openVertices.remove(current)
		closedVertices.add(current)

		#Update scores for vertices near the current position
		for neighbour in graph.get_vertex_neighbours(current):
			if neighbour in closedVertices:
				continue #We have already processed this node exhaustively
			candidateG = G[current] + graph.move_cost(current, neighbour)

			if neighbour not in openVertices:
				openVertices.add(neighbour) #Discovered a new vertex
			elif candidateG >= G[neighbour]:
				continue #This G score is worse than previously found

			#Adopt this G score
			cameFrom[neighbour] = current
			G[neighbour] = candidateG
			H = graph.heuristic(neighbour, end)
			F[neighbour] = G[neighbour] + H

	raise RuntimeError("A* failed to find a solution")


    
def question_1e(start, goal, route, graph):
    steps = []
    steps.append(f"Start: ({start[0]}, {start[1]})")
    for i in range(len(route)):
        step = f"{route[i]}: ({graph.waypoints[route[i]][0]}, {graph.waypoints[route[i]][1]})"
        steps.append(step)
    
    steps.append(f"Goal: ({goal[0]}, {goal[1]})")
    return steps
    
def question_1f(start, goal, route, graph):
    steps = []
    steps.append(f"Step {1}: ({start[0]}, {start[1]})-{graph.waypoints[route[0]]}")
    for i in range(len(route) - 1):
        step = f"Step {i+2}: {graph.waypoints[route[i]]}-{graph.waypoints[route[i+1]]}"
        steps.append(step)
    
    steps.append(f"Step {len(route)+1}: {graph.waypoints[route[-1]]}-({goal[0]}, {goal[1]})")
    return steps


from controller import Robot
import math        

class Controller(Robot):
    timestep = 64
    
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
    
    def calculate_repulsion(self):
        repulsive_force_x = 0
        repulsive_force_y = 0
        
        threshold = 90
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
                
        yaw = self.imu.getRollPitchYaw()[2]
        global_repulsive_force_x = repulsive_force_x * math.cos(yaw) - repulsive_force_y * math.sin(yaw)
        global_repulsive_force_y = repulsive_force_x * math.sin(yaw) + repulsive_force_y * math.cos(yaw)

        return global_repulsive_force_x, global_repulsive_force_y    
        
        
    def calculate_attraction(self, XG, YG, XR, YR):
        d = math.sqrt(pow((math.fabs(XR-XG)),2)+pow((math.fabs(YR-YG)),2))
        theta = math.atan2((YG-YR),(XG-XR))
        attraction_x = math.cos(theta)*0.3#org 0.5
        attraction_y = math.sin(theta)*0.3
        return d, attraction_x, attraction_y
    
    def calculate_motor_speeds(self, d, attract_force_x, attract_force_y, repulsive_force_x, repulsive_force_y, XG, YG):
        max_speed = 6.28  # Maximum speed for each motor
        k_orientation = 0.3  # Proportional gain for orientation control
        k_distance = 0.25  # Proportional gain for distance control
        
        #drop off function for k_distance
        #k_distance *= d
        
        # Calculate the magnitude of the repulsion force and adjust k_orientation accordingly
        repulsion_magnitude = math.sqrt(repulsive_force_x**2 + repulsive_force_y**2)
        repulsion_threshold = 0.02
        if repulsion_magnitude > repulsion_threshold:
            k_orientation += 2*repulsion_magnitude #Will make robot turn away from obstacles faster
        
        #Completely stop moving the robot when close enough to the goal
        at_goal_threshold = 0.05  # 5cm radius within the goal
        if d < at_goal_threshold:
            print(f"Reached waypoint: {XG},{YG}")
            self.number_of_waypoint = self.number_of_waypoint + 1
            return 0, 0
            
        #print(f"Attraction Forces: ({attract_force_x}, {attract_force_y})")
        #print(f"Repulsion Forces: ({repulsive_force_x}, {repulsive_force_y})")
        # Summing up attractive and repulsive forces
        resultant_force_x = attract_force_x + repulsive_force_x
        resultant_force_y = attract_force_y + repulsive_force_y
       # print(f"Resultant Forces: ({resultant_force_x}, {resultant_force_y})\n\n")
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
        forward_speed = force_magnitude * k_distance  #Reduce speed as distance decreases
        
        left_motor_speed = forward_speed - turn_speed
        right_motor_speed = forward_speed + turn_speed
        
        # Ensure the speeds are within bounds
        left_motor_speed = max(-max_speed, min(max_speed, left_motor_speed))
        right_motor_speed = max(-max_speed, min(max_speed, right_motor_speed))
        #print(f"Motor Speeds: Left={left_motor_speed}, Right={right_motor_speed}")
        return left_motor_speed, right_motor_speed
    
    def run(self):
        not_done = 1
        while self.step(self.timestep) != -1:
           if not_done:
               gps_value = self.gps.getValues()
               while math.isnan(gps_value[0]):
                    gps_value = self.gps.getValues()
               not_done = 0
               astar_graph = AStarGraph()
               XR = gps_value[0]
               YR = gps_value[1]
               XG_f =  0.466
               YG_f =  -0.212
               start = (XR,YR)
               goal =(XG_f,YG_f) ### set the goal location here
               # the functions below to get the closest waypoint to start and goal
               first_waypoint = min(astar_graph.waypoints.keys(), key=lambda x: (start[0] - astar_graph.waypoints[x][0])**2 + (start[1] - astar_graph.waypoints[x][1])**2)
               last_waypoint = min(astar_graph.waypoints.keys(), key=lambda x: (goal[0] - astar_graph.waypoints[x][0])**2 + (goal[1] - astar_graph.waypoints[x][1])**2)
               result, cost = AStarSearch(first_waypoint, last_waypoint, astar_graph)
               full_path = ['Start'] + result + ['Goal']
               print("-------------------------------------------------------")
               print("Path from start to goal:", full_path)
               print("Cost (not considering start and goal):", cost)
               steps = question_1e(start, goal, result, astar_graph)
               for step in steps:
                   print(step)        
       
           self.pen.write(True)
            
           XR, YR = self.get_robot_coordinates()
           if self.number_of_waypoint < len(steps):
               step_info = steps[self.number_of_waypoint].split(":")[1].strip()
               pattern = re.compile(r"\(([^)]+)\)")
               match = pattern.search(step_info)
               if match:
                   coordinates = match.group(1).split(',')
                   XG = float(coordinates[0].strip())
                   YG = float(coordinates[1].strip())
           elif self.number_of_waypoint == len(steps):
               exit()

           repulsive_force_x, repulsive_force_y = self.calculate_repulsion()
           d, attract_force_x, attract_force_y = self.calculate_attraction(XG, YG, XR, YR)
            
           left_speed, right_speed = self.calculate_motor_speeds(d, attract_force_x, attract_force_y, repulsive_force_x, repulsive_force_y, XG, YG)
           self.left_motor.setVelocity(left_speed)
           self.right_motor.setVelocity(right_speed)

                
controller = Controller()
controller.run()

