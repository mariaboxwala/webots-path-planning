import random
import math
import json
import os
from math import sqrt, cos, sin, atan2, fabs
import matplotlib.pyplot as plt
from controller import Robot, Motor, GPS, InertialUnit, DistanceSensor

#CONSTANTS DECLARATION ----------------------------------------------------------------------------------------------------------------
#X and Y dimensions, they are halved because we go from coordinates -1 to 1
XDIM = 1
YDIM = 1
EPSILON = 0.1  # Appropriate step size for a 5m by 5m area
NUMNODES = 200 #Number of nodes to be generated per each RRT* graph
RADIUS = 0.25  # Appropriate radius for connecting nodes
# Example obstacle (bottom-left x, bottom-left y, width, height)
OBS = [(0.1, 0, 0.1, 1.1), (0.321, -0.373, 0.8, 0.1), (-0.18, -0.996, 0.1, 0.8)] 
ROBOT_RADIUS = 0.05 #Slightly enlarged robot radius to calculate the robot as an obstacle in update obstacles

BIAS_PROB = 0.8 #Used for heurisitc - 80% chance it will generate a node near the goal

GRAPHS = 0 #Used to make filenames for grpahs

#file paths needed for JSON filea and graph storage
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
filepath_rrt_data = (f"{parent_dir}\\rrt_star_data.json")
filepath_robot_data = (f"{parent_dir}\\robot_data.json")
filepath_graphs = (f"{parent_dir}\\graphs")
#NODE CLASS --------------------------------------------------------------------------------------------------------------------------------
class Node:
    def __init__(self, xcoord, ycoord):
        self.x = xcoord
        self.y = ycoord
        self.cost = 0
        self.parent = None


#FUNCTIONS ---------------------------------------------------------------------------------------------------------------------------------  
def read_robot_positions():
    try:
        robot_positions = []
        with open(filepath_robot_data, "r") as file:
            data = json.load(file)
            for robot in data: 
                robot_positions.append(robot)
            return robot_positions
    except (json.JSONDecodeError, FileNotFoundError) as e:
        print(f"Error reading JSON file: {e}")
        # Return an empty list if there is an error reading the JSON file
        return []
    
def update_obstacles_with_robots(obstacles, robot_positions, robot_radius):
    updated_obstacles = obstacles[:]
    for pos in robot_positions:
        x, y, z = pos
        # Create an obstacle as a square around the robot's position
        updated_obstacles.append((x - robot_radius, y - robot_radius, robot_radius * 2, robot_radius * 2))
    return updated_obstacles

def write_nodes_edges(nodes, edges):
    # Write the newly generated nodes to the JSON file for the supervisor to access
    # Data needs to be serialized as node type cannot be written to a JSON file
    nodes_serializable = [(node.x, node.y) for node in nodes]
    edges_serializable = [((edge[0], edge[1]), (edge[2], edge[3])) for edge in edges]
    # Combining the nodes and edges into one structure
    data = {
        "nodes": nodes_serializable,
        "edges": edges_serializable
    }
    # Writing data to the JSON file
    with open((filepath_rrt_data), "w") as file:
        json.dump(data, file)
    visualize_rrt_star(nodes_serializable,edges)

def visualize_rrt_star(nodes, edges):
    global GRAPHS
    filename = (f"{filepath_graphs}\\RRT_star_{GRAPHS}.png")
    # Plot edges
    for edge in edges:
        plt.plot([edge[0], edge[2]], [edge[1], edge[3]], color='cyan', linestyle='-', linewidth=1)
    # Plot root
    if nodes:
        plt.scatter(nodes[0][0], nodes[0][1], color='red', label='Start node')
    # Plot the rest of the nodes in blue
    if len(nodes) > 1:
        plt.scatter([point[0] for point in nodes[1:]], [point[1] for point in nodes[1:]], color='blue', label='Nodes')
    # Plot obstacles
    robot_positions = read_robot_positions()
    updated_OBS = update_obstacles_with_robots(OBS, robot_positions, ROBOT_RADIUS)
    
    for obs in updated_OBS:
        rect = plt.Rectangle((obs[0], obs[1]), obs[2], obs[3], color='black', alpha=0.5)
        plt.gca().add_patch(rect)

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('RRT* Path and Edges')
    plt.legend()
    plt.grid(True)
    plt.savefig(filename)
    plt.clf()
    
    GRAPHS += 1

#RRT* ALGORITHM -------------------------------------------------------------------------------------------------------------------------
def ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

def checkIntersect(nodeA, nodeB, OBS):
    A = (nodeA.x, nodeA.y)
    B = (nodeB.x, nodeB.y)
    for o in OBS:
        obs = (o[0], o[1], o[0] + o[2], o[1] + o[3])
        # Convert obstacle coordinates to absolute values
        obs_abs = (min(obs[0], obs[2]), min(obs[1], obs[3]), max(obs[0], obs[2]), max(obs[1], obs[3]))
        C1 = (obs_abs[0], obs_abs[1])
        D1 = (obs_abs[0], obs_abs[3])
        C2 = (obs_abs[0], obs_abs[1])
        D2 = (obs_abs[2], obs_abs[1])
        C3 = (obs_abs[2], obs_abs[3])
        D3 = (obs_abs[2], obs_abs[1])
        C4 = (obs_abs[2], obs_abs[3])
        D4 = (obs_abs[0], obs_abs[3])
        inst1 = ccw(A, C1, D1) != ccw(B, C1, D1) and ccw(A, B, C1) != ccw(A, B, D1)
        inst2 = ccw(A, C2, D2) != ccw(B, C2, D2) and ccw(A, B, C2) != ccw(A, B, D2)
        inst3 = ccw(A, C3, D3) != ccw(B, C3, D3) and ccw(A, B, C3) != ccw(A, B, D3)
        inst4 = ccw(A, C4, D4) != ccw(B, C4, D4) and ccw(A, B, C4) != ccw(A, B, D4)
        if not (inst1 or inst2 or inst3 or inst4):
            continue
        else:
            return False
    return True

def dist(p1, p2):
    return sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def step_from_to(p1, p2):
    if dist(p1, p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
        return p1[0] + EPSILON * cos(theta), p1[1] + EPSILON * sin(theta)

def chooseParent(nn, newnode, nodes):
    for p in nodes:
        if checkIntersect(p, newnode, OBS) and dist([p.x, p.y], [newnode.x, newnode.y]) < RADIUS and \
                p.cost + dist([p.x, p.y], [newnode.x, newnode.y]) < nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y]):
            nn = p
    newnode.cost = nn.cost + dist([nn.x, nn.y], [newnode.x, newnode.y])
    newnode.parent = nn
    return newnode, nn

def reWire(nodes, newnode):
    for i in range(len(nodes)):
        p = nodes[i]
        if checkIntersect(p, newnode, OBS) and p != newnode.parent and dist([p.x, p.y], [newnode.x, newnode.y]) < RADIUS and \
                newnode.cost + dist([p.x, p.y], [newnode.x, newnode.y]) < p.cost:
            p.parent = newnode
            p.cost = newnode.cost + dist([p.x, p.y], [newnode.x, newnode.y])
            nodes[i] = p
    return nodes

def drawSolutionPath(start, goal, nodes):
    waypoints = []
    nn = nodes[0]
    for p in nodes:
        if dist([p.x, p.y], [goal.x, goal.y]) < dist([nn.x, nn.y], [goal.x, goal.y]):
            nn = p
    while nn != start:
        waypoints.append((nn.x, nn.y))
        nn = nn.parent
    waypoints.append((start.x, start.y))
    waypoints.reverse()
    return waypoints

def generate_rrt_star_path(start_x, start_y, goal_x, goal_y):
    nodes = []
    edges = []  # List to store edges (parent_x, parent_y, child_x, child_y)
    nodes.append(Node(start_x, start_y))  # Start in the specified position
    start = nodes[0]
    goal = Node(goal_x, goal_y)  # Goal at the specified position
    
    for i in range(NUMNODES):
        #Our Added Heuristic ---------------------------------------------------------
        std_dev = 0.2  # Adjust this value to control the spread of the distribution
        # Generate random numbers from a Gaussian distribution with mean as goal
        rand_x = random.gauss(goal_x, std_dev)
        rand_y = random.gauss(goal_y, std_dev)
        # Create the random node using the sampled coordinates
        if random.random() < BIAS_PROB:
            rand = Node(rand_x, rand_y) 
        else:   
            rand = Node(random.uniform(-XDIM, XDIM), random.uniform(-YDIM, YDIM))
        #----------------------------------------------------------------------------
        
        nn = nodes[0]
        for p in nodes:
            if dist([p.x, p.y], [rand.x, rand.y]) < dist([nn.x, nn.y], [rand.x, rand.y]):
                nn = p
        interpolatedNode = step_from_to([nn.x, nn.y], [rand.x, rand.y])
        newnode = Node(interpolatedNode[0], interpolatedNode[1])
        
        robot_positions = read_robot_positions()
        updated_OBS = update_obstacles_with_robots(OBS, robot_positions, ROBOT_RADIUS)
        
        if checkIntersect(nn, rand, updated_OBS):
            newnode, nn = chooseParent(nn, newnode, nodes)
            nodes.append(newnode)
            edges.append((nn.x, nn.y, newnode.x, newnode.y))  # Store the edge as tuples of coordinates
            nodes = reWire(nodes, newnode)

    #Write to JSON file
    write_nodes_edges(nodes, edges)
    waypoints = drawSolutionPath(start, goal, nodes)
    
    return waypoints, edges


#ROBOT CONTROLLER -----------------------------------------------------------------------------------------------------------------------
# Robot Controller with APF model from Project 2
class RobotController:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Initialize devices
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)
        
        self.imu = self.robot.getDevice('inertial unit')
        self.imu.enable(self.timestep)
        
        self.proximity_sensors = []
        for i in range(8):
            sensor = self.robot.getDevice(f'ps{i}')
            sensor.enable(self.timestep)
            self.proximity_sensors.append(sensor)
        
        self.goal_x = 0.5
        self.goal_y = 0.5
        self.number_of_waypoints = 0
        self.replan_interval = 50 # Replan every 100 timesteps
        self.replan_counter = 0
    
    #APF MODEL ----------------------------------------------------------------------------------------------------------------------------
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
        global_repulsive_force_x = repulsive_force_x * cos(yaw) - repulsive_force_y * sin(yaw)
        global_repulsive_force_y = repulsive_force_x * sin(yaw) + repulsive_force_y * cos(yaw)

        return global_repulsive_force_x, global_repulsive_force_y    
    
    def calculate_attraction(self, XG, YG, XR, YR):
        d = sqrt(pow(fabs(XR-XG), 2) + pow(fabs(YR-YG), 2))
        theta = atan2(YG-YR, XG-XR)
        attraction_x = cos(theta) * 0.3  # org 0.5
        attraction_y = sin(theta) * 0.3
        return d, attraction_x, attraction_y
    
    def calculate_motor_speeds(self, d, attract_force_x, attract_force_y, repulsive_force_x, repulsive_force_y, XG, YG):
        max_speed = 6.28  # Maximum speed for each motor
        k_orientation = 0.3  # Proportional gain for orientation control
        k_distance = 1  # Proportional gain for distance control
        
        # Calculate the magnitude of the repulsion force and adjust k_orientation accordingly
        repulsion_magnitude = sqrt(repulsive_force_x**2 + repulsive_force_y**2)
        repulsion_threshold = 0.02
        if repulsion_magnitude > repulsion_threshold:
            k_orientation += 2 * repulsion_magnitude  # Will make robot turn away from obstacles faster
        
        # Completely stop moving the robot when close enough to the goal
        at_goal_threshold = 0.05  # 5cm radius within the goal
        if d < at_goal_threshold:
            #print(f"Reached waypoint: {XG},{YG}")
            self.number_of_waypoints += 1
            return 0, 0
        
        # Summing up attractive and repulsive forces
        resultant_force_x = attract_force_x + repulsive_force_x
        resultant_force_y = attract_force_y + repulsive_force_y

        # Calculating the angle and magnitude of the resultant force
        resultant_angle = atan2(resultant_force_y, resultant_force_x)
        force_magnitude = sqrt(resultant_force_x**2 + resultant_force_y**2)
        
        imu_value = self.imu.getRollPitchYaw()
        error = atan2(sin(resultant_angle - imu_value[2]), cos(resultant_angle - imu_value[2]))

        # Orientation deadzone
        orientation_deadzone = 0.01  # Adjust this value as needed
        if abs(error) < orientation_deadzone:
            error = 0 
            
        # Adjusting the motor speeds based on the resultant vector
        turn_speed = k_orientation * error * 2
        forward_speed = force_magnitude * k_distance  # Reduce speed as distance decreases
        
        left_motor_speed = forward_speed - turn_speed
        right_motor_speed = forward_speed + turn_speed
        
        # Ensure the speeds are within bounds
        left_motor_speed = max(-max_speed, min(max_speed, left_motor_speed))
        right_motor_speed = max(-max_speed, min(max_speed, right_motor_speed))

        return left_motor_speed, right_motor_speed
    
    def run(self):
        # Wait for GPS to get a valid initial position
        while self.robot.step(self.timestep) != -1:
            gps_value = self.gps.getValues()
            if not (math.isnan(gps_value[0]) or math.isnan(gps_value[1])):
                start_x, start_y = gps_value[0], gps_value[1]
                break
        
        # Initial planning
        waypoints, edges = generate_rrt_star_path(start_x, start_y, self.goal_x, self.goal_y)
        
        # MAIN LOOP -------------------------------------------------------------------------------------------------------------------------
        waypoint_index = 0
        while self.robot.step(self.timestep) != -1:
            #If we have reached the interval then re-generate the RRT* graph with information
            if self.replan_counter >= self.replan_interval:
                # Replan the path
                current_x, current_y = self.get_robot_coordinates()
                waypoints, edges = generate_rrt_star_path(current_x, current_y, self.goal_x, self.goal_y)
                waypoint_index = 0  # Restart waypoint index after replanning
                self.replan_counter = 0  # Reset replan counter
            
            #If we have not reached our goal, then continue to move toward it
            if waypoint_index < len(waypoints):
                target_x, target_y = waypoints[waypoint_index]
                current_x, current_y = self.get_robot_coordinates()
                
                # Calculate APF forces
                d, AFx, AFy = self.calculate_attraction(target_x, target_y, current_x, current_y)
                RFx, RFy = self.calculate_repulsion()
                
                # Calculate motor speeds
                left_speed, right_speed = self.calculate_motor_speeds(d, AFx, AFy, RFx, RFy, target_x, target_y)
                
                # Set motor speeds
                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)
                
                # Check if the waypoint is reached
                if sqrt((target_x - current_x)**2 + (target_y - current_y)**2) < 0.05:  # 5cm tolerance
                    waypoint_index += 1
            else:
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
                break
            
            self.replan_counter += 1

# Create and run the robot controller
controller = RobotController()
controller.run()