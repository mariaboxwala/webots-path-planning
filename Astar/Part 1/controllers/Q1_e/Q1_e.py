###############
###############
# Kindly note that the A* algorithm we used and modified its functions according to our needs was from: https://rosettacode.org/wiki/A*_search_algorithm#:~:text=The%20A*%20search%20algorithm%20is,aka%20vertices)%20of%20a%20graph.

from __future__ import print_function
import math

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
    


from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())
 
astar_graph = AStarGraph()

start_goals = [
    ((-0.497, 0.5), (-0.107, -0.035)), #Required - Case 1
    ((0.23, 0.314), (-0.497, 0.5)), #case 2
    ((-0.456, -0.479), (-0.178, 0.47)), #case 3
    ((-0.465, -0.479),(0.178,0.47)), #case 4
    ((-0.480, 0.465),(0.477, -0.489)), #case 5
]

i = 1
for start, goal in start_goals:
    # the functions below to get the closest waypoint to start and goal
    first_waypoint = min(astar_graph.waypoints.keys(), key=lambda x: (start[0] - astar_graph.waypoints[x][0])**2 + (start[1] - astar_graph.waypoints[x][1])**2)
    last_waypoint = min(astar_graph.waypoints.keys(), key=lambda x: (goal[0] - astar_graph.waypoints[x][0])**2 + (goal[1] - astar_graph.waypoints[x][1])**2)
    result, cost = AStarSearch(first_waypoint, last_waypoint, astar_graph)
    full_path = ['Start'] + result + ['Goal']
    print("-------------------------------------------------------")
    print("Case ", i)
    print("Path from start to goal:", full_path)
    print("Cost (not considering start and goal):", cost)
    steps = question_1e(start, goal, result, astar_graph)
    for step in steps:
        print(step)        
    i = i+1



while robot.step(timestep) != -1:
    exit(0)


controller = Controller()
controller.run()
