"""Robot_Supervisor_Extended controller."""
import json
import random
import os
from controller import Supervisor 

#CONSTANTS DECLARATION ---------------------------------------------------------------------------------------------------------------------
# Robot parameters
robot_names = ["robot_1", "robot_2", "robot_3", "robot_4"]
robots = []
#Set whether we want random intial positions or fixed ones
RANDOM = False
translations = []
if RANDOM:
    for i in range(len(robot_names)):
        XR = random.uniform(-1,1)
        YR = random.uniform(-1,1)
        translations.append([XR, YR, 0.0])
else:
    translations = [[0.9, -0.45, 0.0],[-0.3,-0.35,0],[0.85,-0.15,0.0],[0.01,0.8,0.0]]

#File paths needed
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
filepath_rrt_data = (f"{parent_dir}\\rrt_star_data.json")
filepath_robot_data = (f"{parent_dir}\\robot_data.json")

#Colors
LIGHT_GRAY = 0x808080
RED = 0xBB2222
BLACK = 0x000000

#FUNCTIONS ----------------------------------------------------------------------------------------------------------------------------------
#Function to map coordinates from webots x-y plane to display plane
def map_coordinates(x, y):
    mapped_x = (x + 1) * 32 # Map x from (-1, 1) to (0, 64)
    mapped_y = (1 - y) * 32 # Map y from (1, -1) to (0, 64)
    return mapped_x, mapped_y

#SUPERVISOR CONTROLLER ----------------------------------------------------------------------------------------------------------------------
class Controller(Supervisor):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = 64
        self.ground_display = self.getDevice('display')
        
    # Load nodes and edges from the file
    def load_rrt_star_data(self):
        with open((filepath_rrt_data), "r") as file:
            data = json.load(file)
            self.nodes = data["nodes"]
            self.edges = data["edges"]
     
    #Function to draw every edge       
    def draw_edges(self):
        self.ground_display.setColor(BLACK) #Black
        for edge in self.edges:
            x1, y1 = edge[0][0], edge[0][1] 
            x2, y2 = edge[1][0], edge[1][1] 
            mapped_x1, mapped_y1 = map_coordinates(x1, y1)
            mapped_x2, mapped_y2 = map_coordinates(x2, y2)
            self.ground_display.drawLine(mapped_x1, mapped_y1, mapped_x2, mapped_y2)
    
    #Function to draw every node  
    def draw_nodes(self):
        self.ground_display.setColor(RED) #Red
        for node in self.nodes:
            x, y = node 
            mapped_x, mapped_y = map_coordinates(x, y) 
            self.ground_display.drawPixel(mapped_x, mapped_y) 
            
    def run(self):  
        #Initilize the display
        GROUND_X = 1.0
        GROUND_Y = 1.0

        width = self.ground_display.getWidth()
        height = self.ground_display.getHeight()
       
        self.ground_display.setColor(LIGHT_GRAY)
        self.ground_display.fillRectangle(0, 0, width, height)
             
        # Initialize robots start positions
        for name in robot_names:
            robot = self.getFromDef(name)
            if robot is not None:
                robots.append(robot)
        for i in range(len(robots)):
            translation_field = robots[i].getField('translation')
            translation_field.setSFVec3f(translations[i])
    
        #Main Loop
        while self.step(self.timeStep)!= -1:
            #Constantly update json file with robot positions
            for i in range(len(robots)):
                translation_field = robots[i].getField('translation')
                translations[i] = translation_field.getSFVec3f()
            with open(filepath_robot_data, "w") as file:
                json.dump(translations, file)
                
            #Constantly update RRT* graph on the display
            self.load_rrt_star_data()
            self.ground_display.setColor(LIGHT_GRAY)
            self.ground_display.fillRectangle(0, 0, width, height)
            self.draw_edges()
            self.draw_nodes()

controller = Controller()
controller.run()