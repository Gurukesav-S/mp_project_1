#DO NOT MODIFY FOR HERE
import random
from queue import PriorityQueue, Queue
import numpy as np
import matplotlib.pyplot as plt

###############################        NOTES           ############################################################
#1. Do not import an additional packages, read about the imported packages and learn to use them to solve to problem: Queue and PriorityQueue are the packages to use
#2. Stack is readily available data type in python if you need to use it
#3. Only when you run the code you will see GUI output, however when evaluating it will not show GUI, it will show path calcualted by your algorithm and corrct path if your path is not correct.
#4. You can add any number of additional functions that you can call from already defined functions below
#5. DO NOT MODIFY name, arguments or the return type othrwise your code will not get auto evaluated

#*HINT: may be satrt from UCS implemntatio from the last assignment

#################################################################################################################################

#This fuction is not mandetory to use, you can consider unit edge cost for all the actions/nodes, but if you want to consider non-uniform edge cost you can call this function
#Arguments:
# 1. costs: contains cost between a given node and its neighbor
# 2. node: this is the coordInate of the current node givn as [x,y]
# 3. neighbor: this is the coordinate of the neighbor node givn as [x,y]

# Returns: cost to go from the node to the neighbour

def get_edge_cost(costs, node, neighbor):
    nx, ny = node
    return costs[nx, ny]

#################################################################################################################################

#This function returns all valid neighbours given the maze and the node

#Arguments:
# 1. maze: this is the search space of size X (rows) x Y (columns), its a numpy array of size X rows and Y columns
# 2. node: this is the coordinate of the current node givn as [x,y]

# Returns: all the neighbors to be added to the Queue or Prioity queue

#Note: Put the neighbours in the Stack/Queue in same order as they are returned from this function while maintaining the correct priority
def get_neighbors(maze, node):
    x, y = node
    neighbors = []
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < maze.shape[0] and 0 <= ny < maze.shape[1] and maze[nx, ny] == 0:
            neighbors.append((nx, ny))
    return neighbors

################################################################################################################################# TO HERE

#MODIFY CONTENT FROM THIS LINE ONWARDS HOWEVER DO NOT MODIFY FUNCTION NAMES ARGUMENTS OR RETURN TYPE

#This function returns manhattan distance between the goal state and the current state

#Arguments:
# 1. goal: goal state as provided to the function "a_star" below
# 2. node: this is the coordinate of the current node givn as [x,y] for which you want to calcuate the manhattan heuristic

# Returns: manhattan distance between the goal and the node passed as arguments
def manhattan_heuristice(goal, node):
    distance = 0 #update this variable and return the calcuated/updated value

    "*** YOUR CODE HERE ***"
    distance = abs(goal[0]-node[0])+abs(goal[1]-node[1])
    return distance

#################################################################################################################################

#This function returns euclidean distance between the goal state and the current state

#Arguments:
# 1. goal: goal state as provided to the function "a_star" below
# 2. node: this is the coordinate of the current node givn as [x,y] for which you want to calcuate the euclidean heuristic

# Returns: euclidean distance between the goal and the node passed as arguments
def euclidean_heuristic(goal, node):
    distance = 0 #update this variable and return the calcuated/updated value

    "*** YOUR CODE HERE ***"
    distance = np.sqrt((goal[0]-node[0])**2+(goal[1]-node[1])**2)
    return distance

#This function should implement a_star algorithm with 3 different heurstics, the choice of the heuristic is passed as an argument

#Arguments:
# 1. maze: this is the search space of size X (rows) x Y (columns), its a numpy array of size X rows and Y columns
# 2. start: coordinate of the start node givn as [x,y]
# 3. goal: coordinate of the goal node givn as [x,y]
# 4. costs: if you choose to use non-uniform dge costs you can call get_edge_cost() with appropriate arguments just like how you used it in assignmet 1 UCS implementation
# 5. heuristic: if heuristic=0 use Manhattan Distance heuristic
#               if heuristic=1 use Euclidean Distance heuristic
#               if heuristic>=2 use weighted heuristic with manhattan distance, so calcuated heuristic value should be "heuristic * manhattan_heuristice(goal, node)".
# Return values
# 1. a list of nodes containng path from START to GOAL, rememebr the first node in this list should be START and the last one should be GOAL

def a_star(maze, start, goal, costs, heuristic):
    path = [] #this should contai list of nodes [start, (20,30), (21,30), ...., goal] as a path from start to goal
    "*** YOUR CODE HERE ***"
    fringe = PriorityQueue()
    closed_set = set()
    parent_dict = {start: None}
    g_cost = {start:0}
    h_start = get_heuristic(goal,start,heuristic)
    fringe.put((h_start,0,start))   #(g+h,g,node)
    
    while not fringe.empty():
        current_f,current_g,current_node = fringe.get()
        
        if current_node == goal:
            curr = goal
            while curr is not None:
                path.append(curr)
                curr = parent_dict[curr]
            path.reverse()
            return path, len(closed_set)
            
        # if current_node in closed_set:
        #     continue
        if current_g>g_cost[current_node]:
            continue
        closed_set.add(current_node)
        neighbours = get_neighbors(maze, current_node)
        
        for next_node in neighbours:
            # if next_node in closed_set:
            #     continue
            
            edge_cost = 1 #get_edge_cost(costs,current_node,next_node)
            new_g = g_cost[current_node] + edge_cost
            
            if next_node not in g_cost or new_g < g_cost[next_node]:
                g_cost[next_node] = new_g
                parent_dict[next_node] = current_node
                h_cost = get_heuristic(goal,next_node,heuristic)
                f_cost = new_g +h_cost
                fringe.put((f_cost,new_g,next_node))   
        
                
    return path, len(closed_set)
################################################################################################################################# 
#Any oher functions you may want to define
def get_heuristic(goal,node,heuristic):
    if heuristic == 0 :
        return manhattan_heuristice(goal,node)
    elif heuristic == 1:
        return euclidean_heuristic(goal,node)
    elif heuristic >=2:
        return heuristic * manhattan_heuristice(goal,node)

#######################################
#Improved D* Lite 

def get_improved_neighbors(maze, node):
    """
    Increases moveable directions by allowing paths through 
    grid boundaries (midpoints) and vertices[cite: 561, 568].
    """
    x, y = node
    neighbors = []
    # 8-way connectivity (Vertices) [cite: 559]
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0: continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < maze.shape[0] and 0 <= ny < maze.shape[1] and maze[int(nx), int(ny)] == 0:
                neighbors.append((nx, ny))
    
    # Boundary Midpoints (Interpolation points) [cite: 562, 567]
    for dx, dy in [(-0.5, 0), (0.5, 0), (0, -0.5), (0, 0.5)]:
        nx, ny = x + dx, y + dy
        if 0 <= nx < maze.shape[0] and 0 <= ny < maze.shape[1]:
            # Boundary is valid if the two cells sharing it are free [cite: 225]
            if maze[int(nx-0.5), int(ny)] == 0 and maze[int(nx+0.5), int(ny)] == 0:
                neighbors.append((nx, ny))
    return neighbors

def is_valid_turn(pred, curr, succ, max_angle=60):
    """
    Equation 14: Restricts the turning angle to avoid unfeasible paths[cite: 550, 554].
    """
    if pred is None: return True
    v1 = np.array([curr[0] - pred[0], curr[1] - pred[1]])
    v2 = np.array([succ[0] - curr[0], succ[1] - curr[1]])
    
    norm1, norm2 = np.linalg.norm(v1), np.linalg.norm(v2)
    if norm1 == 0 or norm2 == 0: return True
    
    cos_phi = np.dot(v1, v2) / (norm1 * norm2)
    phi = np.degrees(np.arccos(np.clip(cos_phi, -1.0, 1.0)))
    return phi <= max_angle #[cite: 554]

def get_idw_heuristic(goal, node, heuristic_type):
    """
    Equation 15 & 16: Calculates costs for boundary midpoints[cite: 573, 579].
    """
    if float(node[0]).is_integer() and float(node[1]).is_integer():
        return get_heuristic(goal, node, heuristic_type)
    
    # Find 4 nearest integer vertices [cite: 571]
    x_b, y_b = int(np.floor(node[0])), int(np.floor(node[1]))
    vertices = [(x_b, y_b), (x_b+1, y_b), (x_b, y_b+1), (x_b+1, y_b+1)]
    
    w_sum, total_w = 0, 0
    p = 2 # Optimal power parameter [cite: 577]
    for v in vertices:
        d = np.sqrt((node[0]-v[0])**2 + (node[1]-v[1])**2)
        if d == 0: return get_heuristic(goal, v, heuristic_type)
        weight = 1 / (d**p) #[cite: 573]
        w_sum += weight * get_heuristic(goal, v, heuristic_type) #[cite: 579]
        total_w += weight
    return w_sum / total_w #[cite: 579]

def i_dstar_lite(maze, start, goal, costs, heuristic):
    """
    Final Improved D* Lite implementation[cite: 108].
    Combines path cost optimization, IDW, and motion constraints[cite: 112, 113].
    """
    path = []
    fringe = PriorityQueue()
    g_cost = {start: 0}
    parent_dict = {start: None}
    closed_set = set()

    h_start = get_idw_heuristic(goal, start, heuristic)
    fringe.put((h_start, 0, start))

    while not fringe.empty():
        f, g, current = fringe.get()

        if current == goal:
            curr = goal
            while curr is not None:
                path.append(curr)
                curr = parent_dict.get(curr)
            path.reverse()
            return path, len(closed_set)

        if g > g_cost.get(current, float('inf')):
            continue
        
        closed_set.add(current)
        for neighbor in get_improved_neighbors(maze, current):
            # Check motion constraints 
            if not is_valid_turn(parent_dict.get(current), current, neighbor):
                continue

            edge_cost = get_improved_edge_cost(current, neighbor) #[cite: 379]
            new_g = g + edge_cost

            if neighbor not in g_cost or new_g < g_cost[neighbor]:
                g_cost[neighbor] = new_g
                parent_dict[neighbor] = current
                h = get_idw_heuristic(goal, neighbor, heuristic) #[cite: 579]
                fringe.put((new_g + h, new_g, neighbor))

    return path, len(closed_set)

#############################
def get_8way_neighbors(maze, node):
    """
    Implements 8-way connectivity (Horizontal, Vertical, and Diagonal).
    Standard D* Lite often uses 4-way; the paper expands this to improve length.
    """
    x, y = int(node[0]), int(node[1])
    neighbors = []
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:
            if dx == 0 and dy == 0: continue
            nx, ny = x + dx, y + dy
            if 0 <= nx < maze.shape[0] and 0 <= ny < maze.shape[1]:
                if maze[nx, ny] == 0: # 0 is assumed to be free space
                    neighbors.append((nx, ny))
    return neighbors

def get_improved_edge_cost(node, neighbor):
    """
    Equation 13 from the paper:
    Cost = sqrt(2) * min(dx, dy) + |dx - dy|
    This allows the algorithm to differentiate between straight and diagonal steps.
    """
    dx = abs(node[0] - neighbor[0])
    dy = abs(node[1] - neighbor[1])
    return (np.sqrt(2) * min(dx, dy)) + (max(dx, dy) - min(dx, dy))

def i_dstar_lite_2(maze, start, goal, costs, heuristic):
    """
    Improved D* Lite core using the paper's cost function.
    """
    path = []
    fringe = PriorityQueue()
    g_cost = {start: 0}
    parent_dict = {start: None}
    closed_set = set()

    h_start = get_heuristic(goal, start, heuristic)
    fringe.put((h_start, 0, start))

    while not fringe.empty():
        f, g, current = fringe.get()

        if current == goal:
            curr = goal
            while curr is not None:
                path.append(curr)
                curr = parent_dict.get(curr)
            path.reverse()
            return path, len(closed_set)

        if g > g_cost.get(current, float('inf')):
            continue
        
        closed_set.add(current)
        
        for neighbor in get_8way_neighbors(maze, current):
            # Using Equation 13 instead of uniform costs
            edge_cost = get_improved_edge_cost(current, neighbor)
            new_g = g + edge_cost

            if neighbor not in g_cost or new_g < g_cost[neighbor]:
                g_cost[neighbor] = new_g
                parent_dict[neighbor] = current
                h = get_heuristic(goal, neighbor, heuristic)
                fringe.put((new_g + h, new_g, neighbor))

    return path, len(closed_set)