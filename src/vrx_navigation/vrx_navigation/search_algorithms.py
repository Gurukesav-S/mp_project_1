import heapq
import numpy as np
import math

INF = float('inf')

def heuristic(p1, p2):
    """Euclidean distance as used in the paper for accurate evaluation."""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def get_8way_neighbors(maze, node):
    neighbors = []
    r, c = node
    rows, cols = maze.shape
    for dr in [-1, 0, 1]:
        for dc in [-1, 0, 1]:
            if dr == 0 and dc == 0: continue
            nr, nc = r + dr, c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                neighbors.append((nr, nc))
    return neighbors

def improved_cost(maze, u, v):
    """Reflects Equation (13): 1.0 for straight, 1.414 for diagonal moves."""
    if maze[u] == 1 or maze[v] == 1:
        return INF
    dr = abs(u[0] - v[0])
    dc = abs(u[1] - v[1])
    return math.sqrt(2) * min(dr, dc) + abs(dr - dc)

def calculate_angle(s_pred, s_curr, s_succ):
    """Reflects Equation (14) to limit expansion based on turning angle."""
    if s_pred is None or s_pred == s_curr: return 0.0
    v1 = (s_curr[0] - s_pred[0], s_curr[1] - s_pred[1])
    v2 = (s_succ[0] - s_curr[0], s_succ[1] - s_curr[1])
    dot = v1[0]*v2[0] + v1[1]*v2[1]
    mag1 = math.hypot(*v1)
    mag2 = math.hypot(*v2)
    if mag1 == 0 or mag2 == 0: return 0.0
    cos_phi = max(-1, min(1, dot / (mag1 * mag2)))
    return math.acos(cos_phi)

class ImprovedDStarLite:
    def __init__(self, maze, start, goal, phi_max=math.radians(135)):
        self.maze = np.copy(maze)
        self.s_start = start
        self.s_goal = goal
        self.s_last = start
        self.phi_max = phi_max
        self.k_m = 0.0
        
        self.U = []
        self.U_dict = {}
        self.g = {}
        self.rhs = {}
        self.preds = {} 
        self.expansions = 0
        
        self.rhs[self.s_goal] = 0.0
        self.insert_U(self.s_goal, self.calculate_key(self.s_goal))
        self.compute_shortest_path()

    def get_g(self, s): return self.g.get(s, INF)
    def get_rhs(self, s): return self.rhs.get(s, INF)

    def calculate_key(self, s):
        g_rhs = min(self.get_g(s), self.get_rhs(s))
        return (g_rhs + heuristic(self.s_start, s) + self.k_m, g_rhs)

    def insert_U(self, s, key):
        self.U_dict[s] = key
        heapq.heappush(self.U, (key[0], key[1], s))

    def remove_U(self, s):
        if s in self.U_dict: del self.U_dict[s]

    def update_vertex(self, u):
        if u != self.s_goal:
            min_rhs = INF
            best_neighbor = None
            for s_prime in get_8way_neighbors(self.maze, u):
                angle = calculate_angle(self.preds.get(s_prime), s_prime, u)
                if angle > self.phi_max: continue
                
                current_rhs = improved_cost(self.maze, u, s_prime) + self.get_g(s_prime)
                if current_rhs < min_rhs:
                    min_rhs = current_rhs
                    best_neighbor = s_prime
            self.rhs[u] = min_rhs
            if best_neighbor: self.preds[u] = best_neighbor
        
        self.remove_U(u)
        if self.get_g(u) != self.get_rhs(u):
            self.insert_U(u, self.calculate_key(u))

    def compute_shortest_path(self):
        while True:
            if not self.U: break
            k_old = self.U[0][:2]
            k_start = self.calculate_key(self.s_start)
            if k_old >= k_start and self.get_rhs(self.s_start) == self.get_g(self.s_start):
                break
            
            _, _, u = heapq.heappop(self.U)
            if u not in self.U_dict: continue
            del self.U_dict[u]
            self.expansions += 1
            
            if self.get_g(u) > self.get_rhs(u):
                self.g[u] = self.get_rhs(u)
                for s in get_8way_neighbors(self.maze, u):
                    self.update_vertex(s)
            else:
                self.g[u] = INF
                self.update_vertex(u)
                for s in get_8way_neighbors(self.maze, u):
                    self.update_vertex(s)

    def extract_path(self, start_node=None):
        curr = start_node if start_node is not None else self.s_start
        if self.get_g(curr) == INF:
            return [], self.expansions
        path = [curr]
        while curr != self.s_goal:
            neighbors = get_8way_neighbors(self.maze, curr)
            curr = min(neighbors, key=lambda s: improved_cost(self.maze, curr, s) + self.get_g(s))
            if curr in path: break 
            path.append(curr)
        return path, self.expansions

    def update_map(self, new_maze, current_pos, changed_cells):
        self.k_m += heuristic(self.s_last, current_pos)
        self.s_last = current_pos
        self.s_start = current_pos
        self.maze = np.copy(new_maze) # Update internal map with the one from ROS
        
        for (r, c) in changed_cells:
            u = (r, c)
            self.update_vertex(u)
            for s in get_8way_neighbors(self.maze, u):
                self.update_vertex(s)
        
        self.compute_shortest_path()