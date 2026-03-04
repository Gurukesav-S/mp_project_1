import heapq
import numpy as np
import math
import time
INF = float('inf')

def heuristic(p1, p2):
    # Perfect Octile distance for 8-way grid tracking
    dx = abs(p1[0] - p2[0])
    dy = abs(p1[1] - p2[1])
    return 1.0 * (dx + dy) + (math.sqrt(2) - 2.0) * min(dx, dy)

# def get_8way_neighbors(maze, node):
#     neighbors = []
#     r, c = node
#     rows, cols = maze.shape
    
#     for dr in [-1, 0, 1]:
#         for dc in [-1, 0, 1]:
#             if dr == 0 and dc == 0:
#                 continue
            
#             nr = r + dr
#             nc = c + dc
            
#             # 1. Basic boundary check
#             if 0 <= nr < rows and 0 <= nc < cols:
#                 # 2. Check if the target neighbor itself is an obstacle
#                 if maze[nr, nc] == 1:
#                     continue
                
#                 # 3. CORNER CUTTING PREVENTION:
#                 # If this is a diagonal move, check the two side-cells.
#                 # If either 'side' is an obstacle, do not allow the diagonal path.
#                 if abs(dr) == 1 and abs(dc) == 1:
#                     # Side-cell 1: (r + dr, c)
#                     # Side-cell 2: (r, c + dc)
#                     if maze[r + dr, c] == 1 or maze[r, c + dc] == 1:
#                         continue # Skip this diagonal neighbor
                
#                 neighbors.append((nr, nc))
#     return neighbors

def get_8way_neighbors(maze, node):
    neighbors = []
    r, c = node
    rows, cols = maze.shape
    for dr in [-1, 0, 1]:
        for dc in [-1, 0, 1]:
            if dr == 0 and dc == 0:
                continue
            nr = r + dr
            nc = c + dc
            if 0 <= nr < rows and 0 <= nc < cols:
                neighbors.append((nr, nc))
    return neighbors

def cost(maze, u, v):
    if maze[u] == 1 or maze[v] == 1:
        return INF
    if abs(u[0] - v[0]) == 1 and abs(u[1] - v[1]) == 1:
        return math.sqrt(2)
    return 1.0

# Safe mathematical equality check to prevent float looping
def is_consistent(g_val, rhs_val):
    if g_val == INF and rhs_val == INF:
        return True
    if g_val == INF or rhs_val == INF:
        return False
    return abs(g_val - rhs_val) < 1e-5

class DStarLitePlanner:
    def __init__(self, maze, start, goal):
        self.maze = np.copy(maze)
        self.s_start = start
        self.s_goal = goal
        self.s_last = start
        self.k_m = 0.0
        
        self.U = []
        self.U_dict = {}
        self.g = {}
        self.rhs = {}
        self.expansions = 0
        
        self.latest_compute_time = 0.0
        self.total_compute_time = 0.0

        self.rhs[self.s_goal] = 0.0
        self.insert_U(self.s_goal, self.calculate_key(self.s_goal))
        t_start = time.perf_counter()
        self.compute_shortest_path()
        self.latest_compute_time = time.perf_counter() - t_start
        self.total_compute_time += self.latest_compute_time

    def get_g(self, s):
        return self.g.get(s, INF)

    def get_rhs(self, s):
        return self.rhs.get(s, INF)

    def calculate_key(self, s):
        g_rhs = min(self.get_g(s), self.get_rhs(s))
        
        k1 = g_rhs + heuristic(self.s_start, s) + self.k_m
        k2 = g_rhs
        
        # Safe float rounding strictly for the priority queue
        if k1 != INF: k1 = round(k1, 5)
        if k2 != INF: k2 = round(k2, 5)
        
        return (k1, k2)

    def insert_U(self, s, key):
        self.U_dict[s] = key
        heapq.heappush(self.U, (key[0], key[1], s))

    def remove_U(self, s):
        if s in self.U_dict:
            del self.U_dict[s]

    def top_key(self):
        while self.U:
            k1, k2, s = self.U[0]
            if self.U_dict.get(s) == (k1, k2):
                return (k1, k2)
            else:
                heapq.heappop(self.U)
        return (INF, INF)

    def pop_U(self):
        while self.U:
            k1, k2, s = heapq.heappop(self.U)
            if self.U_dict.get(s) == (k1, k2):
                del self.U_dict[s]
                return (k1, k2), s
        return (INF, INF), None

    def update_vertex(self, u):
        if u != self.s_goal:
            min_rhs = INF
            for s_prime in get_8way_neighbors(self.maze, u):
                c = cost(self.maze, u, s_prime)
                if c != INF and self.get_g(s_prime) != INF:
                    min_rhs = min(min_rhs, c + self.get_g(s_prime))
            self.rhs[u] = min_rhs
        
        self.remove_U(u)
        
        if not is_consistent(self.get_g(u), self.get_rhs(u)):
            self.insert_U(u, self.calculate_key(u))

    def compute_shortest_path(self):
        while True:
            k_old = self.top_key()
            k_start = self.calculate_key(self.s_start)
            
            if k_old >= k_start and is_consistent(self.get_g(self.s_start), self.get_rhs(self.s_start)):
                break
            
            k_old_popped, u = self.pop_U()
            if u is None:
                break
                
            self.expansions += 1
            k_new = self.calculate_key(u)
            
            if k_old_popped < k_new:
                self.insert_U(u, k_new)
            elif self.get_g(u) > self.get_rhs(u):
                self.g[u] = self.get_rhs(u)
                for s in get_8way_neighbors(self.maze, u):
                    self.update_vertex(s)
            else:
                self.g[u] = INF
                self.update_vertex(u)
                for s in get_8way_neighbors(self.maze, u):
                    self.update_vertex(s)

    def update_map(self, new_maze, current_start):
        self.k_m += heuristic(self.s_last, current_start)
        self.s_last = current_start
        self.s_start = current_start
        
        changed_indices = np.argwhere(self.maze != new_maze)
        changed_cells = [(r, c) for r, c in changed_indices]
        
        for (r, c) in changed_cells:
            u = (r, c)
            self.maze[r, c] = new_maze[r, c]
            self.update_vertex(u)
            for s in get_8way_neighbors(self.maze, u):
                self.update_vertex(s)
        t_start = time.perf_counter()    
        self.compute_shortest_path()
        self.latest_compute_time = time.perf_counter() - t_start
        self.total_compute_time += self.latest_compute_time

    def extract_path(self, start_node):
        if self.get_g(start_node) == INF:
            return [], self.expansions
            
        path = [start_node]
        current = start_node
        visited = {start_node} 
        
        while current != self.s_goal:
            min_cost = INF
            next_node = None
            
            for s in get_8way_neighbors(self.maze, current):
                c_edge = cost(self.maze, current, s)
                if c_edge == INF: continue
                
                g_s = self.get_g(s)
                if g_s == INF: continue
                
                c = round(c_edge + g_s, 5)
                if c < min_cost:
                    min_cost = c
                    next_node = s
            
            if next_node is None or min_cost == INF or next_node in visited:
                break
                
            visited.add(next_node)
            current = next_node
            path.append(current)
            
        return path, self.expansions

