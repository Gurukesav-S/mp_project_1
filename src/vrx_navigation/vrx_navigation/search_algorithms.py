import heapq
import numpy as np
import math

INF = float('inf')

def heuristic(p1, p2):
    # Perfect Octile distance for 8-way grid tracking
    dx = abs(p1[0] - p2[0])
    dy = abs(p1[1] - p2[1])
    return 1.0 * (dx + dy) + (math.sqrt(2) - 2.0) * min(dx, dy)

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
        
        self.rhs[self.s_goal] = 0.0
        self.insert_U(self.s_goal, self.calculate_key(self.s_goal))
        self.compute_shortest_path()

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
                
        self.compute_shortest_path()

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







# import heapq
# import numpy as np
# import math

# INF = float('inf')

# def heuristic(p1, p2):
#     return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

# def get_8way_neighbors(maze, node):
#     neighbors = []
#     r, c = node
#     rows, cols = maze.shape
#     for dr in [-1, 0, 1]:
#         for dc in [-1, 0, 1]:
#             if dr == 0 and dc == 0:
#                 continue
#             nr, nc = r + dr, c + dc
#             if 0 <= nr < rows and 0 <= nc < cols:
#                 neighbors.append((nr, nc))
#     return neighbors

# def cost(maze, u, v):
#     if maze[u] == 1 or maze[v] == 1:
#         return INF
#     if abs(u[0] - v[0]) == 1 and abs(u[1] - v[1]) == 1:
#         return 1.41421 
#     return 1.0

# # THE FIX: Safely handles INF values to prevent Python from crashing
# def safe_round(val, decimals=5):
#     if math.isinf(val) or math.isnan(val):
#         return val
#     return round(val, decimals)

# class DStarLitePlanner:
#     def __init__(self, maze, start, goal):
#         self.maze = np.copy(maze)
#         self.s_start = start
#         self.s_goal = goal
#         self.s_last = start
#         self.k_m = 0.0
        
#         self.U = []
#         self.U_dict = {}
#         self.g = {}
#         self.rhs = {}
#         self.expansions = 0
        
#         self.rhs[self.s_goal] = 0.0
#         self.insert_U(self.s_goal, self.calculate_key(self.s_goal))
#         self.compute_shortest_path()

#     def get_g(self, s):
#         return self.g.get(s, INF)

#     def get_rhs(self, s):
#         return self.rhs.get(s, INF)

#     def calculate_key(self, s):
#         g_rhs = min(self.get_g(s), self.get_rhs(s))
#         return (safe_round(g_rhs + heuristic(self.s_start, s) + self.k_m, 5), safe_round(g_rhs, 5))

#     def insert_U(self, s, key):
#         self.U_dict[s] = key
#         heapq.heappush(self.U, (key[0], key[1], s))

#     def remove_U(self, s):
#         if s in self.U_dict:
#             del self.U_dict[s]

#     def top_key(self):
#         while self.U:
#             k1, k2, s = self.U[0]
#             if self.U_dict.get(s) == (k1, k2):
#                 return (k1, k2)
#             else:
#                 heapq.heappop(self.U)
#         return (INF, INF)

#     def pop_U(self):
#         while self.U:
#             k1, k2, s = heapq.heappop(self.U)
#             if self.U_dict.get(s) == (k1, k2):
#                 del self.U_dict[s]
#                 return (k1, k2), s
#         return (INF, INF), None

#     def update_vertex(self, u):
#         if u != self.s_goal:
#             min_rhs = INF
#             for s_prime in get_8way_neighbors(self.maze, u):
#                 min_rhs = min(min_rhs, cost(self.maze, u, s_prime) + self.get_g(s_prime))
#             self.rhs[u] = min_rhs
        
#         self.remove_U(u)
        
#         # THE FIX: Explicitly ignore NaN results to prevent infinite loop errors
#         diff = abs(self.get_g(u) - self.get_rhs(u))
#         if diff > 1e-5 and not math.isnan(diff):
#             self.insert_U(u, self.calculate_key(u))

#     def compute_shortest_path(self):
#         while True:
#             k_old = self.top_key()
#             k_start = self.calculate_key(self.s_start)
            
#             diff = abs(self.get_rhs(self.s_start) - self.get_g(self.s_start))
#             if k_old >= k_start and (diff < 1e-5 or math.isnan(diff)):
#                 break
            
#             k_old_popped, u = self.pop_U()
#             if u is None:
#                 break
                
#             self.expansions += 1
#             k_new = self.calculate_key(u)
            
#             if k_old_popped < k_new:
#                 self.insert_U(u, k_new)
#             elif self.get_g(u) > self.get_rhs(u):
#                 self.g[u] = self.get_rhs(u)
#                 for s in get_8way_neighbors(self.maze, u):
#                     self.update_vertex(s)
#             else:
#                 self.g[u] = INF
#                 self.update_vertex(u)
#                 for s in get_8way_neighbors(self.maze, u):
#                     self.update_vertex(s)

#     # THE FIX: Diff calculation moved inside the class for perfect sync
#     def update_map(self, new_maze, current_start):
#         self.k_m += heuristic(self.s_last, current_start)
#         self.s_last = current_start
#         self.s_start = current_start
        
#         changed_indices = np.argwhere(self.maze != new_maze)
#         changed_cells = [(r, c) for r, c in changed_indices]
        
#         for (r, c) in changed_cells:
#             u = (r, c)
#             self.maze[r, c] = new_maze[r, c]
#             self.update_vertex(u)
#             for s in get_8way_neighbors(self.maze, u):
#                 self.update_vertex(s)
                
#         self.compute_shortest_path()

#     def extract_path(self, start_node):
#         if self.get_g(start_node) == INF:
#             return [], self.expansions
            
#         path = [start_node]
#         current = start_node
#         visited = {start_node} 
        
#         while current != self.s_goal:
#             min_cost = INF
#             next_node = None
#             for s in get_8way_neighbors(self.maze, current):
#                 c = safe_round(cost(self.maze, current, s) + self.get_g(s), 5)
#                 if c < min_cost:
#                     min_cost = c
#                     next_node = s
            
#             if next_node is None or min_cost == INF or next_node in visited:
#                 break
                
#             visited.add(next_node)
#             current = next_node
#             path.append(current)
            
#         return path, self.expansions





# # import heapq
# # import numpy as np
# # import math

# # INF = float('inf')

# # def heuristic(p1, p2):
# #     return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

# # def get_8way_neighbors(maze, node):
# #     neighbors = []
# #     r, c = node
# #     rows, cols = maze.shape
# #     for dr in [-1, 0, 1]:
# #         for dc in [-1, 0, 1]:
# #             if dr == 0 and dc == 0:
# #                 continue
# #             nr, nc = r + dr, c + dc
# #             if 0 <= nr < rows and 0 <= nc < cols:
# #                 neighbors.append((nr, nc))
# #     return neighbors

# # def cost(maze, u, v):
# #     if maze[u] == 1 or maze[v] == 1:
# #         return INF
# #     if abs(u[0] - v[0]) == 1 and abs(u[1] - v[1]) == 1:
# #         return 1.41421 # Added precision to prevent jitter
# #     return 1.0

# # class DStarLitePlanner:
# #     def __init__(self, maze, start, goal):
# #         self.maze = np.copy(maze)
# #         self.s_start = start
# #         self.s_goal = goal
# #         self.s_last = start
# #         self.k_m = 0.0
        
# #         self.U = []
# #         self.U_dict = {}
# #         self.g = {}
# #         self.rhs = {}
# #         self.expansions = 0
        
# #         self.rhs[self.s_goal] = 0.0
# #         self.insert_U(self.s_goal, self.calculate_key(self.s_goal))
# #         self.compute_shortest_path()

# #     def get_g(self, s):
# #         return self.g.get(s, INF)

# #     def get_rhs(self, s):
# #         return self.rhs.get(s, INF)

# #     def calculate_key(self, s):
# #         g_rhs = min(self.get_g(s), self.get_rhs(s))
# #         # Rounding keys completely eliminates the infinite-loop precision bug
# #         return (round(g_rhs + heuristic(self.s_start, s) + self.k_m, 5), round(g_rhs, 5))

# #     def insert_U(self, s, key):
# #         self.U_dict[s] = key
# #         heapq.heappush(self.U, (key[0], key[1], s))

# #     def remove_U(self, s):
# #         if s in self.U_dict:
# #             del self.U_dict[s]

# #     def top_key(self):
# #         while self.U:
# #             k1, k2, s = self.U[0]
# #             if self.U_dict.get(s) == (k1, k2):
# #                 return (k1, k2)
# #             else:
# #                 heapq.heappop(self.U)
# #         return (INF, INF)

# #     def pop_U(self):
# #         while self.U:
# #             k1, k2, s = heapq.heappop(self.U)
# #             if self.U_dict.get(s) == (k1, k2):
# #                 del self.U_dict[s]
# #                 return (k1, k2), s
# #         return (INF, INF), None

# #     def update_vertex(self, u):
# #         if u != self.s_goal:
# #             min_rhs = INF
# #             for s_prime in get_8way_neighbors(self.maze, u):
# #                 min_rhs = min(min_rhs, cost(self.maze, u, s_prime) + self.get_g(s_prime))
# #             self.rhs[u] = min_rhs
        
# #         self.remove_U(u)
        
# #         # Safe float comparison
# #         if abs(self.get_g(u) - self.get_rhs(u)) > 1e-5:
# #             self.insert_U(u, self.calculate_key(u))

# #     def compute_shortest_path(self):
# #         while True:
# #             k_old = self.top_key()
# #             k_start = self.calculate_key(self.s_start)
            
# #             if k_old >= k_start and abs(self.get_rhs(self.s_start) - self.get_g(self.s_start)) < 1e-5:
# #                 break
            
# #             k_old_popped, u = self.pop_U()
# #             if u is None:
# #                 break
                
# #             self.expansions += 1
# #             k_new = self.calculate_key(u)
            
# #             if k_old_popped < k_new:
# #                 self.insert_U(u, k_new)
# #             elif self.get_g(u) > self.get_rhs(u):
# #                 self.g[u] = self.get_rhs(u)
# #                 for s in get_8way_neighbors(self.maze, u):
# #                     self.update_vertex(s)
# #             else:
# #                 self.g[u] = INF
# #                 self.update_vertex(u)
# #                 for s in get_8way_neighbors(self.maze, u):
# #                     self.update_vertex(s)

# #     def update_map(self, new_maze, current_start, changed_cells):
# #         self.k_m += heuristic(self.s_last, current_start)
# #         self.s_last = current_start
# #         self.s_start = current_start
        
# #         for (r, c) in changed_cells:
# #             u = (r, c)
# #             self.maze[r, c] = new_maze[r, c]
# #             self.update_vertex(u)
# #             for s in get_8way_neighbors(self.maze, u):
# #                 self.update_vertex(s)
                
# #         self.compute_shortest_path()

# #     def extract_path(self, start_node):
# #         if self.get_g(start_node) == INF:
# #             return [], self.expansions
            
# #         path = [start_node]
# #         current = start_node
# #         visited = {start_node}
        
# #         while current != self.s_goal:
# #             min_cost = INF
# #             next_node = None
# #             for s in get_8way_neighbors(self.maze, current):
# #                 c = round(cost(self.maze, current, s) + self.get_g(s), 5)
# #                 if c < min_cost:
# #                     min_cost = c
# #                     next_node = s
            
# #             # Anti-Cycle Protection
# #             if next_node is None or min_cost >= INF or next_node in visited:
# #                 break
                
# #             visited.add(next_node)
# #             current = next_node
# #             path.append(current)
            
# #         return path, self.expansions
# # import heapq
# # import numpy as np
# # import math

# # INF = float('inf')

# # def heuristic(p1, p2):
# #     # Euclidean distance
# #     return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

# # def get_8way_neighbors(maze, node):
# #     neighbors = []
# #     r, c = node
# #     rows, cols = maze.shape
# #     for dr in [-1, 0, 1]:
# #         for dc in [-1, 0, 1]:
# #             if dr == 0 and dc == 0:
# #                 continue
# #             nr, nc = r + dr, c + dc
# #             if 0 <= nr < rows and 0 <= nc < cols:
# #                 neighbors.append((nr, nc))
# #     return neighbors

# # def cost(maze, u, v):
# #     # If either cell is an obstacle, traversing is impossible
# #     if maze[u] == 1 or maze[v] == 1:
# #         return INF
# #     # Diagonal cost
# #     if abs(u[0] - v[0]) == 1 and abs(u[1] - v[1]) == 1:
# #         return 1.414
# #     # Straight cost
# #     return 1.0

# # class DStarLitePlanner:
# #     def __init__(self, maze, start, goal):
# #         self.maze = np.copy(maze)
# #         self.s_start = start
# #         self.s_goal = goal
# #         self.s_last = start
# #         self.k_m = 0.0
        
# #         self.U = []          # Priority Queue
# #         self.U_dict = {}     # Fast lookup for queue states
# #         self.g = {}
# #         self.rhs = {}
# #         self.expansions = 0
        
# #         # D* Lite searches BACKWARDS. The goal is the root of the tree.
# #         self.rhs[self.s_goal] = 0.0
# #         self.insert_U(self.s_goal, self.calculate_key(self.s_goal))
# #         self.compute_shortest_path()

# #     def get_g(self, s):
# #         return self.g.get(s, INF)

# #     def get_rhs(self, s):
# #         return self.rhs.get(s, INF)

# #     def calculate_key(self, s):
# #         g_rhs = min(self.get_g(s), self.get_rhs(s))
# #         return (g_rhs + heuristic(self.s_start, s) + self.k_m, g_rhs)

# #     def insert_U(self, s, key):
# #         self.U_dict[s] = key
# #         heapq.heappush(self.U, (key[0], key[1], s))

# #     def remove_U(self, s):
# #         if s in self.U_dict:
# #             del self.U_dict[s]

# #     def top_key(self):
# #         # Lazy deletion for priority queue
# #         while self.U:
# #             k1, k2, s = self.U[0]
# #             if self.U_dict.get(s) == (k1, k2):
# #                 return (k1, k2)
# #             else:
# #                 heapq.heappop(self.U)
# #         return (INF, INF)

# #     def pop_U(self):
# #         while self.U:
# #             k1, k2, s = heapq.heappop(self.U)
# #             if self.U_dict.get(s) == (k1, k2):
# #                 del self.U_dict[s]
# #                 return (k1, k2), s
# #         return (INF, INF), None

# #     def update_vertex(self, u):
# #         # Determine inconsistency for non-goal nodes based on neighbors
# #         if u != self.s_goal:
# #             min_rhs = INF
# #             for s_prime in get_8way_neighbors(self.maze, u):
# #                 min_rhs = min(min_rhs, cost(self.maze, u, s_prime) + self.get_g(s_prime))
# #             self.rhs[u] = min_rhs
        
# #         self.remove_U(u)
        
# #         # If node is inconsistent, add it to queue for repair
# #         if self.get_g(u) != self.get_rhs(u):
# #             self.insert_U(u, self.calculate_key(u))

# #     def compute_shortest_path(self):
# #         while True:
# #             k_old = self.top_key()
# #             k_start = self.calculate_key(self.s_start)
            
# #             if k_old >= k_start and self.get_rhs(self.s_start) == self.get_g(self.s_start):
# #                 break
            
# #             k_old_popped, u = self.pop_U()
# #             if u is None:
# #                 break
                
# #             self.expansions += 1
# #             k_new = self.calculate_key(u)
            
# #             if k_old_popped < k_new:
# #                 self.insert_U(u, k_new)
# #             elif self.get_g(u) > self.get_rhs(u):
# #                 self.g[u] = self.get_rhs(u)
# #                 for s in get_8way_neighbors(self.maze, u):
# #                     self.update_vertex(s)
# #             else:
# #                 self.g[u] = INF
# #                 self.update_vertex(u)
# #                 for s in get_8way_neighbors(self.maze, u):
# #                     self.update_vertex(s)

# #     def update_map(self, new_maze, current_start, changed_cells):
# #         # Update heuristic modifier because the ASV has physically moved
# #         self.k_m += heuristic(self.s_last, current_start)
# #         self.s_last = current_start
# #         self.s_start = current_start
        
# #         # Update ONLY the broken edges
# #         for (r, c) in changed_cells:
# #             u = (r, c)
# #             self.maze[r, c] = new_maze[r, c]
# #             self.update_vertex(u)
# #             for s in get_8way_neighbors(self.maze, u):
# #                 self.update_vertex(s)
                
# #         # Repair the tree (Lightning fast!)
# #         self.compute_shortest_path()

# #     def extract_path(self, start_node):
# #         # If g is infinity, we are completely walled off
# #         if self.get_g(start_node) == INF:
# #             return [], self.expansions
            
# #         path = [start_node]
# #         current = start_node
        
# #         # Follow the "gravity well" of lowest G costs down to the goal
# #         for _ in range(self.maze.size): 
# #             if current == self.s_goal:
# #                 break
# #             min_cost = INF
# #             next_node = None
# #             for s in get_8way_neighbors(self.maze, current):
# #                 c = cost(self.maze, current, s) + self.get_g(s)
# #                 if c < min_cost:
# #                     min_cost = c
# #                     next_node = s
            
# #             if next_node is None or min_cost == INF:
# #                 return [], self.expansions
                
# #             current = next_node
# #             path.append(current)
            
# #         return path, self.expansions




# # # #DO NOT MODIFY FOR HERE
# # # import random
# # # from queue import PriorityQueue, Queue
# # # import numpy as np
# # # import matplotlib.pyplot as plt

# # # ###############################        NOTES           ############################################################
# # # #1. Do not import an additional packages, read about the imported packages and learn to use them to solve to problem: Queue and PriorityQueue are the packages to use
# # # #2. Stack is readily available data type in python if you need to use it
# # # #3. Only when you run the code you will see GUI output, however when evaluating it will not show GUI, it will show path calcualted by your algorithm and corrct path if your path is not correct.
# # # #4. You can add any number of additional functions that you can call from already defined functions below
# # # #5. DO NOT MODIFY name, arguments or the return type othrwise your code will not get auto evaluated

# # # #*HINT: may be satrt from UCS implemntatio from the last assignment

# # # #################################################################################################################################

# # # #This fuction is not mandetory to use, you can consider unit edge cost for all the actions/nodes, but if you want to consider non-uniform edge cost you can call this function
# # # #Arguments:
# # # # 1. costs: contains cost between a given node and its neighbor
# # # # 2. node: this is the coordInate of the current node givn as [x,y]
# # # # 3. neighbor: this is the coordinate of the neighbor node givn as [x,y]

# # # # Returns: cost to go from the node to the neighbour

# # # def get_edge_cost(costs, node, neighbor):
# # #     nx, ny = node
# # #     return costs[nx, ny]

# # # #################################################################################################################################

# # # #This function returns all valid neighbours given the maze and the node

# # # #Arguments:
# # # # 1. maze: this is the search space of size X (rows) x Y (columns), its a numpy array of size X rows and Y columns
# # # # 2. node: this is the coordinate of the current node givn as [x,y]

# # # # Returns: all the neighbors to be added to the Queue or Prioity queue

# # # #Note: Put the neighbours in the Stack/Queue in same order as they are returned from this function while maintaining the correct priority
# # # def get_neighbors(maze, node):
# # #     x, y = node
# # #     neighbors = []
# # #     for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
# # #         nx, ny = x + dx, y + dy
# # #         if 0 <= nx < maze.shape[0] and 0 <= ny < maze.shape[1] and maze[nx, ny] == 0:
# # #             neighbors.append((nx, ny))
# # #     return neighbors

# # # ################################################################################################################################# TO HERE

# # # #MODIFY CONTENT FROM THIS LINE ONWARDS HOWEVER DO NOT MODIFY FUNCTION NAMES ARGUMENTS OR RETURN TYPE

# # # #This function returns manhattan distance between the goal state and the current state

# # # #Arguments:
# # # # 1. goal: goal state as provided to the function "a_star" below
# # # # 2. node: this is the coordinate of the current node givn as [x,y] for which you want to calcuate the manhattan heuristic

# # # # Returns: manhattan distance between the goal and the node passed as arguments
# # # def manhattan_heuristice(goal, node):
# # #     distance = 0 #update this variable and return the calcuated/updated value

# # #     "*** YOUR CODE HERE ***"
# # #     distance = abs(goal[0]-node[0])+abs(goal[1]-node[1])
# # #     return distance

# # # #################################################################################################################################

# # # #This function returns euclidean distance between the goal state and the current state

# # # #Arguments:
# # # # 1. goal: goal state as provided to the function "a_star" below
# # # # 2. node: this is the coordinate of the current node givn as [x,y] for which you want to calcuate the euclidean heuristic

# # # # Returns: euclidean distance between the goal and the node passed as arguments
# # # def euclidean_heuristic(goal, node):
# # #     distance = 0 #update this variable and return the calcuated/updated value

# # #     "*** YOUR CODE HERE ***"
# # #     distance = np.sqrt((goal[0]-node[0])**2+(goal[1]-node[1])**2)
# # #     return distance

# # # #This function should implement a_star algorithm with 3 different heurstics, the choice of the heuristic is passed as an argument

# # # #Arguments:
# # # # 1. maze: this is the search space of size X (rows) x Y (columns), its a numpy array of size X rows and Y columns
# # # # 2. start: coordinate of the start node givn as [x,y]
# # # # 3. goal: coordinate of the goal node givn as [x,y]
# # # # 4. costs: if you choose to use non-uniform dge costs you can call get_edge_cost() with appropriate arguments just like how you used it in assignmet 1 UCS implementation
# # # # 5. heuristic: if heuristic=0 use Manhattan Distance heuristic
# # # #               if heuristic=1 use Euclidean Distance heuristic
# # # #               if heuristic>=2 use weighted heuristic with manhattan distance, so calcuated heuristic value should be "heuristic * manhattan_heuristice(goal, node)".
# # # # Return values
# # # # 1. a list of nodes containng path from START to GOAL, rememebr the first node in this list should be START and the last one should be GOAL

# # # def a_star(maze, start, goal, costs, heuristic):
# # #     path = [] #this should contai list of nodes [start, (20,30), (21,30), ...., goal] as a path from start to goal
# # #     "*** YOUR CODE HERE ***"
# # #     fringe = PriorityQueue()
# # #     closed_set = set()
# # #     parent_dict = {start: None}
# # #     g_cost = {start:0}
# # #     h_start = get_heuristic(goal,start,heuristic)
# # #     fringe.put((h_start,0,start))   #(g+h,g,node)
    
# # #     while not fringe.empty():
# # #         current_f,current_g,current_node = fringe.get()
        
# # #         if current_node == goal:
# # #             curr = goal
# # #             while curr is not None:
# # #                 path.append(curr)
# # #                 curr = parent_dict[curr]
# # #             path.reverse()
# # #             return path, len(closed_set)
            
# # #         # if current_node in closed_set:
# # #         #     continue
# # #         if current_g>g_cost[current_node]:
# # #             continue
# # #         closed_set.add(current_node)
# # #         neighbours = get_neighbors(maze, current_node)
        
# # #         for next_node in neighbours:
# # #             # if next_node in closed_set:
# # #             #     continue
            
# # #             edge_cost = 1 #get_edge_cost(costs,current_node,next_node)
# # #             new_g = g_cost[current_node] + edge_cost
            
# # #             if next_node not in g_cost or new_g < g_cost[next_node]:
# # #                 g_cost[next_node] = new_g
# # #                 parent_dict[next_node] = current_node
# # #                 h_cost = get_heuristic(goal,next_node,heuristic)
# # #                 f_cost = new_g +h_cost
# # #                 fringe.put((f_cost,new_g,next_node))   
        
                
# # #     return path, len(closed_set)
# # # ################################################################################################################################# 
# # # #Any oher functions you may want to define
# # # def get_heuristic(goal,node,heuristic):
# # #     if heuristic == 0 :
# # #         return manhattan_heuristice(goal,node)
# # #     elif heuristic == 1:
# # #         return euclidean_heuristic(goal,node)
# # #     elif heuristic >=2:
# # #         return heuristic * manhattan_heuristice(goal,node)