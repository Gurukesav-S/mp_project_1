import heapq
import math
import numpy as np

class ImprovedDLite:
    def __init__(self, grid_map, start, goal, phi_max=math.radians(135)):
        self.start, self.goal = start, goal
        self.grid = grid_map  # 0: free, 1: obstacle
        self.phi_max = phi_max  # Maximum turning angle constraint (Eq. 14) [cite: 178]
        
        self.U, self.km = [], 0
        self.rhs, self.g = {}, {}
        
        for r in range(grid_map.shape[0]):
            for c in range(grid_map.shape[1]):
                self.rhs[(r, c)] = float('inf')
                self.g[(r, c)] = float('inf')
        
        self.rhs[self.goal] = 0
        heapq.heappush(self.U, (self.calculate_key(self.goal), self.goal))

    def calculate_cost(self, s, s_prime):
        # Boundary check to prevent IndexError
        if not (0 <= s_prime[0] < self.grid.shape[0] and 0 <= s_prime[1] < self.grid.shape[1]):
            return float('inf')

        if self.grid[int(s_prime[0]), int(s_prime[1])] == 1: 
            return float('inf')
        
        dr, dc = abs(s[0] - s_prime[0]), abs(s[1] - s_prime[1])
        return math.sqrt(2) * min(dr, dc) + abs(dr - dc)

    def calculate_key(self, s):
        h = self.calculate_cost(self.start, s)
        k1 = min(self.g[s], self.rhs[s]) + h + self.km
        k2 = min(self.g[s], self.rhs[s])
        return (k1, k2)

    def get_neighbors(self, s):
        neighbors = []
        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            n = (s[0] + dr, s[1] + dc)
            if 0 <= n[0] < self.grid.shape[0] and 0 <= n[1] < self.grid.shape[1]:
                neighbors.append(n)
        return neighbors

    def is_angle_valid(self, s_pred, s, s_succ):
        """Limit direction of expansion to avoid unfeasible turns (Eq. 14)[cite: 177, 178]."""
        if s_pred is None or s_pred == s: return True
        v1 = np.array([s[0] - s_pred[0], s[1] - s_pred[1]])
        v2 = np.array([s_succ[0] - s[0], s_succ[1] - s[1]])
        norm = np.linalg.norm(v1) * np.linalg.norm(v2)
        if norm == 0: return True
        # Cosine law to determine expansion angle [cite: 177]
        phi = math.acos(max(-1.0, min(1.0, np.dot(v1, v2) / norm)))
        return phi <= self.phi_max

    def update_vertex(self, u):
        if u != self.goal:
            self.rhs[u] = min(self.calculate_cost(u, s_p) + self.g[s_p] 
                             for s_p in self.get_neighbors(u))
        self.U = [i for i in self.U if i[1] != u]
        heapq.heapify(self.U)
        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, (self.calculate_key(u), u))

    def compute_shortest_path(self, current_pred=None):
        while len(self.U) > 0 and (self.U[0][0] < self.calculate_key(self.start) 
                                   or self.rhs[self.start] != self.g[self.start]):
            k_old, u = heapq.heappop(self.U)
            if k_old < self.calculate_key(u):
                heapq.heappush(self.U, (self.calculate_key(u), u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.get_neighbors(u):
                    if self.is_angle_valid(current_pred, u, s): # Optimization [cite: 178]
                        self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                for s in self.get_neighbors(u) + [u]:
                    self.update_vertex(s)

    def extract_path(self, start_node):
        self.start = start_node
        self.compute_shortest_path()
        path = [self.start]
        curr = self.start
        while curr != self.goal:
            best_n = min(self.get_neighbors(curr), 
                         key=lambda n: self.calculate_cost(curr, n) + self.g[n], 
                         default=None)
            if best_n is None or best_n == curr: break
            curr = best_n
            path.append(curr)
        return path, self.g.get(self.start, float('inf'))