import numpy as np
import random
import time
import networkx as nx
import matplotlib.pyplot as plt
from field_plotter import FieldPlotter

NUM_OBSTACLES = 10
TIME_CHANGE_THETAS_OBS = 5
V_MIN_OBS, V_MAX_OBS = 0, 3

X_MIN, X_MAX = -11, 11
Y_MIN, Y_MAX = -7, 7

DT = 0.025

N = 20

MAP_DIM = (X_MAX, Y_MAX)


class PathPlannerGraph:
    def __init__(self, start, goal, obstacles, points_per_obstacle=5, ellipse_factor=1.1):
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.obstacles = obstacles
        self.points_per_obstacle = points_per_obstacle
        self.ellipse_factor = ellipse_factor
        self.graph = nx.Graph()
        

    def _is_point_in_ellipse(self, point):
        sum_dist = (np.linalg.norm(np.array(point) - np.array(self.start)) + (np.linalg.norm(np.array(point) - np.array(self.goal))))
        max_dist = self.ellipse_factor * np.linalg.norm(np.array(self.start) - np.array(self.goal))
        return sum_dist <= max_dist

    def _generate_and_connect_nodes(self):

        self.graph.add_node(self.start)
        self.graph.add_node(self.goal)
        if not self._edge_intersects_obstacles(self.start, self.goal):
            self.graph.add_edge(self.start, self.goal, 
                            weight=np.linalg.norm(np.array(self.start) - np.array(self.goal)))
            return [self.start, self.goal]
    
        self._connect_node_to_existing_nodes(self.start)
        
        for idx, (ox, oy, radius) in enumerate(self.obstacles):
            for i in range(self.points_per_obstacle):
                angle = 2 * np.pi * i / self.points_per_obstacle
                px = ox + (radius + 0.3) * np.cos(angle)
                py = oy + (radius + 0.3) * np.sin(angle)
                pt = (px, py)

                inside_other_obstacle = any(
                    np.linalg.norm(np.array([px, py]) - np.array([other_ox, other_oy])) < other_r
                    for jdx, (other_ox, other_oy, other_r) in enumerate(self.obstacles)
                    if jdx != idx
                )

                if not inside_other_obstacle and self._is_point_in_ellipse(pt):
                    self.graph.add_node(pt)
                    self._connect_node_to_existing_nodes(pt)

    def _connect_node_to_existing_nodes(self, new_node):
        existing_nodes = list(self.graph.nodes())
        for existing_node in existing_nodes:
            if existing_node != new_node:
                if not self._edge_intersects_obstacles(new_node, existing_node):
                    distance = np.linalg.norm(np.array(new_node) - np.array(existing_node))
                    self.graph.add_edge(new_node, existing_node, weight=distance)

    def _edge_intersects_obstacles(self, p1, p2):
        """Verifica se a aresta entre p1 e p2 colide com algum obstáculo."""
        p1_np, p2_np = np.array(p1), np.array(p2)
        line = p2_np - p1_np
        
        for (ox, oy, r) in self.obstacles:
            center = np.array([ox, oy])
            if np.linalg.norm(line) == 0: 
                continue
            t = np.dot(center - p1_np, line) / np.dot(line, line)
            t = np.clip(t, 0, 1)
            closest_point = p1_np + t * line
            if np.linalg.norm(center - closest_point) < r:
                return True
        return False

    def find_path(self, start, goal, obstacles):
        self.graph.clear()
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.obstacles = obstacles
        self._generate_and_connect_nodes()

        try:
            return nx.astar_path(self.graph, self.start, self.goal, heuristic=lambda a, b: np.linalg.norm(np.array(a)-np.array(b)))
        except nx.NetworkXNoPath:
            return []

    def plot(self, path=None):
        pos = {node: node for node in self.graph.nodes}
        nx.draw(self.graph, pos, node_size=20, edge_color='blue')
        
        # Desenha a elipse de restrição
        self._plot_ellipse()
        
        if path:
            edges = list(zip(path, path[1:]))
            nx.draw_networkx_edges(self.graph, pos, edgelist=edges, edge_color='red', width=2)
        
        circle1 = plt.Circle(self.start, 0.28, color='green', alpha=0.5)
        circle2 = plt.Circle(self.goal, 0.28, color='orange', alpha=0.5)
        plt.gca().add_patch(circle1)
        plt.gca().add_patch(circle2)
        for (x, y, r) in self.obstacles:
            circle1 = plt.Circle((x, y), r, color='gray', alpha=0.5)
            circle2 = plt.Circle((x, y), 0.28, color='red', alpha=0.5)
            plt.gca().add_patch(circle1)
            plt.gca().add_patch(circle2)
        plt.axis('equal')
        plt.grid()
        plt.show()

    def _plot_ellipse(self):
        """Desenha a elipse que restringe a área de geração de nós."""
        start = np.array(self.start)
        goal = np.array(self.goal)
        
        # Parâmetros da elipse
        c = np.linalg.norm(goal - start) / 2  # Distância do centro a cada foco
        a = self.ellipse_factor * c  # Semi-eixo maior (soma das distâncias = 2a)
        
        if a <= c:
            return  # Não é uma elipse válida (a > c é necessário)
        
        # Centro da elipse (ponto médio entre start e goal)
        center = (start + goal) / 2
        
        # Ângulo de rotação da elipse (ângulo da linha start-goal)
        angle_rad = np.arctan2(goal[1] - start[1], goal[0] - start[0])
        
        # Semi-eixo menor (b = sqrt(a² - c²))
        b = np.sqrt(a**2 - c**2)
        
        # Gera pontos da elipse rotacionada
        t = np.linspace(0, 2 * np.pi, 100)
        ellipse_x = center[0] + a * np.cos(t) * np.cos(angle_rad) - b * np.sin(t) * np.sin(angle_rad)
        ellipse_y = center[1] + a * np.cos(t) * np.sin(angle_rad) + b * np.sin(t) * np.cos(angle_rad)
        
        # Plota a elipse
        plt.plot(ellipse_x, ellipse_y, 'r--', linewidth=1, alpha=0.7, label='Área de busca (elipse)')
        plt.legend()


# Normalizador de ângulo
def normalizeAngle(angle):
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle

# Inicializa posições dos obstáculos
def initializeObstaclesPositions():
    x = np.random.uniform(X_MIN, X_MAX, NUM_OBSTACLES)
    y = np.random.uniform(Y_MIN, Y_MAX, NUM_OBSTACLES)
    return np.array([x, y, np.zeros(NUM_OBSTACLES), np.zeros(NUM_OBSTACLES)], dtype=np.float64)

# Atualiza os obstáculos em movimento
def updateObstacles(obstacles_positions, thetas_obs, v_obs, dt):
    for i in range(NUM_OBSTACLES):
        if obstacles_positions[0, i] < X_MIN or obstacles_positions[0, i] > X_MAX:
            thetas_obs[i] = 180 - thetas_obs[i]
        if obstacles_positions[1, i] < Y_MIN or obstacles_positions[1, i] > Y_MAX:
            thetas_obs[i] = -thetas_obs[i]

        obstacles_positions[0, i] += v_obs[i] * np.cos(np.deg2rad(thetas_obs[i])) * dt
        obstacles_positions[1, i] += v_obs[i] * np.sin(np.deg2rad(thetas_obs[i])) * dt
        obstacles_positions[2, i] = v_obs[i]
        obstacles_positions[3, i] = normalizeAngle(np.deg2rad(thetas_obs[i]))
    return obstacles_positions


traj = np.zeros((N, N))


def main():
    robot_position = [0, 0]
    goal = [10, 6]
    plotter = FieldPlotter(numObstacles=NUM_OBSTACLES, N=N, PRINT=True)

    obstacles_positions = initializeObstaclesPositions()
    thetas_obs = [random.randint(-180, 180) for _ in range(NUM_OBSTACLES)]
    v_obs = [random.randint(V_MIN_OBS, V_MAX_OBS) for _ in range(NUM_OBSTACLES)]

    obstacles_positions = updateObstacles(obstacles_positions, thetas_obs, v_obs, DT)

    # Predição de posições futuras
    obstacle_list = []
    for obs_idx in range(NUM_OBSTACLES):
        d = np.linalg.norm(np.array(robot_position) - obstacles_positions[:2, obs_idx])
        ang = np.arctan2(obstacles_positions[1, obs_idx] - robot_position[1],
                            obstacles_positions[0, obs_idx] - robot_position[0])
        v = v_obs[obs_idx]
        dir = np.deg2rad(thetas_obs[obs_idx])

        for t in range(N):
            obs_x = robot_position[0] + d * np.cos(ang) + v * np.cos(dir) * t * DT
            obs_y = robot_position[1] + d * np.sin(ang) + v * np.sin(dir) * t * DT
            obstacle_list.append([obs_x, obs_y])

    planner = PathPlannerGraph(start=robot_position, goal=goal, obstacles=obstacle_list)
    
    while plotter.running:
        # Atualiza obstáculos reais
        obstacles_positions = updateObstacles(obstacles_positions, thetas_obs, v_obs, DT)

        # Predição de posições futuras
        obstacle_list = []
        for obs_idx in range(NUM_OBSTACLES):
            d = np.linalg.norm(np.array(robot_position) - obstacles_positions[:2, obs_idx])
            ang = np.arctan2(obstacles_positions[1, obs_idx] - robot_position[1],
                                obstacles_positions[0, obs_idx] - robot_position[0])
            v = v_obs[obs_idx]
            dir = np.deg2rad(thetas_obs[obs_idx])

            # for t in range(N):
            obs_x = robot_position[0] + d * np.cos(ang) # + v * np.cos(dir) * t * DT
            obs_y = robot_position[1] + d * np.sin(ang) # + v * np.sin(dir) * t * DT

            dist_to_target = np.sqrt((obs_x - goal[0])**2 + (obs_y - goal[1])**2)
            if dist_to_target <= 0.2:
                obs_r = 0.2
            elif dist_to_target >= 1.4:
                obs_r = 1.00
            else:
                ratio = (dist_to_target - 0.2) / (1.4 - 0.2)
                obs_r = 0.2 + ratio * (1.00 - 0.2)

            obstacle_list.append((obs_x, obs_y, obs_r))

        st = time.perf_counter()
        path = planner.find_path(robot_position, goal, obstacle_list)
        ft = time.perf_counter()
        if (1 / (ft-st)) < 40:
            print(f"Process Time: {ft-st} | Freq: {1 / (ft-st)}")

            print("####################################################################################################################")
            print("####################################################################################################################")
            print("####################################################################################################################")
            print("####################################################################################################################")
            print("####################################################################################################################")
            print("####################################################################################################################")

        # planner.plot(path)

        if path:
            traj = np.array(path).T

        plot_data = (
            goal,                             
            obstacles_positions[:2].T,        
            np.array(obstacle_list),
            robot_position,                           
            robot_position,                            
            traj if traj.shape[1] else [[], []],  
            traj if traj.shape[1] else [[], []],  
            0, 0,                             
            goal,                             
            0
        )

        plotter.updatePlot(plot_data)
        time.sleep(DT)

    plotter.stopPlot()

if __name__ == "__main__":
    main()