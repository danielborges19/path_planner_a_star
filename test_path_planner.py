import build.path_planner as path_planner
import time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt 

def plot(start, goal, obstacles, graph, path=None):
    pos = {node: node for node in graph.nodes}
    nx.draw(graph, pos, node_size=20, edge_color='blue')

    _plot_ellipse(start, goal, ellipse_factor=1.1)
    
    if path:
        edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(graph, pos, edgelist=edges, edge_color='red', width=2)
    
    circle1 = plt.Circle(start, 0.28, color='green', alpha=0.5)
    circle2 = plt.Circle(goal, 0.28, color='orange', alpha=0.5)
    plt.gca().add_patch(circle1)
    plt.gca().add_patch(circle2)
    for (x, y, r) in obstacles:
        circle1 = plt.Circle((x, y), r, color='gray', alpha=0.5)
        circle2 = plt.Circle((x, y), 0.28, color='red', alpha=0.5)
        plt.gca().add_patch(circle1)
        plt.gca().add_patch(circle2)
    plt.axis('equal')
    plt.grid()
    plt.show()

def _plot_ellipse(start, goal, ellipse_factor):
    """Desenha a elipse que restringe a área de geração de nós."""
    start = np.array(start)
    goal = np.array(goal)
    
    # Parâmetros da elipse
    c = np.linalg.norm(goal - start) / 2  # Distância do centro a cada foco
    a = ellipse_factor * c  # Semi-eixo maior (soma das distâncias = 2a)
    
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


def find_path(start, goal, graph):
    try:
        return nx.astar_path(graph, start, goal, heuristic=lambda a, b: np.linalg.norm(np.array(a)-np.array(b)))
    except nx.NetworkXNoPath:
        return []


start = (0.0, 0.0)
goal = (10.0, 10.0)
obstacles = [(3.0, 4.0, 1.0), (3.0, 2.0, 1.0), (3.0, 3.0, 1.0), (7.0, 9.0, 1.0), (7.0, 7.0, 1.0)]

st = time.time()
planner = path_planner.PathPlannerGraph(start, goal, obstacles)

planner.generate_graph()

graph = planner.get_graph()

G = nx.Graph()
for node, edges in graph.items():
    for neighbor, weight in edges:
        G.add_edge(node, neighbor, weight=weight)

path = find_path(start, goal, G)

ft = time.time()
print(f"Process Time: {ft-st} | Freq: {1 / (ft-st)}")

# Exemplo de como acessar o grafo
# print("Nodes:", graph.keys())
# for node, edges in graph.items():
#     print(f"Node {node} connects to:")
#     for edge in edges:
#         neighbor, weight = edge
#         print(f"  - {neighbor} with weight {weight}")

plot(start, goal, obstacles, G, path)
