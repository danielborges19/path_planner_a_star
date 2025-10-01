from matplotlib import patches
import build.path_planner as path_planner
import time
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt 
import matplotlib.lines as mlines

def plot(start, goal, obstacles, graph, path=None):
    A = 17.00 
    B = 10.50 
    C = 6.50 
    D = 3.50 
    E = 2.25 
    F = 0.75 
    H = 3.50 
    I = 3.00 
    J = 0.15 
        

    field_length = A
    field_width = B

    plt.gca().set_xlim(-field_length/2 - 1, field_length/2 + 1)
    plt.gca().set_ylim(-field_width/2 - 1, field_width/2 + 1)


    # Robot
    # plt.gca().set_facecolor("green")
    outer_rect = plt.Rectangle((-field_length/2, -field_width/2), field_length, field_width, linewidth=2, edgecolor="green", facecolor="green", alpha=1)
    plt.gca().add_patch(outer_rect)


    # Gols
    plt.gca().add_patch(plt.Rectangle((-field_length/2 - 0.5, -1.2), 0.5, 2.4, facecolor="silver", alpha=1))
    plt.gca().add_patch(plt.Rectangle((field_length/2, -1.2), 0.5, 2.4, facecolor="silver", alpha=1))

    outer_rect = plt.Rectangle((-field_length/2, -field_width/2), field_length, field_width, linewidth=2, edgecolor="white", facecolor="none", alpha=0.75)
    plt.gca().add_patch(outer_rect)

    plt.gca().add_patch(plt.Rectangle([0, -field_width/2], 0, field_width, color="white", linewidth=2, alpha=1))

    # Círculo central
    center_circle = plt.Circle((0, 0), H/2, linewidth=2, edgecolor="white", facecolor="none", alpha=1)
    plt.gca().add_patch(center_circle)
    plt.gca().add_patch(plt.Circle((0, 0), J, color="white", alpha=1))  # ponto central

    # Áreas do gol 
    area_length = E
    area_width = C
    plt.gca().add_patch(plt.Rectangle((-field_length/2, -area_width/2), area_length, area_width, linewidth=2, edgecolor="white", facecolor="none", alpha=1))
    plt.gca().add_patch(plt.Rectangle((field_length/2 - area_length, -area_width/2), area_length, area_width, linewidth=2, edgecolor="white", facecolor="none", alpha=1))

    # Pequena área do gol
    small_area_length = F
    small_area_width = D
    plt.gca().add_patch(plt.Rectangle((-field_length/2, -small_area_width/2), small_area_length, small_area_width, linewidth=2, edgecolor="white", facecolor="none", alpha=1))
    plt.gca().add_patch(plt.Rectangle((field_length/2 - small_area_length, -small_area_width/2), small_area_length, small_area_width, linewidth=2, edgecolor="white", facecolor="none", alpha=1))

    # Pontos de pênalti
    penalty_point = A/2 - I
    plt.gca().add_patch(plt.Circle((-penalty_point, 0), J, color="white", alpha=1))
    plt.gca().add_patch(plt.Circle((penalty_point, 0), J, color="white", alpha=1))
    
    pos = {node: node for node in graph.nodes}
    
    _plot_ellipse(start, goal, ellipse_factor=1.2)
    
    
    circle1 = plt.Circle(start, 0.28, color='orange', alpha=1)
    circle2 = plt.Circle(goal, 0.28, color='black', alpha=1)
    plt.gca().add_patch(circle1)
    plt.gca().add_patch(circle2)
    for i, (x, y, r) in enumerate(obstacles):
        if i == 0:
            circle1 = plt.Circle((x, y), r, color='gray', alpha=1)
            circle2 = plt.Circle((x, y), 0.28, color='red', alpha=1.0)
            plt.gca().add_patch(circle1)
            plt.gca().add_patch(circle2)
        else:
            circle1 = plt.Circle((x, y), r, color='gray', alpha=1)
            circle2 = plt.Circle((x, y), 0.28, color='red', alpha=1.0)
            plt.gca().add_patch(circle1)
            plt.gca().add_patch(circle2)
    
    nx.draw_networkx_nodes(graph, pos, node_size=20, node_color="blue", label='Nós')

    nx.draw(graph, pos, node_size=0, edge_color='royalblue')

    if path:
        edges = list(zip(path, path[1:]))
        nx.draw_networkx_edges(graph, pos, edgelist=edges, edge_color='red', width=3)


    # legend_handles = [
    #     mlines.Line2D([], [], color='orange', marker='o', linestyle='None',
    #                 markersize=10, label='Posição do Robô'),
    #     mlines.Line2D([], [], color='green', marker='o', linestyle='None',
    #                 markersize=10, label='Posição Objetivo'),
    #     mlines.Line2D([], [], color='gray', marker='o', linestyle='None',
    #                 markersize=10, alpha=0.5, label='Área de Segurança'),
    #     mlines.Line2D([], [], color='red', marker='o', linestyle='None',
    #                 markersize=10, label='Obstáculo'),
    #     mlines.Line2D([], [], color='blue', marker='o', linestyle='None',
    #                 markersize=10, label='Nós'),
    # ]

    # plt.legend(handles=legend_handles, loc="upper right")

    plt.axis('equal')
    # plt.grid()
    # plt.legend()
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
    plt.plot(ellipse_x, ellipse_y, 'r--', linewidth=1, alpha=0.7)
    plt.legend()


def find_path(start, goal, graph):
    try:
        return nx.astar_path(graph, start, goal, heuristic=lambda a, b: np.linalg.norm(np.array(a)-np.array(b)))
    except nx.NetworkXNoPath:
        return []


start = (0.0, 0.0)
goal = (7.5, 0.0)
obstacles = [(3.0, 0.0, 0.75), (7.0, 0.5, 0.75), (7.0, -0.5, 0.75), (5.0, 3.0, 0.75)] #, (7.0, 7.0, 1.0)]

st = time.time()
planner = path_planner.PathPlannerGraph()

graph = planner.generateGraph(start, goal, obstacles)


G = nx.Graph()
for node, edges in graph.items():
    for neighbor, weight in edges:
        G.add_edge(node, neighbor, weight=weight)

path = find_path(start, goal, G)

# print("Vizinhos do robô:", list(G.neighbors(start)))

ft = time.time()
print(f"Process Time: {ft-st} | Freq: {1 / (ft-st)}")

# Exemplo de como acessar o grafo
print("Nodes:", graph.keys())
for node, edges in graph.items():
    print(f"Node {node} connects to:")
    for edge in edges:
        neighbor, weight = edge
        print(f"  - {neighbor} with weight {weight}")

plot(start, goal, obstacles, G, path)
