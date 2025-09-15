import numpy as np
import time
import pygame

class FieldPlotter:
    def __init__(self, numObstacles=20, N=15, PRINT=True):
        pygame.init()
        self.PRINT = PRINT

        if self.PRINT:
            # Campo real em metros
            self.field_length = 240  # metros (-110 a 110)
            self.field_width = 160   # metros (-70 a 70)
            self.numObstacles = numObstacles
            self.N = N

            # Tamanho da janela
            self.width, self.height = 1200, 800
            self.screen = pygame.display.set_mode((self.width, self.height))
            pygame.display.set_caption("Campo de Futebol Robótico")

            # Escalas para conversão metros -> pixels
            self.scale_x = self.width / self.field_length
            self.scale_y = self.height / self.field_width

            self.clock = pygame.time.Clock()
            self.running = True

    def to_pixel(self, x, y):
        """Converte (x, y) em metros para pixels na tela."""
        px = int((x + self.field_length/2) * self.scale_x) 
        py = int((self.field_width/2 - y) * self.scale_y) 
        return px, py

    def drawField(self):
        # Fundo
        self.screen.fill((0, 128, 0))

        # Gols
        pygame.draw.rect(self.screen, (192, 192, 192),
            (*self.to_pixel(-115, 12), self.scale_x*5, self.scale_y*24))
        pygame.draw.rect(self.screen, (192, 192, 192),
            (*self.to_pixel(110, 12), self.scale_x*5, self.scale_y*24))

        # Linhas do campo
        pygame.draw.rect(self.screen, (255, 255, 255),
            (*self.to_pixel(-110, 70), self.scale_x*220, self.scale_y*140), 2)
        pygame.draw.line(self.screen, (255, 255, 255),
            self.to_pixel(0, 70), self.to_pixel(0, -70), 2)

        # Círculo central e ponto central
        pygame.draw.circle(self.screen, (255, 255, 255), 
                          self.to_pixel(0, 0), int(self.scale_x*20), 2)
        pygame.draw.circle(self.screen, (255, 255, 255), 
                          self.to_pixel(0, 0), int(self.scale_x*1.5), 0)

        # Áreas do gol
        pygame.draw.rect(self.screen, (255, 255, 255),
            (*self.to_pixel(-110, 35.5), self.scale_x*22.5, self.scale_y*71), 2)
        pygame.draw.rect(self.screen, (255, 255, 255),
            (*self.to_pixel(87.5, 35.5), self.scale_x*22.5, self.scale_y*71), 2)

        # Pequena área do gol
        pygame.draw.rect(self.screen, (255, 255, 255),
            (*self.to_pixel(-110, 19.5), self.scale_x*7.5, self.scale_y*39), 2)
        pygame.draw.rect(self.screen, (255, 255, 255),
            (*self.to_pixel(102.5, 19.5), self.scale_x*7.5, self.scale_y*39), 2)

        # Marcação de pênalti
        pygame.draw.circle(self.screen, (255, 255, 255), 
                          self.to_pixel(-74, 0), int(self.scale_x*1.5), 0)
        pygame.draw.circle(self.screen, (255, 255, 255), 
                          self.to_pixel(74, 0), int(self.scale_x*1.5), 0)

    def plotStartFinishPositions(self, finish):
        pygame.draw.circle(self.screen, (0,0,139), 
                          self.to_pixel(finish[0]*10, finish[1]*10), 
                          int(self.scale_x*1.5), 0)

    def plotObstacles(self, init_obstacles_positions, obstacles_positions):
        
            for obs_p in init_obstacles_positions:
                pygame.draw.circle(
                    self.screen,
                    (255, 1, 32),  # Cor sólida
                    self.to_pixel(obs_p[0] * 10, obs_p[1] * 10),
                    int(self.scale_x * 2.8),
                    0  # preenchido
                )

            # Obstáculos previstos (posições futuras)
            for obs_p in obstacles_positions:
                pygame.draw.circle(
                    self.screen,
                    (255, 1, 32),  
                    self.to_pixel(obs_p[0] * 10, obs_p[1] * 10),
                    int(obs_p[2]* self.scale_x * 10),
                    2  # apenas contorno
                )

    def plotRobot(self, robotPosition, realrobotPosition, robotPositionGoal):
        pygame.draw.circle(self.screen, (192,192,192), 
                          self.to_pixel(realrobotPosition[0]*10, realrobotPosition[1]*10), 
                          int(self.scale_x*2.8), 0)
        pygame.draw.circle(self.screen, (0,0,0), 
                          self.to_pixel(robotPosition[0]*10, robotPosition[1]*10), 
                          int(self.scale_x*2.8), 0)

    def plotTree(self, tree):

        for node in tree:
            if node.parent is not None:
                parent = tree[node.parent]
                # plt.plot([node.x, parent.x], [node.y, parent.y], "-g")
                x1, y1 = [node.x, node.y]
                x2, y2 = [parent.x, parent.y]
                p1 = self.to_pixel(x1 * 10, y1 * 10)
                p2 = self.to_pixel(x2 * 10, y2 * 10)
                pygame.draw.line(self.screen, (150, 150, 150), p1, p2, 1)
                
        # for node in tree:
        #     if node.parent is not None:
        #         x1, y1 = node.position
        #         x2, y2 = node.parent.position
        #         p1 = self.to_pixel(x1 * 10, y1 * 10)
        #         p2 = self.to_pixel(x2 * 10, y2 * 10)
        #         pygame.draw.line(self.screen, (150, 150, 150), p1, p2, 1)

    def plotTrajectory(self, trajectory_estimated, trajectory_estimated_follower, 
                      set_point_v, desired_velocity):
        # color = [0, 100, 0]
        # for i in range(len(trajectory_estimated[0])-1):
        #     color[0] = min(color[0]+100,255)
        #     color[2] = min(color[2]+25,255)
        #     pygame.draw.circle(self.screen, color, 
        #                      self.to_pixel(trajectory_estimated[0][i+1]*10, trajectory_estimated[1][i+1]*10), 
        #                      int(self.scale_x*2.8), 2)

        # color = [0, 100, 0]
        # for i in range(len(trajectory_estimated_follower[0])-1):
        #     color[0] = min(color[0]+100,255)
        #     color[2] = min(color[2]+25,255)
        #     pygame.draw.circle(self.screen, color, 
        #                      self.to_pixel(trajectory_estimated_follower[0][i+1]*10, trajectory_estimated_follower[1][i+1]*10), 
        #                      int(self.scale_x*1.0), 2)

         # Trajetória estimada principal (linha verde)
        if len(trajectory_estimated[0]) > 1:
            points = [self.to_pixel(x * 10, y * 10) for x, y in zip(trajectory_estimated[0], trajectory_estimated[1])]
            pygame.draw.lines(self.screen, (0, 255, 0), False, points, 3)

        # Trajetória do seguidor (linha azul)
        if len(trajectory_estimated_follower[0]) > 1:
            points = [self.to_pixel(x * 10, y * 10) for x, y in zip(trajectory_estimated_follower[0], trajectory_estimated_follower[1])]
            pygame.draw.lines(self.screen, (0, 0, 255), False, points, 2)
                
    def plotSpeedMap(self, x_mesh, y_mesh, speed_map, MESH_MIN_V, MESH_MAX_V):
        """Plot the speed map as a semi-transparent overlay on the field."""
        # Create a surface for the speed map
        speed_surface = pygame.Surface((self.width, self.height), pygame.SRCALPHA)
        
        # Normalize speed values to 0-255 range
        normalized_speed = (speed_map - MESH_MIN_V) / (MESH_MAX_V - MESH_MIN_V) * 255
        
        # Convert mesh coordinates to pixel coordinates
        x_pixels = ((x_mesh * 10 + self.field_length/2) * self.scale_x ).astype(int)
        y_pixels = ((self.field_width/2 - y_mesh * 10) * self.scale_y ).astype(int)
        
        # Get dimensions
        rows, cols = speed_map.shape
        
        # Draw each cell with appropriate color
        for i in range(rows - 1):
            for j in range(cols - 1):
                points = [
                    (x_pixels[i, j], y_pixels[i, j]),
                    (x_pixels[i+1, j], y_pixels[i+1, j]),
                    (x_pixels[i+1, j+1], y_pixels[i+1, j+1]),
                    (x_pixels[i, j+1], y_pixels[i, j+1])
                ]
                speed_val = int(np.mean(normalized_speed[i:i+2, j:j+2]))  # Média simples

                
                # Convert speed to a color (using plasma-like colormap)
                if speed_val < 64:
                    r = int(13 * speed_val / 64)
                    g = 0
                    b = int(135 * speed_val / 64 + 120 * (64 - speed_val) / 64)
                elif speed_val < 128:
                    r = int(240 * (speed_val - 64) / 64 + 13 * (128 - speed_val) / 64)
                    g = int(2 * (speed_val - 64) / 64)
                    b = int(120 * (128 - speed_val) / 64)
                elif speed_val < 192:
                    r = 240 + int(15 * (speed_val - 128) / 64)
                    g = 2 + int(222 * (speed_val - 128) / 64)
                    b = 0
                else:
                    r = 255
                    g = 227 + int(28 * (speed_val - 192) / 64)
                    b = 0 + int(255 * (speed_val - 192) / 64)
                
                # Draw the polygon with some transparency
                pygame.draw.polygon(speed_surface, (r, g, b, 255), points)
        
        # Blit the speed surface onto the screen
        self.screen.blit(speed_surface, (0, 0))
        
    def stopPlot(self):

        pygame.quit()

    def updatePlot(self, plot_data):
        
        for pygame_event in pygame.event.get():
            if pygame_event.type == pygame.QUIT:
                self.running = False
        
        if self.PRINT:
            finish, init_obstacles_positions, obstaclesPositions,  \
            robotPosition, realrobotPosition, trajectory_estimated, trajectory_estimated_follower, \
            set_point_v, desired_velocity, robotPositionGoal, node_list = plot_data
            
            
            #self.plotSpeedMap(x_mesh, y_mesh, speed_map, MESH_MIN_V, MESH_MAX_V)  # Add this line
            self.drawField()

            self.plotStartFinishPositions(finish)
            self.plotObstacles(init_obstacles_positions, obstaclesPositions)
            self.plotRobot(robotPosition, realrobotPosition, robotPositionGoal)
            # self.plotTree(node_list)
            self.plotTrajectory(trajectory_estimated, trajectory_estimated_follower, 
                                set_point_v, desired_velocity)
            
            pygame.display.flip()
            
            self.clock.tick(30)

            

