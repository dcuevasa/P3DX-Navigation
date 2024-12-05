import rclpy
import numpy as np
import heapq
from rclpy.node import Node
import matplotlib.pyplot as plt
import readline
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
import math
import time
import threading


class PotentialFieldPlanner(Node):
    def __init__(self):
        super().__init__("potential_field_planner")
        
        self.get_logger().info("Potential Field Planner Node started.")
        scene_file = self.rlinput("Insert file:","./src/path_generator/path_generator/Escena-Problema1.txt")
        self.resolution = 5  # Grid resolution in cm
        self.robot_size = 30  # Robot size in cm

        self.dimensions_x = 0
        self.dimensions_y = 0

        self.grid = []

        self.lecturas_filtradas = []
        self.q0 = [0, 0, 0]  # Initial robot position
        self.qf = [0, 0, 0]
        self.number_of_obstacles = 0
        self.obstacles = []
        self.directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

        self.current_position = [0, 0, 0]
        
        self.load_scene(scene_file)
        print("Scene loaded")
        self.path = self.a_star((self.q0[0],self.q0[1]), (self.qf[0],self.qf[1]))
        self.visualize_grid()
        print("path: ", self.path)
        
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/model/pioneer3dx/cmd_vel", 10)
        self.pose_subscriber = self.create_subscription(PoseArray, "/world/pioneer3_world/pose/info", self.callback_pose, 10)
        self.sensor_subscriber = self.create_subscription(LaserScan, "/lidar", self.callback_sensor, 10)
        
        # Ejecutar ruta después de inicializar todo
        time.sleep(3)
        execute_thread = threading.Thread(target=self.execute_path)
        execute_thread.start() 

        

    def execute_path(self):
        """Ejecuta la ruta planificada"""
        linear_tolerance = 13
        linear_velocity = 0.15
        directions = self.find_directions(self.path)
        angle_threshold = 0.1  # Threshold in radians

        if abs(self.current_position[2] - math.radians(0)) < angle_threshold:
            self.current_direction = "up"
        elif abs(self.current_position[2] - math.radians(180)) < angle_threshold:
            self.current_direction = "down"
        elif abs(self.current_position[2] - math.radians(90)) < angle_threshold:
            self.current_direction = "left"
        elif abs(self.current_position[2] - (-math.radians(90))) < angle_threshold:
            self.current_direction = "right"
        if self.current_position[2] == math.radians(0):
            self.current_direction = "up"
        elif self.current_position[2] == math.radians(180):
            self.current_direction = "down"
        elif self.current_position[2] == math.radians(90):
            self.current_direction = "left"
        elif self.current_position[2] == -math.radians(90):
            self.current_direction = "right"
        for i in range(len(self.path)-1):
            point = self.path[i]
            print(f"Current position: {self.current_position}")
            print(f"Moving to point {point}")
            print(f"Direction: {directions[i]}")
            if directions[i] != self.current_direction:
                print("Rotating to direction", directions[i])
                print("Current direction", self.current_direction)
                self.rotato_to_direction(directions[i])
                self.current_direction = directions[i]
            self.publish_velocity(0, 0)
            if directions[i] == "up" or directions[i] == "down":
                while abs(point[0] - self.current_position[0]) > linear_tolerance:
                    self.publish_velocity(linear_velocity, 0)
                    print("Current position: ", self.current_position)
                    print("Point: ", point)
            elif directions[i] == "left" or directions[i] == "right":
                while abs(point[1] - self.current_position[1]) > linear_tolerance:
                    self.publish_velocity(linear_velocity, 0)
                    print("Current position: ", self.current_position)
                    print("Point: ", point)
            print("Point reached")
            self.publish_velocity(0, 0)
        print("Path completed")
        #final direction
        final_direction = "up"
        if self.qf[2] == 0:
            final_direction = "up"
        elif self.qf[2] == 180:
            final_direction = "down"
        elif self.qf[2] == 90:
            final_direction = "left"
        elif self.qf[2] == -90:
            final_direction = "right"
        #current direction
        current_direction = "up"
        if abs(self.current_position[2] - math.radians(0)) < angle_threshold:
            current_direction = "up"
        elif abs(self.current_position[2] - math.radians(180)) < angle_threshold:
            current_direction = "down"
        elif abs(self.current_position[2] - math.radians(90)) < angle_threshold:
            current_direction = "left"
        elif abs(self.current_position[2] - (-math.radians(90))) < angle_threshold:
            current_direction = "right"
        #rotate to qf
        if final_direction != current_direction:
            self.rotato_to_direction(final_direction)
        threshold = 1
        print("lecturas",self.lecturas_filtradas)
        print("dfrente",self.dFrente)
        print("dderecha",self.dDerecha)
        # Calculate actual qf position based on lidar readings
        actual_qf = [0, 0, 0]
        actual_qf[0] = self.dimensions_x - self.lecturas_filtradas[4] * 100  # Front distance
        actual_qf[1] = self.dimensions_y - self.lecturas_filtradas[6] * 100  # Right distance
        actual_qf[2] = self.current_position[2]  # Current orientation

        self.qf = actual_qf
        self.get_logger().info(f"Expected qf: {self.qf}")
        self.get_logger().info(f"Actual qf: {actual_qf}")

        with open('path.txt', 'w') as f:
            for point in self.path:
                f.write(f"{point[0]},{point[1]}\n")
            f.write(f"{self.qf[0]},{self.qf[1]}\n")
            f.write(f"{actual_qf[0]},{actual_qf[1]}\n")
        if abs(self.lecturas_filtradas[4] - self.dFrente) < threshold and abs(self.lecturas_filtradas[6] - self.dDerecha) < threshold:
            self.get_logger().info("Lidar detection matches both dFrente and dDerecha within threshold")
        elif abs(self.lecturas_filtradas[4] - self.dFrente) < threshold:
            self.get_logger().info("Lidar detection matches dFrente within threshold")
        elif abs(self.lecturas_filtradas[6] - self.dDerecha) < threshold:
            self.get_logger().info("Lidar detection matches dDerecha within threshold")
        else:
            self.get_logger().info("Lidar detection does not match dFrente or dDerecha within threshold")

        self.get_logger().info("Path executed successfully")

        self.path = self.path[::-1]  # Invert path list
        qf_temp = self.qf
        self.qf = self.q0  # Set final position as the initial position
        self.q0 = qf_temp  # Set initial position as the final position
        
        directions = self.find_directions(self.path)

        if abs(self.current_position[2] - math.radians(0)) < angle_threshold:
            self.current_direction = "up"
        elif abs(self.current_position[2] - math.radians(180)) < angle_threshold:
            self.current_direction = "down"
        elif abs(self.current_position[2] - math.radians(90)) < angle_threshold:
            self.current_direction = "left"
        elif abs(self.current_position[2] - (-math.radians(90))) < angle_threshold:
            self.current_direction = "right"
        if self.current_position[2] == math.radians(0):
            self.current_direction = "up"
        elif self.current_position[2] == math.radians(180):
            self.current_direction = "down"
        elif self.current_position[2] == math.radians(90):
            self.current_direction = "left"
        elif self.current_position[2] == -math.radians(90):
            self.current_direction = "right"
        for i in range(len(self.path)-1):
            point = self.path[i]
            print(f"Current position: {self.current_position}")
            print(f"Moving to point {point}")
            print(f"Direction: {directions[i]}")
            if directions[i] != self.current_direction:
                print("Rotating to direction", directions[i])
                print("Current direction", self.current_direction)
                self.rotato_to_direction(directions[i])
                self.current_direction = directions[i]
            self.publish_velocity(0, 0)
            if directions[i] == "up" or directions[i] == "down":
                while abs(point[0] - self.current_position[0]) > linear_tolerance:
                    self.publish_velocity(linear_velocity, 0)
                    print("Current position: ", self.current_position)
                    print("Point: ", point)
            elif directions[i] == "left" or directions[i] == "right":
                while abs(point[1] - self.current_position[1]) > linear_tolerance:
                    self.publish_velocity(linear_velocity, 0)
                    print("Current position: ", self.current_position)
                    print("Point: ", point)
            print("Point reached")
            self.publish_velocity(0, 0)
        print("Path completed")
        #final direction
        final_direction = "up"
        if self.qf[2] == 0:
            final_direction = "up"
        elif self.qf[2] == 180:
            final_direction = "down"
        elif self.qf[2] == 90:
            final_direction = "left"
        elif self.qf[2] == -90:
            final_direction = "right"
        #current direction
        current_direction = "up"
        if abs(self.current_position[2] - math.radians(0)) < angle_threshold:
            current_direction = "up"
        elif abs(self.current_position[2] - math.radians(180)) < angle_threshold:
            current_direction = "down"
        elif abs(self.current_position[2] - math.radians(90)) < angle_threshold:
            current_direction = "left"
        elif abs(self.current_position[2] - (-math.radians(90))) < angle_threshold:
            current_direction = "right"
        #rotate to qf
        if final_direction != current_direction:
            self.rotato_to_direction(final_direction)
        threshold = 1

        with open('path2.txt', 'w') as f:
            for point in self.path:
                f.write(f"{point[0]},{point[1]}\n")
            f.write(f"{self.qf[0]},{self.qf[1]}\n")
            f.write(f"{actual_qf[0]},{actual_qf[1]}\n")
        

    def callback_pose(self, msg):
        """Actualiza la posición actual del robot"""
        poses = msg.poses
        robot_pose = poses[1]
        x = robot_pose.position.x * 100
        y = robot_pose.position.y * 100
        orientation = robot_pose.orientation
        qz = orientation.z
        qw = orientation.w
        theta = 2 * math.atan2(qz, qw)
        self.current_position = [x, y, theta]
        #self.get_logger().info("Posición del robot: ({:.2f}, {:.2f}, {:.2f})".format(x, y, theta))

    def rotate_to_angle(self, angle):
        """Rotate to a specific angle in radians."""
        current_theta = self.current_position[2]
        angle_difference = (angle - current_theta + math.pi) % (2 * math.pi) - math.pi
        angular_velocity = 0.15
        angle_tolerance = 0.02
        while abs(angle_difference) > angle_tolerance:
            if angle_difference > 0:
                self.publish_velocity(0, -angular_velocity)
            else:
                self.publish_velocity(0, angular_velocity)
            current_theta = self.current_position[2]
            angle_difference = current_theta - angle
            #print("Current angle: ", current_theta)
            #print("angle: ", angle)
        self.publish_velocity(0, 0)
        
    def rotato_to_direction(self, direction):
        """Rotate to a specific direction."""
        if direction == "up":
            angle = math.radians(0)
        elif direction == "down":
            angle = math.radians(180)
        elif direction == "left":
            angle = math.radians(90)
        elif direction == "right":
            angle = -math.radians(90)
        self.rotate_to_angle(angle)

    def publish_velocity(self, x, z):
        msg = Twist()
        # ASEGURARSE DE LIMITAR LAS MAGNITUDES DE VELOCIDADES A 0.5
        msg.linear.x = float(x)
        msg.angular.z = float(z)
        self.cmd_vel_publisher_.publish(msg)

    
    def find_directions(self,path):
        directions = []
        for i in range(len(path) - 1):
            point1 = path[i]
            point2 = path[i + 1]

            if point2[1] > point1[1]:
                directions.append("left")
            elif point2[1] < point1[1]:
                directions.append("right")
            elif point2[0] > point1[0]:
                directions.append("up")
            elif point2[0] < point1[0]:
                directions.append("down")

        return directions

    def load_scene(self, scene_file):
        """Loads the scene from a text file."""
        with open(scene_file, "r") as file:
            lines = file.readlines()

        dimensions = lines[0].strip().split(",")
        self.dimensions_x = int(float(dimensions[1])*100)+100 #convert from meters to cm to use as ints
        self.dimensions_y = int(float(dimensions[2])*100)+100 #convert from meters to cm to use as ints

        q0_line = lines[1].strip().split(",")
        # Initial robot position x,y,theta
        self.q0 = [int(float(q0_line[1])*100), int(float(q0_line[2])*100), int(float(q0_line[3])*100)] #convert from meters to cm to use as ints

        qf_line = lines[2].strip().split(",")
        # Final robot position x,y,theta
        self.qf = [int(float(qf_line[1])*100), int(float(qf_line[2])*100), int(float(qf_line[3])*100)] #convert from meters to cm to use as ints

        self.dFrente = float(lines[3].strip().split(",")[1])

        self.dDerecha = float(lines[4].strip().split(",")[1])
        self.number_of_obstacles = int(lines[5].strip().strip().split(",")[1])
        
        
        self.grid = np.zeros((self.dimensions_x, self.dimensions_y))
        self.grid = [[0 for _ in range(self.dimensions_y)] for _ in range(self.dimensions_x)]
        #Build grid
        for i in range(0,self.dimensions_x, self.resolution):
            for j in range(0,self.dimensions_y, self.resolution):
                self.grid[i][j] = 0
        
        #Obstacles
        # Add borders with padding
        border_padding = 40  # padding from the actual dimensions
        # Top and bottom borders
        for x in range(border_padding, self.dimensions_x - border_padding):
            for y in range(border_padding, border_padding + self.robot_size):
                self.grid[x][y] = 1  # Bottom border
            for y in range(self.dimensions_y - border_padding - self.robot_size, self.dimensions_y - border_padding):
                self.grid[x][y] = 1  # Top border

        # Left and right borders
        for y in range(border_padding, self.dimensions_y - border_padding):
            for x in range(border_padding, border_padding + self.robot_size):
                self.grid[x][y] = 1  # Left border
            for x in range(self.dimensions_x - border_padding - self.robot_size, self.dimensions_x - border_padding):
                self.grid[x][y] = 1  # Right border

        #Each obstacle is defined by two lines
        for i in range(0,self.number_of_obstacles*2,2):
            print("Obstacle ", int(i/2)+1)
            print("Reading lines: ", 6 + i, " and ", 6 + i + 1)
            obstacle_edge1_line = lines[6 + i].strip().split(",")
            obstacle_edge1_coord = [int(float(obstacle_edge1_line[1])*100), int(float(obstacle_edge1_line[2])*100)]
            obstacle_edge2_line = lines[6 + i + 1].strip().split(",")
            obstacle_edge2_coord = [int(float(obstacle_edge2_line[1])*100), int(float(obstacle_edge2_line[2])*100)]
            extra_padding = int(self.robot_size)
            for x in range(obstacle_edge1_coord[0]-extra_padding, obstacle_edge2_coord[0]+extra_padding, self.resolution):
                for y in range(obstacle_edge1_coord[1]-extra_padding, obstacle_edge2_coord[1]+extra_padding, self.resolution):
                    print("obstacle at: ", x, y)
                    self.grid[x][y] = 1
                    

    def rlinput(self,prompt, prefill=''):
        readline.set_startup_hook(lambda: readline.insert_text(prefill))
        try:
            return input(prompt)  # or raw_input in Python 2
        finally:
            readline.set_startup_hook()

    def visualize_grid(self):
        # Visualize the grid using matplotlib
        plt.imshow(self.grid, cmap='Blues', interpolation='nearest')
        plt.colorbar()  # Add a colorbar to indicate values
        plt.title("Grid with Obstacles")

        # If a path exists, plot it on top of the grid
        if self.path:
            path_x = [point[1] for point in self.path]  # Get x-coordinates
            path_y = [point[0] for point in self.path]  # Get y-coordinates
            plt.plot(path_x, path_y, color='red', linewidth=2, label="Path")

        plt.legend()  # Add a legend to explain the path
        plt.show()


    def heuristic(self, a, b):
        """Calculate Manhattan distance."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    
    def a_star(self, start, goal):
        """Find the shortest path from start to goal avoiding obstacles."""
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(start, goal), 0, start))  # (f_cost, g_cost, node)
        came_from = {}
        g_costs = {start: 0}
        f_costs = {start: self.heuristic(start, goal)}

        while open_set:
            _, current_cost, current_node = heapq.heappop(open_set)

            # If we reached the goal
            if current_node == goal:
                path = []
                while current_node in came_from:
                    path.append(current_node)
                    current_node = came_from[current_node]
                path.reverse()  # Reverse the path
                return path

            # Explore neighbors
            for dx, dy in self.directions:
                neighbor = (current_node[0] + dx * self.resolution, current_node[1] + dy * self.resolution)
                
                # Check if neighbor is within bounds and not an obstacle
                if (0 <= neighbor[0] < len(self.grid) and 0 <= neighbor[1] < len(self.grid[0]) and 
                        self.grid[neighbor[0]][neighbor[1]] == 0):
                    tentative_g_cost = current_cost + 1  # Moving cost is 1 per step
                    
                    if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                        g_costs[neighbor] = tentative_g_cost
                        f_costs[neighbor] = tentative_g_cost + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_costs[neighbor], tentative_g_cost, neighbor))
                        came_from[neighbor] = current_node

        return None  # No path found

    def callback_sensor(self, msg):
        lecturas = msg.ranges
        self.lecturas_filtradas = lecturas[1:8]

def main(args=None):
    rclpy.init(args=args)
    node = PotentialFieldPlanner()
    executor = rclpy.executors.MultiThreadedExecutor()

    # Add the node to the executor
    executor.add_node(node)

    # Use a separate thread for spinning the executor
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
