import numpy as np
import math
import random
import matplotlib.pyplot as plt
import copy

# Vanilla rrt.

class Node:
    def __init__(self, pos):
        self.pos = pos
        self.parent = None

class RRT:
    def __init__(self, start, goal, search_space_min=0, search_space_max=50, step_size=1, goal_bias=0.1, max_iteration=1000):
        self.start = Node(start)
        self.goal = Node(goal)
        self.nodes = []
        self.min_x = search_space_min
        self.min_y = search_space_min
        self.max_x = search_space_max
        self.max_y = search_space_max
        self.step_size = step_size
        self.goal_bias = goal_bias
        self.max_iteration = max_iteration

    def plan(self):
        
        # 
        self.initPlot()
        
        # 1) Add start/initial node to node set.
        self.nodes.append(self.start)

        # 2) Iterate...
        for i in range(0, self.max_iteration):
            
            if i % 50 == 0:
                print(i)
            
            # 3) Randomly sample search space.
            # Optionally, to speed up search, a goal bias probability can be set.
            # random_node = self.random_sample()
            if random.random() > self.goal_bias:
                random_node = self.random_sample()
            else:
                random_node = self.goal
            
            # 4) Find nearest node from the node set to the randomly sampled node.
            nearest_node = self.nearest_node(self.nodes, random_node)
            
            # 5) Steer from the randomly sampled node to the nearest node.
            new_node = self.steer(nearest_node, random_node)
            
            # 6) Check if the edge between the new node found above and the nearest node is not in collision, and if yes,
            if self.obstacle_free(nearest_node, new_node):
                
                # 7) Add the new node to the node set, and add the edge between the new node and the nearest node.
                self.nodes.append(new_node)
                
                # * Check if goal is reached, by checking if goal is within a step size distance from new node, and if the connection between the goal and new node is collision free.
                dist = self.get_distance(new_node.pos, self.goal.pos)
                if dist <= self.step_size and self.obstacle_free(new_node, self.goal):
                    self.steer(new_node, self.goal)
                    print("goal!")
                    return self.extract_path(new_node)
                
            # 
            self.plotPoint(new_node.pos[0], new_node.pos[1], '.')


    def random_sample(self):
        random_x = random.uniform(self.min_x, self.max_x) # float random range
        random_y = random.uniform(self.min_y, self.max_y) # float random range
        return Node((random_x,random_y))
    
    def nearest_node(self, nodeList, random_node):
        minDist = math.inf
        # minDistNode = None
        minDistNode = nodeList[0]
        for node in nodeList:
            dist = self.get_distance(node.pos, random_node.pos)
            if dist < minDist:
                minDist = dist
                minDistNode = node
        return minDistNode

    def steer(self, nearest_node, random_node):
        angle = self.get_angle(nearest_node.pos, random_node.pos)
        # 
        new_node = nearest_node
        x = new_node.pos[0] + self.step_size * math.cos(angle)
        y = new_node.pos[1] + self.step_size * math.sin(angle)
        new_node = Node((x, y))
        # Set parent of this new node as the nearest node.
        new_node.parent = nearest_node
        return new_node

    def get_distance(self, pos1, pos2):
        dist = math.sqrt(math.pow(pos2[0] - pos1[0], 2) + math.pow(pos2[1] - pos1[1], 2))
        return dist

    def get_angle(self, node_start_pos, node_end_pos):
        # Get Deltas of X & y between the nearest node and randomly sampled node.
        # * It must be random node pos - neareast!
        dx = node_end_pos[0] - node_start_pos[0]
        dy = node_end_pos[1] - node_start_pos[1]
        # Use the arc tan of the deltas in y and x to find the angle between nearest node and randomly sampled node.
        return math.atan2(dy, dx)

    def obstacle_free(self, nearest_node, new_node):
        # ...
        return True
    
    def extract_path(self, node_end):
        path = [(self.goal.pos[0], self.goal.pos[1])]
        node_now = node_end
        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.pos[0], node_now.pos[1]))
        return path

    def initPlot(self):
        plt.ion()
        plt.show()

    def plotPoint(self, x_pos, y_pos, str='o'):
        plt.plot(x_pos, y_pos, str)
        plt.draw()
        plt.pause(0.001)

    def plotPath(self, path):
        if len(path) != 0:
            plt.plot([x[0] for x in path], [x[1] for x in path], '-r', linewidth=1)
        plt.ioff()
        plt.show()


def main():
    start = (0,0)
    goal = (40,40)
    search_space_min = 0
    search_space_max = 50
    step_size = 1
    goal_bias = 0.1
    max_iteration = 1000
    rrt = RRT(start, goal, search_space_min, search_space_max, step_size, goal_bias, max_iteration)
    path = rrt.plan()
    rrt.plotPoint(start[0], start[0], 'go')
    rrt.plotPoint(goal[0], goal[1], 'ro')
    rrt.plotPath(path)
    
if __name__ == "__main__":
    main()
