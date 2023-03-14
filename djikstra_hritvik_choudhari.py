import numpy as np
import matplotlib.pyplot as plt
import cv2

# Class containing node attributes
class createNode:
    def __init__(self, pos, parent, c2c):
        self.pos = pos
        self.parent = parent
        self.c2c = c2c

# Function to move node in a particular direction
def Direction(dir, node):
    if dir == 'South':
        cost = 1
        x, y = node.pos
        child_created = createNode((x, y - 1), node, node.c2c + cost)
        return cost, child_created

    if dir == 'North':
        cost = 1
        x, y = node.pos
        child_created = createNode((x, y + 1), node, node.c2c + cost)
        return cost, child_created

    if dir == 'West':
        cost = 1
        x, y = node.pos
        child_created = createNode((x - 1, y), node, node.c2c + cost)
        return cost, child_created

    if dir == 'East':
        cost = 1
        x, y = node.pos
        child_created = createNode((x + 1, y), node, node.c2c + cost)
        return cost, child_created

    if dir == 'North_West':
        cost = 1.4
        x, y = node.pos
        child_created = createNode((x - 1, y + 1), node, node.c2c + cost)
        return cost, child_created

    if dir == 'North_East':
        cost = 1.4
        x, y = node.pos
        child_created = createNode((x + 1, y + 1), node, node.c2c + cost)
        return cost, child_created

    if dir == 'South_West':
        cost = 1.4
        x, y = node.pos
        child_created = createNode((x - 1, y - 1), node, node.c2c + cost)
        return cost, child_created

    if dir == 'South_East':
        cost = 1.4
        x, y = node.pos
        child_created = createNode((x + 1, y - 1), node, node.c2c + cost)
        return cost, child_created

# Function to create obstacles in a grid of given size
def Obstacle_space(dims):
    w, h = dims
    angle = np.deg2rad(30)
    grid = np.zeros((h + 1, w + 1, 3), dtype=np.uint8)
    grid.fill(255)
    rect1 = np.array([[100, 150], [150, 150], [150, 250], [100, 250]])
    rect2 = np.array([[100, 100], [150, 100], [150, 0], [100, 0]])
    triangle = np.array([[460, 25], [510, 125], [460, 225]])
    hexagon = np.array([[300, 200], [300 + 75 * np.cos(angle), 200 - 75 * np.sin(angle)],
                        [300 + 75 * np.cos(angle), 50 + 75 * np.sin(angle)], [300, 50],
                        [300 - 75 * np.cos(angle), 50 + 75 * np.sin(angle)],
                        [300 - 75 * np.cos(angle), 200 - 75 * np.sin(angle)]]).astype(int)
    grid = cv2.fillPoly(grid, pts=[rect1, rect2, triangle, hexagon], color=(0, 0, 0))
    grid = cv2.flip(grid, 0)
    return grid

# Function to check whether current node is in the obstacle space or not (including bloating)
def invade_obstacle(loc):
    xMax, yMax = [600 + 1, 250 + 1]
    xMin, yMin = [0, 0]
    x, y = loc
    gap = 5

    h1 = (-40 / 69) * x + (8764 / 69)
    h2 = 369
    h3 = (40 / 69) * x - (13812 / 69)
    h4 = (-4 / 7) * x + (8492 / 7)
    h5 = 230
    h6 = (4 / 7) * x - (732 / 7)

    t1 = (-114 / 59) * x + (9955 / 59)
    t2 = (114 / 59) * x + (6515 / 59)
    t3 = 457

    if y < h1 and x < h2 and y > h3 and y > h4 and x > h5 and y < h6:
        return False

    if y < t1 and y > t2 and x > t3:
        return False

    if (x < xMin + gap) or (y < yMin + gap) or (x >= xMax - gap) or (y >= yMax - gap):
        return False

    if ((x <= 150 + gap) and (x >= 100 - gap) and (y <= 100 + gap)) or (
            (x <= 150 + gap) and (x >= 100 - gap) and (y >= 150 - gap)):
        return False
    else:
        return True

# Function to get child of input parent
def get_child(node):
    xMax, yMax = [600 + 1, 250 + 1]
    xMin, yMin = [0, 0]

    xc, yc = node.pos
    children = []

    # check if moving down is possible
    if yc > yMin:
        (actionCost, child) = Direction('South', node)
        if invade_obstacle(child.pos):
            # if node is not generated, append in child list
            children.append((actionCost, child))
        else:
            del child

    # check if moving up is possible
    if yc < yMax:
        (actionCost, child) = Direction('North', node)
        if invade_obstacle(child.pos):
            # if node is not generated, append in child list
            children.append((actionCost, child))
        else:
            del child

    # check if moving left is possible
    if xc > xMin:
        (actionCost, child) = Direction('West', node)
        if invade_obstacle(child.pos):
            # if node is not generated, append in child list
            children.append((actionCost, child))
        else:
            del child

    # check if moving right is possible
    if xc < xMax:
        (actionCost, child) = Direction('East', node)
        if invade_obstacle(child.pos):
            # if node is not generated, append in child list
            children.append((actionCost, child))
        else:
            del child

    # check if moving up-right is possible
    if yc < yMax and xc < xMax:
        (actionCost, child) = Direction('North_East', node)
        if invade_obstacle(child.pos):
            # if node is not generated, append in child list
            children.append((actionCost, child))
        else:
            del child

    # check if moving up-left is possible
    if yc < yMax and xc > xMin:
        (actionCost, child) = Direction('North_West', node)
        if invade_obstacle(child.pos):
            # if node is not generated, append in child list
            children.append((actionCost, child))
        else:
            del child

    # check if moving down-right is possible
    if yc > yMin and xc < xMax:
        (actionCost, child) = Direction('South_East', node)
        if invade_obstacle(child.pos):
            # if node is not generated, append in child list
            children.append((actionCost, child))
        else:
            del child

    # check if moving down-left is possible
    if yc > yMin and xc > xMin:
        (actionCost, child) = Direction('South_West', node)
        if invade_obstacle(child.pos):
            # if node is not generated, append in child list
            children.append((actionCost, child))
        else:
            del child
    return children

# Function to backtrack the shortest path recognized by the algotithm
def backtrack(current_node):
    path = []
    parent = current_node
    while parent != None:
        path.append(parent.pos)
        parent = parent.parent
    return path

# Djikstra's path planning algorithm
def djikstra_algo(start_pos, goal_pos):
    save = cv2.VideoWriter('video.avi', cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), 10, (600 + 1, 250 + 1))
    grid = Obstacle_space((600, 250))
    if not invade_obstacle(start_pos):
        print("start_pos position is in Obstacle grid")
        return False
    if not invade_obstacle(goal_pos):
        print("start_pos position is in Obstacle grid")
        return False

    cv2.circle(grid, (start_pos[0], grid.shape[0] - start_pos[1] - 1), 1, (0, 0, 255), 1)
    cv2.circle(grid, (goal_pos[0], grid.shape[0] - goal_pos[1] - 1), 1, (0, 255, 0), 1)

    openList = []
    open_dict = {}
    closedList = []
    closed_dict = {}
    initial_node = createNode(start_pos, None, 0)
    openList.append((initial_node.c2c, initial_node))
    open_dict[initial_node.pos] = initial_node

    while len(openList) > 0:
        openList.sort(key=lambda x: x[0])
        current_nodeCost, current_node = openList.pop(0)
        open_dict.pop(current_node.pos)
        closedList.append(current_node)
        closed_dict[current_node.pos] = current_node
        if current_node.pos == goal_pos:
            pathTaken = backtrack(current_node)
            for i in pathTaken[::-1]:
                grid[grid.shape[0] - i[1] - 1, i[0]] = [0, 0, 255]
                save.write(grid)
            save.release()
            return current_node
        else:
            childList = get_child(current_node)
            for actionCost, child_created in childList:
                if child_created.pos in list(closed_dict.keys()):
                    del child_created
                    continue
                if child_created.pos in list(open_dict.keys()):
                    if open_dict[child_created.pos].c2c > current_node.c2c + actionCost:
                        open_dict[child_created.pos].parent = current_node
                        open_dict[child_created.pos].c2c = current_node.c2c + actionCost
                else:
                    child_created.parent = current_node
                    child_created.c2c = current_node.c2c + actionCost
                    openList.append((child_created.c2c, child_created))
                    open_dict[child_created.pos] = child_created
                    x, y = child_created.pos
                    grid[grid.shape[0] - y - 1, x] = [255, 255, 50]
                    save.write(grid)
        
startpoint = int(input('Enter the start node x,y coordinate in tuple:'))
goalpoint = int(input('Enter the goal node x,y coordinate in tuple:'))
path_plan = djikstra_algo(startpoint, goalpoint)