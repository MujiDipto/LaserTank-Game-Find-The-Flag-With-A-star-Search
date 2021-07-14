#!/usr/bin/python
import sys
import copy
from laser_tank import LaserTankMap
from queue import *
import heapq

"""
COMP3702 2020 Assignment 1
@author Mujibul Islam Dipto, 44961374

"""

MOVE_FORWARD = 'f'
TURN_LEFT = 'l'
TURN_RIGHT = 'r'
SHOOT_LASER = 's'


# state representation
class Node:
    def __init__(self, uid, x, y, parent, previous_action, g_cost, heading, Map, f_cost, h_cost):
        self.f_cost = f_cost  # total cost
        self.uid = uid  # unique identifier
        self.x = x  # x coordinate
        self.y = y  # y coordinate
        self.parent = parent  # parent node
        self.previous_action = previous_action  # previous move
        self.g_cost = g_cost  # g cost
        self.heading = heading  # current direction
        self.map = Map  # game map
        self.h_cost = h_cost  # heuristic cost
        self.f_cost = g_cost + h_cost

    def __eq__(self, obj):
        return (self.x == obj.x and self.y == obj.y) and self.map == obj.map

    def __lt__(self, other):
        return self.f_cost < other.f_cost


def write_output_file(filename, actions):
    """
    Write a list of actions to an output file. You should use this method to write your output file.
    :param filename: name of output file
    :param actions: list of actions where is action is in LaserTankMap.MOVES
    """
    f = open(filename, 'w')
    for i in range(len(actions)):
        f.write(str(actions[i]))
        if i < len(actions) - 1:
            f.write(',')
    f.write('\n')
    f.close()


def pathfinder(uid, visited):
    actions = []
    while uid != 0:
        actions = visited[uid].previous_action + actions
        uid = visited[uid].parent
    return actions


# distance from given set of points to the goal
def manhattan_distance(goal0, goal1, x, y):
    cost = abs(goal0 - x)
    cost += abs(goal1 - y)
    return cost


# perform A* search
def search(pq, goal, game_map, teleports, moves, distances, ratio, ice_tiles):
    visited = {}
    shootables = ['U', 'D', 'L', 'R', 'B', '1', '2', '3', '4']
    uid = 1
    t1 = -1
    t2 = -1
    if teleports is not None:
        t1 = teleports[0]
        t2 = teleports[1]

    while pq:
        node = heapq.heappop(pq)
        if node.x == goal[0] and node.y == goal[1]:
            visited[node.uid] = node
            return pathfinder(node.uid, visited)
        visited[node.uid] = node

        shootables_exists = False
        for i in range(0, game_map.y_size):
            for j in range(0, game_map.x_size):
                if game_map.grid_data[i][j] in shootables:
                    shootables_exists = True

        moves = [[MOVE_FORWARD], [TURN_LEFT, TURN_LEFT, MOVE_FORWARD], [TURN_LEFT, MOVE_FORWARD],
                 [TURN_RIGHT, MOVE_FORWARD],
                 [SHOOT_LASER], [TURN_LEFT, TURN_LEFT, SHOOT_LASER],
                 [TURN_LEFT, SHOOT_LASER], [TURN_RIGHT, SHOOT_LASER]]
        # no need to shoot
        moves2 = [[MOVE_FORWARD], [TURN_LEFT, TURN_LEFT, MOVE_FORWARD], [TURN_LEFT, MOVE_FORWARD],
                  [TURN_RIGHT, MOVE_FORWARD]]

        if not shootables_exists:
            moves = moves2

        for move in moves:
            game_map.player_x = node.x
            game_map.player_y = node.y
            game_map.player_heading = node.heading
            game_map.grid_data = [row[:] for row in node.map]
            out = None
            for each in move:
                out = game_map.apply_move(each)
            if out != 0:
                continue

            h1_cost = manhattan_distance(goal[0], goal[1], node.x, node.y)

            if teleports is not None:
                h2_cost = manhattan_distance(teleports[1], teleports[0], game_map.player_x, game_map.player_y)
            else:
                h2_cost = sys.maxsize

            if len(ice_tiles) > 0:
                h3_cost = manhattan_distance(ice_tiles[1], ice_tiles[0], game_map.player_x, game_map.player_y)
            else:
                h3_cost = sys.maxsize

            # if teleports is not None:
            #     h2_cost = distances[node.x, node.y]
            #

            h_cost = min(h1_cost, h2_cost, h3_cost)

            if ratio >= 0.09:
                h_cost = h_cost * 0.2

            g = len(move)

            # h_cost = 0
            new_node = Node(uid, game_map.player_x, game_map.player_y, node.uid, [row[:] for row in move], g,
                            game_map.player_heading, [row[:] for row in game_map.grid_data], g + h_cost, h_cost)
            # change unique identifier for new node

            uid += 1
            exists = False
            for key in visited.keys():
                if new_node == visited[key]:
                    exists = True
                    break
            if not exists:
                heapq.heappush(pq, new_node)


def main(arglist):
    # if a map has a lot of shootables then emphasize on the g cost than h cost
    input_file = arglist[0]
    output_file = arglist[1]

    # input_file = "testcases/t3_the_river.txt"
    # input_file = "testcases/t1_crossfire.txt"
    # output_file = "testcases/out.txt"

    # Read the input test case file
    game_map = LaserTankMap.process_input_file(input_file)

    init_node = Node(0, game_map.player_x, game_map.player_y, '', '', 0, game_map.player_heading,
                     [row[:] for row in game_map.grid_data], 0, 0)

    # find the flag
    goal_state = None
    for i in range(0, game_map.y_size):
        for j in range(0, game_map.x_size):
            if game_map.grid_data[i][j] == 'F':
                goal_state = [j, i]
                break

    # find ice tiles
    ice_tiles = []
    for i in range(0, game_map.y_size):
        for j in range(0, game_map.x_size):
            if game_map.grid_data[i][j] == 'I':
                ice_tiles = [j, i]

    # look for teleporters
    # if teleports are found, heuristics will be developed to favor reaching them
    teleporters = []
    for i in range(0, game_map.y_size):
        for j in range(0, game_map.x_size):
            if game_map.grid_data[i][j] == 'T':
                teleporters.append([j, i])

    if len(teleporters) > 1:
        d1 = (manhattan_distance(teleporters[0][0], teleporters[0][1], game_map.player_x, game_map.player_y))
        d2 = (manhattan_distance(teleporters[1][0], teleporters[1][1], game_map.player_x, game_map.player_y))

        if d1 < d2:
            t_location = teleporters[0]
        else:
            t_location = teleporters[1]
    else:
        t_location = teleporters

    if len(teleporters) == 0:
        t_location = None

    # calculate distance of each point to the nearest teleporter

    distances = {}
    if t_location is not None:
        for i in range(0, game_map.y_size):
            for j in range(0, game_map.x_size):
                d = manhattan_distance(goal_state[0], goal_state[1], j, i)
                distances[j, i] = d

    # look for shootables
    # if map does not contain any shootable, no need to use the laser tank shoot move

    shootables = ['U', 'D', 'L', 'R', 'B', '1', '2', '3', '4']
    need_to_shoot = False
    sht = 0
    for i in range(0, game_map.y_size):
        for j in range(0, game_map.x_size):
            if game_map.grid_data[i][j] in shootables:
                sht += 1
                need_to_shoot = True
    if need_to_shoot:
        moves = [[MOVE_FORWARD], [TURN_LEFT, TURN_LEFT, MOVE_FORWARD], [TURN_LEFT, MOVE_FORWARD],
                 [TURN_RIGHT, MOVE_FORWARD],
                 [SHOOT_LASER], [TURN_LEFT, TURN_LEFT, SHOOT_LASER],
                 [TURN_LEFT, SHOOT_LASER], [TURN_RIGHT, SHOOT_LASER]]
    else:
        # no need to shoot
        moves = [[MOVE_FORWARD], [TURN_LEFT, TURN_LEFT, MOVE_FORWARD], [TURN_LEFT, MOVE_FORWARD],
                 [TURN_RIGHT, MOVE_FORWARD]]

    pq = [init_node]
    heapq.heapify(pq)
    sz = game_map.x_size * game_map.y_size
    ratio = sht / sz
    actions = search(pq, goal_state, game_map, t_location, moves, distances, ratio, ice_tiles)
    # Write the solution to the output file
    write_output_file(output_file, actions)


# reducing action space by reducing no of moves
if __name__ == '__main__':
    main(sys.argv[1:])
