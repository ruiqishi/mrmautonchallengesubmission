"""
Ruiqi (Peter) Shi

More efficient version of the other agent.py files, moves towards target point until it is blocked/we are close and then replans
replans every 25 ticks if more efficient path is discovered
requires more tuning though and i dont rlly have time for that, fails case 12 now, passes 11 sometimes, fails 04 sometimes i dont really know
some single line changes pass different combinations of 4 11 and 12
All 3 submitted solutions perform about the same so im just putting them all out there

scan surroundings every tick and update map, regenerate path with A*
secondary map with all borders extended is used for A* path generation to build the robot width margin into the path
search reverse through through path to find shortcuts (latest unblocked path point)
needs better path following even though i should be good at that
also needs better path generating in terms of object avoidance margin although this might just be a path following thing
all the path following control stuff i know is mainly for non-holonomic drive bases i might be overthinking it a lot
sensor noise seems not to affect everything too much but filtering that would probably help

"""

import math
import copy
import heapq

class Cell:
  def __init__(self):
    self.parent_i = 0
    self.parent_j = 0
    self.f = float('inf')
    self.g = float('inf')
    self.h = 0.0

class Agent:

  def __init__(self, cfg:dict):
    """cfg keys: width_m, height_m, resolution, robot_radius, v_max, a_max, dt, sense_cells, goal_tol, goal (x, y)."""
    self.cfg = cfg

    # map of field with walls
    self.map = [['.' for _ in range(82)] for _ in range(122)]

    for i in range(122):
      self.map[i][0] = "#"
      self.map[i][81] = "#"
    for i in range(82):
      self.map[0][i] = "#"
      self.map[121][i] = "#"

    # map of field with expanded walls for pathfinding
    self.tmpmap = [["." for _ in range(82)] for _ in range(122)]

    # initializing velocity commands, path to follow, current target point
    self.vx = 0.0
    self.vy = 0.0
    self.targets = []
    self.t = (0, 0)

    # init generates the path so we dont skip it if 0,0 is unblocked
    self.init = False

    # tolerance to target point for replanning to trigger
    self.tolerance = 8

    # counter for replanning timer
    self.counter = 0

  # distance between points helper function
  def dist(self, x1:float, y1:float, x2:float, y2:float):
    return math.sqrt((x1 - x2)**2+(y1 - y2)**2)

  # checking if 2 points are blocked along a line of some width
  def blocked(self, x1:float, y1:float, x2:float, y2:float, buffer:float):
    if (x1 == x2 and y1 == y2):
      return False
    minX = math.floor(min(x1, x2))
    maxX = math.ceil(max(x1, x2))
    minY = math.floor(min(y1, y2))
    maxY = math.ceil(max(y1, y2))
    # loops through all points in the box bounded by the 2 points and checks each box within a buffer of the line defined by the 2 points
    for x in range(max(int(minX-buffer/1.5), 0), min(int(maxX+1+buffer/1.5), 120)):
      for y in range(max(int(minY-buffer/1.5), 0), min(int(maxY+1+buffer/1.5), 80)):
        # line equation: (y1-y2)x + (x2-x1)y + (x1y2 - x2y1)
        # point: x, y
        # line point distance: abs((y1-y2)x + (x2-x1)y + (x1y2 - x2y1))/sqrt((y1-y2)**2 + (x2-x1)**2)
        if (((abs((y1-y2)*x + (x2-x1)*y + (x1*y2 - x2*y1))/math.sqrt((y1-y2)**2 + (x2-x1)**2)) <= (buffer/2)  #) and (self.map[x+1][y+1] == '#')):
             *2-abs(4*abs(math.atan2((y2-y1),(x2-x1))/math.pi)-1)) and (self.map[x+1][y+1] == '#')):
          return True
    return False

  # pathfinding algorithm
  def aStar(self, x1, y1, x2, y2):
    # list of visited elements and node list
    closed_list = [[False for _ in range(80)] for _ in range(120)]
    cell_details = [[Cell() for _ in range(80)] for _ in range(120)]

    # initialization of current checking coordinates
    i, j = x1, y1

    # initializing starting node
    cell_details[i][j].f = 0
    cell_details[i][j].g = 0
    cell_details[i][j].h = 0

    # -10 is unreachable value, we will know we have reached starting node when this value is found
    cell_details[i][j].parent_i = -10
    cell_details[i][j].parent_j = -10

    # initializing open list
    open_list = []

    # open list begins with starting node
    heapq.heappush(open_list, (0.0, i, j))

    # movement directions for the path
    directions = [
      (0, 1), (0, -1), (1, 0), (-1, 0)
      , (1, 1), (1, -1), (-1, 1), (-1, -1)
    ]

    # loop through open set
    while open_list:
      # pop lowest h value open set element
      _, i, j = heapq.heappop(open_list)

      # set path when ending node is reached
      if i == x2 and j == y2:
        # empty the path
        self.targets = []
        xx = x2
        yy = y2

        # search back up the nodes until starting node is reached and add to the path
        while not (cell_details[xx][yy].parent_i == -10 and cell_details[xx][yy].parent_j == -10):
          self.targets.append((xx, yy))
          xxx = cell_details[xx][yy].parent_i
          yyy = cell_details[xx][yy].parent_j
          xx = xxx
          yy = yyy
        self.targets.append((xx, yy))

        # reverse to start at right end
        self.targets.reverse()

        # path found
        return True

      # skip if node visited
      if closed_list[i][j]:
        continue

      # add node to visited list
      closed_list[i][j] = True

      for di, dj in directions:
        # check in a direction of the current node
        new_i = i + di
        new_j = j + dj

        # checking if node is valid
        # bounds
        if (new_i < 0 or new_i > 119 or new_j < 0 or new_j > 79):
          continue

        # starting node
        if (new_i == x1 and new_j == y1):
          continue

        # wall
        if self.tmpmap[new_i+1][new_j+1] == "#":
          continue

        # visited
        if closed_list[new_i][new_j]:
          continue

        # calculate h cost for this node
        g_new = cell_details[i][j].g + math.sqrt(di**2 + dj**2)
        h_new = self.dist(new_i, new_j, x2, y2)
        f_new = g_new + h_new

        # extra safeguard for starting node thats here because of troubleshooting and im scared to remove it
        if not(cell_details[new_i][new_j].parent_i == x1 and cell_details[new_i][new_j].parent_j == y1):
          # add node to open set if cost is lower than previous node cost value from diffrent path
          if cell_details[new_i][new_j].f > f_new:
            cell_details[new_i][new_j].f = f_new
            cell_details[new_i][new_j].g = g_new
            cell_details[new_i][new_j].h = h_new

            cell_details[new_i][new_j].parent_i = i
            cell_details[new_i][new_j].parent_j = j

            heapq.heappush(
              open_list,
              (f_new, new_i, new_j)
            )
    # path not found
    return False

  def step(self, pose:tuple[float, float], scan:tuple[int, int, list[str]]) -> tuple[float, float]:
    """
    called once per tick.

    pose: (x, y) metres from SLAM, ~2 cm gaussian noise.
    scan: (cx0, cy0, rows) -- a (2*sense_cells+1)^2 window of '#'/'.' around the robot. rows[j][i] is cell (cx0+i, cy0+j).
          everything in the window is observed, nothing outside it is.
          the window origin comes from the noisy pose, so walls can land one cell off between scans.
    returns: (vx, vy) world-frame velocity command in m/s. sim clamps speed and acceleration.
    """

    # robot position
    rx = pose[0]*10
    ry = pose[1]*10

    # defining goal because i dont know how to make it global i dont use python very much
    goal = (self.cfg["goal"][0]*10, self.cfg["goal"][1]*10)

    # scan stuff
    for sx in range(len(scan[2][0])):
      for sy in range(len(scan[2])):
        if (scan[0]+sx >= 1 and scan[0]+sx < 121 and scan[1]+sy >= 1 and scan[1]+sy < 81):
          self.map[scan[0]+sx+1][scan[1]+sy+1] = scan[2][sy][sx]

    # making a copy of the real map with all walls expanded as a tolerance for robot width
    self.tmpmap = [["." for _ in range(82)] for _ in range(122)]
    for y in range(0, 82):
      for x in range(0, 122):
        if self.map[x][y] == "#":
          for xx in range(min(max(x-6, 0), 120), min(max(x+6, 0), 120)):
            for yy in range(min(max(y-4, 0), 80), min(max(y+4, 0), 80)):
              self.tmpmap[xx][yy] = "#"

    # set robot pos and surrounding squares to empty because they have to be
    roundx = math.floor(rx+0.5)
    roundy = math.floor(ry+0.5)
    for x in range(min(max(roundx-1, 1), 120), min(max(roundx+1, 1), 120)):
      for y in range(min(max(roundy-1, 1), 80), min(max(roundy+1, 1), 80)):
        self.tmpmap[x][y] = "."

    # same for goal
    roundx = math.floor(goal[0]+0.5)
    roundy = math.floor(goal[1]+0.5)
    for x in range(min(max(roundx-1, 1), 120), min(max(roundx+3, 1), 120)):
      for y in range(min(max(roundy-1, 1), 80), min(max(roundy+3, 1), 80)):
        self.tmpmap[x][y] = "."

    tBlocked = True
    if self.init:
      tBlocked = self.blocked(rx, ry, self.t[0], self.t[1], 6) or self.dist(rx, ry, self.t[0], self.t[1]) < self.tolerance or self.counter % 25 == 0
    else:
      # set default target to current pos
      self.t = (rx, ry)
    
    # A* pathfinding algo
    nextTarget = self.t

    if tBlocked or not self.init:
      self.init = True
      if self.aStar(math.floor(rx), math.floor(ry), math.floor(goal[0]+0.5), math.floor(goal[1]+0.5)):
        # search through generated path starting at the end node to find shortcuts
        found = False
        for i in range(len(self.targets)):
          target = self.targets[len(self.targets)-1-i]
          if not self.blocked(rx, ry, target[0], target[1], 6):
            # set current target to shortcut if unblocked
            self.t = copy.copy(target)
            nextTarget = self.targets[min(len(self.targets)-i, len(self.targets)-1)]
            found = True
            break
        # set current target to a few nodes ahead if no shortcut
        # this shouldnt happen if we are close to goal the path should be unblocked within 5 grid spaces
        if (not found):
          self.t = self.targets[5]
          nextTarget = self.targets[6]

    # round target for some reason idk i wanted the pos to be integers and robot pos is float
    tx = math.floor(self.t[0]+0.5)
    ty = math.floor(self.t[1]+0.5)
    ntx = math.floor(nextTarget[0]+0.5)
    nty = math.floor(nextTarget[1]+0.5)

    # debug
    # print(tx, ty)
    # for target in self.targets:
    #   print(target)
    # print("-----------")
    # for a in range(82):
    #   for x in range(122):
    #     y = 81 - a
    #     if (abs(x + 1 - rx) < 3 and abs(y + 1- ry) < 3):
    #       print("r", end=" ")
    #     elif (abs(x + 1 - goal[0]) < 3 and abs(y + 1 - goal[1]) < 3):
    #       print("g", end=" ")
    #     elif (abs(x + 1 - tx) < 3 and abs(y + 1 - ty) < 3):
    #       print("x", end=" ")
    #     elif ((x + 1, y + 1) in self.targets):
    #       print("p", end=" ")
    #     else:
    #       print(self.tmpmap[x][y], end=" ")
    #   print()

    # velocity angle control
    vTheta = math.atan2(ty + 1 - math.floor(ry+0.5), tx - math.floor(rx+0.5))

    # try max v
    v = 2

    # damp when approaching wall
    if self.map[min(max(math.floor(rx+8*math.cos(vTheta)+0.5), 1), 120)][min(max(math.floor(ry+8*math.sin(vTheta)+0.5), 1), 80)] == "#":
      v -= 1      
    # if self.map[min(max(math.floor(rx+6*math.cos(vTheta)+0.5), 1), 120)][min(max(math.floor(ry+6*math.sin(vTheta)+0.5), 1), 80)] == "#":
    #   v -= 1.3
    # damp velocity at high angle change/turns
    else:
      # dThetaDamp = math.sin(abs(math.atan2(self.vy, self.vx) - vTheta))
      dThetaDamp = math.sin(abs(vTheta - math.atan2(nty + 1 - math.floor(ry+0.5), ntx - math.floor(rx+0.5))))*1.5
      v -= dThetaDamp

    
    # damps oscillations its like a shitty kp term but its lowk going the other way
    vTheta -= (vTheta - math.atan2(self.vy, self.vx))*0.2

    # final velocity calculation
    self.vx = v*math.cos(vTheta)
    self.vy = v*math.sin(vTheta)

    self.counter += 1

    return (self.vx, self.vy)

  def debug(self) -> dict:
    """
    optional, for `harness.py --viz` only.
    keys: blocked (cells), free (cells), path ([(x, y), ...]).
    """

    return {"blocked": self.targets}