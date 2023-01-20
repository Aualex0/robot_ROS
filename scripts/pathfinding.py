# Script de pathfinding

from __future__ import annotations
from typing import Iterator, Tuple, TypeVar, Optional
from shapely.geometry import Polygon
import heapq

#Guide d'utilisation du module de pathfinding
#from implementation import *
#start, goal = (1, 30), (28, 0)
#came_from, cost_so_far = a_star_search(diagram, start, goal)
#draw_grid(diagram, point_to=came_from, start=start, goal=goal)
#draw_grid(diagram, path=reconstruct_path(came_from, start=start, goal=goal))
#draw_grid(diagram, path=optimize_path(reconstruct_path(came_from, start=start, goal=goal)))


#Definition des types de variables
T = TypeVar('T')
Location = TypeVar('Location')
GridLocation = Tuple[int, int]

#Zones encore disponibles
zone1_available = True
zone2_available = True
zone3_available = True
zone4_available = True

#coordonnées des autres robots
coords_opponent = []
coords_opp_annex = []
coords_annex = []

"""
coords_zone_1 = [[(0,365), (435, 365), (435, 985), (0, 985)],
                    [(515, 915), (935, 915), (935, 1335), (515, 1335)]]

coords_zone_2 = [[(0, 2635), (435, 2635), (435, 2015), (0, 2015)], 
                    [(515, 2085), (935, 2085), (935, 1665), (515, 1665)]]

coords_zone_3 = [[(2000, 2635), (1565, 2635), (1565, 2015), (2000, 2015)], 
                    [(1485, 2085), (1065, 2085), (1065, 1665), (1485, 1665)]]

coords_zone_4 = [[(2000, 365), (1565, 365), (1565, 985), (2000, 985)], 
                    [(1485, 915), (1065, 915), (1065, 1335), (1485, 1335)]]
"""

#coordonnées des zones de ressources (connues)
coords_zone_1 = [[(0,400), (499, 400), (499, 999), (0, 999)],
                    [(500, 900), (899, 900), (899, 1299), (500, 1299)]]

coords_zone_2 = [[(0, 2599), (499, 2599), (499, 2000), (0, 2000)], 
                    [(500, 2099), (899, 2099), (899, 1700), (500, 1700)]]

coords_zone_3 = [[(1999, 2599), (1500, 2599), (1500, 2000), (1999, 2000)], 
                    [(1499, 2099), (1100, 2099), (1100, 1700), (1499, 1700)]]

coords_zone_4 = [[(1999, 400), (1500, 400), (1500, 999), (1999, 999)], 
                    [(1499, 900), (1100, 900), (1100, 1299), (1499, 1299)]]

#murs dûs aux ressources
walls_zone_1 = []
walls_zone_2 = []
walls_zone_3 = []
walls_zone_4 = []

#on les ajoute à la main car on connait leurs positions
for i in range(0,5):
    for j in range(4, 10):
        walls_zone_1.append((i,j))
        walls_zone_2.append((i, 29-j))
        walls_zone_3.append((19-i, 29-j))
        walls_zone_4.append((19-i, j))
for i in range(5, 9):
    for j in range(9, 13):
        walls_zone_1.append((i,j))
        walls_zone_2.append((i, 29-j))
        walls_zone_3.append((19-i, 29-j))
        walls_zone_4.append((19-i, j))


#Definition de la class Graph
class Graph:
    def neighbors(self, id: Location) -> list[Location]: pass


# utility functions for dealing with square grids
def from_id_width(id, width):
    return (id % width, id // width)

# fonction d'affichage d'une case
def draw_tile(graph, id, style):
    r = " . "
    if 'number' in style and id in style['number']: r = " %-2d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = " > "
        if x2 == x1 - 1: r = " < "
        if y2 == y1 + 1: r = " v "
        if y2 == y1 - 1: r = " ^ "
    if 'path' in style and id in style['path']:   r = " @ "
    if 'start' in style and id == style['start']: r = " S "
    if 'goal' in style and id == style['goal']:   r = " G "
    if id in graph.walls: r = "###"
    return r

#fonction d'affichage de la grille complète
def draw_grid(graph, **style):
    print("___" * graph.width)
    for y in range(graph.height):
        for x in range(graph.width):
            print("%s" % draw_tile(graph, (x, y), style), end="")
        print()
    print("~~~" * graph.width)

#Definition de la classe grille carrée
class SquareGrid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.walls: list[GridLocation] = []
    
    def in_bounds(self, id: GridLocation) -> bool:
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id: GridLocation) -> bool:
        return id not in self.walls
    
    def neighbors(self, id: GridLocation) -> Iterator[GridLocation]:
        (x, y) = id
        neighbors = [(x+1, y), (x-1, y), (x, y-1), (x, y+1)] # E W N S
        # see "Ugly paths" section for an explanation:
        if (x + y) % 2 == 0: neighbors.reverse() # S N W E
        results = filter(self.in_bounds, neighbors)
        results = filter(self.passable, results)
        return results

#fonction de mise à jour des murs avec un nouveau polygone
def update_walls(polygone):
    walls = []
    for i in range(20):
        for j in range(30):
            p = Polygon([(i*100, j*100), ((i+1)*100 - 1, j*100), (i*100, (j+1)*100-1), ((i+1)*100 - 1, (j+1)*100 - 1)])
            if p.intersects(polygone):
                walls.append((i,j))
    return walls

#Definition de la class de file de priorité
class PriorityQueue:
    def __init__(self):
        self.elements: list[tuple[float, T]] = []
    
    def empty(self) -> bool:
        return not self.elements
    
    def put(self, item: T, priority: float):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self) -> T:
        return heapq.heappop(self.elements)[1]

#fonction de reconstruction du chemin après calcul
def reconstruct_path(came_from: dict[Location, Location],
                     start: Location, goal: Location) -> list[Location]:

    current: Location = goal
    path: list[Location] = []
    if goal not in came_from: # no path was found
        return []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    return path

#fonction heuristique
def heuristic(a: GridLocation, b: GridLocation) -> float:
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

#fonction de pathfinding
def a_star_search(graph: Graph, start: Location, goal: Location):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from: dict[Location, Optional[Location]] = {}
    cost_so_far: dict[Location, float] = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current: Location = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + 1
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(next, goal)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

#fonction de recherche de la non-nécessité d'un pas du chemin
def merge_available(Loc1, Loc2, Loc3):
    Loc1 = (Loc1[0]*100 + 5, Loc1[1]*100 + 5)
    Loc2 = (Loc2[0]*100 + 5, Loc2[1]*100 + 5)
    Loc3 = (Loc3[0]*100 + 5, Loc3[1]*100 + 5)
    p = Polygon([Loc1, Loc2, Loc3])
    print(Loc1, Loc2, Loc3)
    if zone1_available and (Polygon(coords_zone_1[0]).intersects(p) or Polygon(coords_zone_1[1]).intersects(p)):
        return False
    elif zone2_available and (Polygon(coords_zone_2[0]).intersects(p) or Polygon(coords_zone_2[1]).intersects(p)):
        return False
    elif zone3_available and (Polygon(coords_zone_3[0]).intersects(p) or Polygon(coords_zone_3[1]).intersects(p)):
        return False
    elif zone4_available and (Polygon(coords_zone_4[0]).intersects(p) or Polygon(coords_zone_4[1]).intersects(p)):
        return False
    else:
        return (not Polygon(coords_opponent).intersects(p)) and (not Polygon(coords_opp_annex).intersects(p)) and (not Polygon(coords_annex).intersects(p))

#fonction d'optimisation du chemin calculé
def optimize_path(path):
    #regarder si il existe une intersection entre le polygone créé et les murs
    #si non, merge le path
    print(path)
    new_path: list[Location] = [path[0]]
    ind = 1
    l = len(path)
    if l == 1:
        return new_path
    while ind < l-1:
        if not merge_available(new_path[len(new_path) - 1],path[ind], path[ind+1]):
            new_path.append(path[ind])
        ind += 1
    new_path.append(path[l-1])
    return new_path

diagram = SquareGrid(20, 30)
diagram.walls += walls_zone_1 + walls_zone_2 + walls_zone_3 + walls_zone_4
#diagram.walls += update_walls(Polygon(coords_zone_1[0])) + update_walls(Polygon(coords_zone_1[1]))
#diagram.walls += update_walls(Polygon(coords_zone_2[0])) + update_walls(Polygon(coords_zone_2[1]))
#diagram.walls += update_walls(Polygon(coords_zone_3[0])) + update_walls(Polygon(coords_zone_3[1]))
#diagram.walls += update_walls(Polygon(coords_zone_4[0])) + update_walls(Polygon(coords_zone_4[1]))
