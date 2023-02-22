# Script de pathfinding

from __future__ import annotations
from typing import Iterator, Tuple, TypeVar, Optional
from shapely.geometry import Polygon
import heapq

#Guide d'utilisation du module de pathfinding
#from pathfinding import *
#start, goal = (1, 28), (18, 0)
#came_from, cost_so_far = a_star_search(diagram, start, goal)
#draw_grid(diagram, point_to=came_from, start=start, goal=goal)
#draw_grid(diagram, path=reconstruct_path(came_from, start=start, goal=goal))
#draw_grid(diagram, path=optimize_path(reconstruct_path(came_from, start=start, goal=goal)))

#Definition des types de variables
T = TypeVar('T')
Location = TypeVar('Location')
GridLocation = Tuple[int, int]

#Initialisation des variables hard-coded
dist_robot = 180 #taille en mm du robot max depuis le centre de rotation
dist_robot_adv = 220 #taille robot adverse (marge au cas où)
taille_palet = 60 #rayon du palet
polygon_walls = []
coords_ressources = []
coords_annex = (0,0)
coords_opp_annex = (0,0)
coords_opponent = (0,0)

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
    for i in range(2000):
        for j in range(3000):
            p = Polygon([(i, j), (i+1, j), (i, j+1), (i+1, j+1)])
            if p.intersects(polygone):
                walls.append((i,j))
    return walls

#Definition de la classe de file de priorité
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
    p = Polygon([Loc1, Loc2, Loc3])
    for pol in polygon_walls:
        if p.intersects(pol):
            return False
    return True

#fonction d'optimisation du chemin calculé
def optimize_path(path):
    #regarder si il existe une intersection entre le polygone créé et les murs
    #si non, merge le path
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

#fonction annexe de création de polygones dilatés
def dilatation_polygone(x, y, border):
    #x, y = positions du centre
    return Polygon([(center + border, center + border),
                    (center - border, center + border),
                    (center - border, center - border),
                    (center + border, center - border)])

#fonction de génération des murs à partir des coordonnées des objets
def generate_walls(team):
    polygon_walls = []

    #murs dûs aux ressources
    for (x,y) in coords_ressources:
        polygon_walls.append(dilatation_polygone(x, y, taille_palet + dist_robot))

    #murs dûs aux robots
    (x,y) = coords_opponent
    polygon_walls.append(dilatation_polygone(x, y, dist_robot + dist_robot_adv))
    (x,y) = coords_opp_annex
    polygon_walls.append(dilatation_polygone(x, y, dist_robot + dist_robot_adv))
    (x,y) = coords_annex
    polygon_walls.append(dilatation_polygone(x, y, dist_robot + dist_robot_adv))


    #murs dûs aux bords
    polygon_walls.append(Polygon([(0,0), (dist_robot, 0), (0,2999), (dist_robot, 2999)]))
    polygon_walls.append(Polygon([(1999,0), (1999-dist_robot, 0), (1999,2999), (1999-dist_robot, 2999)]))
    polygon_walls.append(Polygon([(0,0), (0, dist_robot), (1999,0), (1999, dist_robot)]))
    polygon_walls.append(Polygon([(0, 2999), (0, 2999-dist_robot), (1999,2999), (1999, 2999-dist_robot)]))

    if (team == 0):
        polygon_walls.append(dilatation_polygone(225, 225, 225 + dist_robot))
    else:
        polygon_walls.append(dilatation_polygone(1999-225, 225, 225 + dist_robot)) 

    #update walls
    for wall in polygon_walls:
        diagram.walls += update_walls(wall)

def main(x, y, Table_description, team)
    diagram = SquareGrid(2000, 3000)

    #récupération des coordonnées des objets

    #coordonnées des autres robots
    coords_opponent = (Table_description.opp_main.x, Table_description.opp_main.y)
    coords_opp_annex = (Table_description.opp_annex.x, Table_description.opp_annex.y)
    coords_annex = (Table_description.annex.x, Table_description.annex.y)

    #coordonnées des ressources restantes
    coords_ressources = []
    for obj in Table_description.other.q1:
        coords_ressources.append((obj.x, obj.y))
    for obj in Table_description.other.q2:
        coords_ressources.append((obj.x, obj.y))
    for obj in Table_description.other.q3:
        coords_ressources.append((obj.x, obj.y))
    for obj in Table_description.other.q4:
        coords_ressources.append((obj.x, obj.y))
    for obj in Table_description.other.gateaux:
        coords_ressources.append((obj.x, obj.y))

    #génération des murs à partir des coordonnées des objets
    generate_walls()

    #calcul de la trajectoire
    start, goal = (Table_description.itself.x, Table_description.itself.y), (x, y)
    came_from, cost_so_far = a_star_search(diagram, start, goal)

    return optimize_path(reconstruct_path(came_from, start=start, goal=goal))
