import math
import turtle

def change_coordinates(new_origin, point):
    # projection dans le repère du robot
    projected = [math.cos(new_origin[2])*point[0] + math.sin(new_origin[2])*point[1], - math.sin(new_origin[2])*point[0] + math.cos(new_origin[2])*point[1]]
    rot_origin = [math.cos(new_origin[2])*point[0] + math.sin(new_origin[2])*point[1], - math.sin(new_origin[2])*point[0] + math.cos(new_origin[2])*point[1]]
    projected[0] -= rot_origin[0]
    projected[1] -= rot_origin[1]
    return projected

turtle.setup(600, 400)
wn = turtle.Screen()

show = turtle.Turtle()
show.color("blue")
show.penup() 

objective = [150, 26]
position = [126, 473, 78]
show.goto(objective[0], objective[1])
show.dot()

show.left(position[2])
show.forward(change_coordinates(position, objective)[0])
show.left(90)
show.forward(change_coordinates(position, objective)[1])
show.dot()
show.hideturtle()

input("Press Enter to continue...")
