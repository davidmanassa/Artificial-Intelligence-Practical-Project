import math

def calculateDistance(x1, y1, x2, y2):
    dx = x2-x1
    dy = y2-y1
    distance = math.sqrt((dx**2) + (dy**2))
    return distance
				

