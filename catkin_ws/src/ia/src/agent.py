#!/usr/bin/env python
# encoding: utf8
# Artificial Intelligence, UBI 2019-20
# Modified by: David Pires nº37272 and Pedro Maria nº39866

import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import math
import uuid
import time
from Graph import Graph
import CoordHelper

startTime = time.time()

x_ant = 0
y_ant = 0
obj_ant = ''
lastVisitedRoom = -1
actualRoom = -1

# (numero da s,x1,y1,x2,y2)
room_list = [(1,-15.6,-3.0,-3.6,-0.8), (2,-12.0,-1.4,-8.9,4.8), (3,-10.6,4.8,3.6,7.9), (4,-4.6,-0.8,-0.8,4.8), (5,-15.6,-1.4,-12.5,3.0), (6,-15.6,3.0,-12.5,7.9), (7,-15.6,7.9,-10.6,11.1), (8,-10.6,7.9,-5.6,11.1), (9,-5.6,7.9,-0.5,11.1), (10,-0.5,7.9,3.6,11.1), (11,-0.8,1.4,3.6,4.8), (12,-0.8,-0.8,3.6,1.4), (13,-8.9,-0.8,-6.5,4.8),(14,-6.5,-0.8,-4.6,4.8)]

## (room_id, object_name, object_id)
object_list = []

## UUID --distance-> UUID
## UUID is a Point UUID
graph = Graph()

## (uuid, coordX, coordY, roomA, roomB)
point_list = []
## point is a door
def new_point(coordX, coordY, roomA, roomB):
	uid = uuid.uuid4()
	point_list.append((uid, coordX, coordY, roomA, roomB))
	for point in point_list:
		if point[3] == roomA or point[3] == roomB or point[4] == roomA or point[4] == roomB:
			distance = CoordHelper.calculateDistance(point[1], point[2], coordX, coordY)
			graph.add_edge(uid, point[0], distance)

def calculateDistanceFromCoordToPoint(CoordX, CoordY, PointA):
	auxX = 0
	auxY = 0
	for point in point_list:
		if point[0] == PointA:
			auxX = point[1]
			auxY = point[2]
	return CoordHelper.calculateDistance(CoordX, CoordY, auxX, auxY)

def calculateDistance(PlayerCoordX, PlayerCoordY, pointPath):
	distance = calculateDistanceFromCoordToPoint(PlayerCoordX, PlayerCoordY, pointPath[0])
	last_point = pointPath[0] ## uuid
	first = True
	for point in pointPath:
		if first:
			first = False
			continue
		distance += graph.getWeight(last_point, point)
		last_point = point
	return distance

def getCoordPath(pointPath):
	path = []
	for point in pointPath:
		for point1 in point_list:
			if (point1[0] == point):
				path.append((point1[1], point1[2]))
	return path

def getRoomPath(pointPath, startRoom):
	result = [startRoom]
	for point in pointPath:
		for point1 in point_list:
			if point1[0] == point:
				if point1[3] in result:
					result.append(point1[4])
				if point1[4] in result:
					result.append(point1[3])
	return result

def dijsktraRooms(roomA, roomB):
	pointA = 0
	pointB = 0
	for point in point_list:
		if (point[3] == roomA or point[4] == roomA):
			pointA = point[0]
		if (point[3] == roomB or point[4] == roomB):
			pointB = point[0]
	if (pointA == 0 or pointB == 0):
		return "No route"
	return (dijsktra(graph, pointA, pointB))

def dijsktra(graph, initial, end):
    # shortest paths is a dict of nodes
    # whose value is a tuple of (previous node, weight)
    shortest_paths = {initial: (None, 0)}
    current_node = initial
    visited = set()
    
    while current_node != end:
        visited.add(current_node)
        destinations = graph.edges[current_node]
        weight_to_current_node = shortest_paths[current_node][1]

        for next_node in destinations:
            weight = graph.weights[(current_node, next_node)] + weight_to_current_node
            if next_node not in shortest_paths:
                shortest_paths[next_node] = (current_node, weight)
            else:
                current_shortest_weight = shortest_paths[next_node][1]
                if current_shortest_weight > weight:
                    shortest_paths[next_node] = (current_node, weight)
        
        next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
        if not next_destinations:
            return "Route Not Possible"
        # next node is the destination with the lowest weight
        current_node = min(next_destinations, key=lambda k: next_destinations[k][1])
    
    # Work back through destinations in shortest path
    path = []
    while current_node is not None:
        path.append(current_node)
        next_node = shortest_paths[current_node][0]
        current_node = next_node
    # Reverse path
    path = path[::-1]
    return path

def match_room(x, y):
	for room in room_list:
		if x >= room[1] and x <= room[3] and y >= room[2] and y <= room[4]:
			return room[0]
	return -1
	
def auxIsSuite(roomNumber):
	if door_number <= 4:
		return False
	for door in door_list:
		if (roomNumber == door[2] or roomNumber == door[3]) and (door[3] > 4 and door[2] > 4):
			return True
	return False

def getRoomType(roomNumber):
	obj_list = []
	for i in object_list:
		if (i[0] == roomNumber):
			obj_list.append(i)
	counterBed = 0
	counterChair = 0
	counterTable = 0
	for i in obj_list:
		if i[1] == "bed":
			counterBed += 1
		elif i[1] == "chair":
			counterChair += 1
		elif i[1] == "table":
			counterTable += 1
	if counterBed == 1:
		return "Single room"
	elif counterBed == 2:
		return "Double room"
	elif counterBed > 1 and auxIsSuite(roomNumber) == True:
		return "Suite room"
	elif counterTable == 1 and counterChair > 1:
		return "Meeting room"
	else:
		return "Generic room"


# ---------------------------------------------------------------

# 1 - Quantas salas _não_ estão ocupadas?
def question1():
	counterOccuped = 0
	counterNotKnown = 0
	for roomNumber in range(5, len(room_list) + 1):
		counterObj = 0
		for obj in object_list:
			if obj[0] == roomNumber:
				counterObj += 1
				if obj[1] == "person":
					counterOccuped += 1
		if counterObj == 0:
			counterNotKnown += 1
	print( " There are %d rooms not occuped by people in %d known rooms. " % (((10 - counterNotKnown) - counterOccuped), (10 - counterNotKnown)) ) 

# 2 - Quantas suites encontraste até agora?
def question2():
	counter = 0
	for roomNumber in range(1, len(room_list) + 1):
		if (getRoomType(roomNumber) == "Suite room"):
			counter += 1
	print( " I've found %d Suite rooms so far. " % counter )

# 3 - É mais provavel encontrar pessoas nos corredores ou nos quartos?
def question3():
	counterHall = 0
	counterRooms = 0
	for obj in object_list:
		if obj[1] == "person":
			if obj[0] <= 4:
				counterHall += 1
			else:
				counterRooms += 1
	if counterHall > counterRooms:
		print( " Is more likely to meet people in the halls. " )
	elif counterHall < counterRooms:
		print( " Is more likely to meet people in the rooms. " )
	elif counterHall == 0 and counterRooms == 0:
		print ( "I don't know any person yet. " )
	else:
		print( " The probability of find people in rooms or in the halls is equal. " )

# 4 - Se queres encontrar um PC, para que sala vais?
def question4():
	roomNumber = -1
	for obj in object_list:
		if obj[1] == "computer":
			if getRoomType(obj[0]) == "Meeting room" or getRoomType(obj[0]) == "Generic room": # Only for privacy :)  
				roomNumber = obj[0]
				break
			roomNumber = obj[0]
	if roomNumber == -1:
		print( " I don't know any room with a computer. " )
	else:
		print( " Go to room number %d to find a Computer. " % roomNumber )


# 5 - Qual é o numero da sala (Single room) mais próxima?
def closestSingleRoom(atualX, atualY):
	min_room = -1
	min_distance = 9999999
	for room in room_list:
		if (getRoomType(room[0]) == "Single room"):
			tempDistance = calculateDistance(atualX, atualY, dijsktraRooms(match_room(atualX, atualY), room[0]))
			if (tempDistance < min_distance):
				min_distance = tempDistance
				min_room = room[0]
	return min_room

def question5():
	csr = closestSingleRoom(x_ant, y_ant)
	if csr != -1:
		print("The closest Single room is %d." % csr)
	else:
		print("I don't know any Single room yet.")

# 6 - Como podes ir da sala onde estás até ao elevador?
def question6():
	roomPath = getRoomPath(dijsktraRooms(match_room(x_ant, y_ant), -1), match_room(x_ant, y_ant))
	roomPath = roomPath[1:-1]
	result = " Visit the follow rooms to go to the Elevator: "
	for room in roomPath:
		result += str(room) + "  "
	print(result)

# 7 - Quantos livros achas que vais encontrar nos próximos dois minutos?
def question7():
	actualTime = time.time()
	counterBooks = 0
	for obj in object_list:
		if obj[1] == "book":
			counterBooks += 1
	result = (120 * counterBooks) / (actualTime - startTime)
	print( " I think I will find %d books in the next 2 minutes. " % int(result))

# 8 - Qual a probabilidade de encontrar uma mesa em uma sala que não tenha livros mas tenha pelo menos uma cadeira?
def question8():
	counterRoomWithChairAndNotBook = 0
	counterRoomWithTableAndChairAndNotBook = 0
	for room in range(5, 14):
		counterBook = 0
		counterChair = 0
		counterTable = 0
		for obj in object_list:
			if obj[0] == room:
				if obj[1] == "chair":
					counterChair += 1
				if obj[1] == "book":
					counterBook += 1
				if obj[1] == "table":
					counterTable += 1
		if counterChair > 0 and counterBook == 0:
			counterRoomWithChairAndNotBook += 1
		if counterChair > 0 and counterTable > 0 and counterBook == 0:
			counterRoomWithTableAndChairAndNotBook += 1
	if counterRoomWithChairAndNotBook == 0:
		print( " I don't know any room without books but that has at least one chair yet. " )
	else:
		result = counterRoomWithTableAndChairAndNotBook / counterRoomWithChairAndNotBook
		print( " The probability of finding a table in a room without books but that has at least one chair is %d. " % result )

# ---------------------------------------------------------------
# odometry callback
def callback(data):
	global x_ant, y_ant, actualRoom, lastVisitedRoom
	x=data.pose.pose.position.x-15
	y=data.pose.pose.position.y-1.5
	tempRoom = match_room(x, y)
	if tempRoom != actualRoom and tempRoom != -1:
		lastVisitedRoom = actualRoom
		actualRoom = tempRoom
		new_point(x_ant, y_ant, lastVisitedRoom, actualRoom)
	
	# show coordinates only when they change
	if x != x_ant or y != y_ant:
		print(" x=%.1f y=%.1f room=%d " % (x, y, actualRoom))
	x_ant = x
	y_ant = y

# ---------------------------------------------------------------
# object_recognition callback

def existentObject(obj_id):
	for i in range(len(object_list)):
		if (obj_id == object_list[i][2]):
			return True
	return False

def registerObject(data):
	global actualRoom
	obj_lst = data.split(',')
	for i in range(len(obj_lst)):
		objd = obj_lst[i]
		obj_name = objd.split('_')[0]
		obj_id = objd.split('_')[1]
		if existentObject(obj_id) == False:
			object_list.append((actualRoom, obj_name, obj_id))

def callback1(data):
	global obj_ant
	obj = data.data
	if obj != obj_ant and data.data != "":
		print("object is %s" % data.data)
		registerObject(data.data)
	obj_ant = obj
		
# ---------------------------------------------------------------
# questions_keyboard callback
def callback2(data):
	print("question is %s" % data.data)
	if data.data == "1":
		question1()
	elif data.data == "2":
		question2()
	elif data.data == "3":
		question3()
	elif data.data == "4":
		question4()
	elif data.data == "5":
		question5()
	elif data.data == "6":
		question6()
	elif data.data == "7":
		question7()
	elif data.data == "8":
		question8()

# ---------------------------------------------------------------
def agent():
	rospy.init_node('agent')

	rospy.Subscriber("questions_keyboard", String, callback2)
	rospy.Subscriber("object_recognition", String, callback1)
	rospy.Subscriber("odom", Odometry, callback)

	rospy.spin()

# ---------------------------------------------------------------
if __name__ == '__main__':
	agent()

# ---------------------------------------------------------------

