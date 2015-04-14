# ----------
# CS 8803 Final Project
# Team: LearningRobots
# 
#

from math import *
import random

print "CS 8803 Final Project - Team: LearningRobots"

def track_data(measurements):
	for point in measurements:
		x = point[0]
		y = point[1]
		# x, y 

	print "total measurements: ", len(measurements)

    
# load data file
points = []
f = open('hexbug-training_video-centroid_data') 
for line in f.readlines():
	y = line
	y = y.replace("[", "")
	y = y.replace("]", "")
	vals = y.split(',')	
	pt = [vals[0], vals[1]]
	points.append(pt)
f.close()

track_data(points)

