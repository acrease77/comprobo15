#!/usr/bin/env python

class Triangle:
	"""represents a triangle"""
	def __init__(self,x1,y1, x2,y2, x3,y3):
		"""the three verticies of the triangle"""
		self.v1 = Point(x1,y1)
		self.v2 = Point(x2,y2)
		self.v3 = Point(x3,y3)

class Point:
	"""A class to represent a point in 2D space"""
	def __init__(self,x=0.0,y=0.0): #put code for creating all attributes associated with Point object here
		"""initialize a point with specified X and Y positions"""
		self.x = x   #need to define these and relate to init method
		self.y = y	

	def move(self,dx,dy):	#self needs to be passed through every function
		"""modifies the point by translating it by dx and dy in the x and Y direcitons"""
		self.x = self.x + dx
		self.y = self.y + dy	#modifies the point itself, so don't need to return anything

	def scale(self, scale_factor):
		"""Scales the point by the specified scale factor"""
		self.x = self.x * scale_factor
		self.y = self.y * scale_factor





p = Point(2.0,1.0)	#init method is called just by typing out the name of the class
print p.x
print p.y#can access attributes of the class with dot notation
p.move(3.0,-1.0)
print '(',p.x, p.y,')'