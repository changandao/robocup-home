#!/usr/bin/env python
import rospy

## Storage answer class
class ArenaStorage:
    ## Create a Arena_Storage class object.
    #
	def __init__(self):
		## Object stored
		self.object = ''

## Storage answer class
class ArenaStorage:
    ## Create a Arena_Storage class object.
    #
	def __init__(self):
		## Location of counted object
		self.location = ''

## Count objects answer class
class ArenaNumber:
    ## Create a Arena_Number class object.
    #
	def __init__(self):
		## Object which should be counted
		self.object = ''
		## Property of counted object
		self.property = 'None'
		## Location of counted object
		self.location = ''

## Check objects types answer class
class ArenaObjects:
    ## Create a Arena_Objects class object.
    #
	def __init__(self):
		## Location of objects
		self.location = ''

## Classify objects answer class
class ObjectClassify:
    ## Create a Object_Classify class object.
    #
	def __init__(self):
		## To be classified object
		self.location = ''

## Compare objects answer class
class ObjectComp:
    ## Create a Object_Classify class object.
    #
	def __init__(self):
		## List of classified object
		self.objects = []
		## Compared property
		self.property = None

## Compare classes of objects answer class
class ObjectCompCat:
    ## Create a Object_Classify class object.
    #
	def __init__(self):
		## List of classified object
		self.objects = []

## Object trivia answer class
class ObjectTrivia:
    ## Create a object_trivia class object.
    #
	def __init__(self):
		## Property of object
		self.property = ''

## Object color answer class
class ObjectColor:
    ## Create a object_color class object.
    #
	def __init__(self):
		## Object which is the subject of question
		self.object = ''
		## Property of object
		self.property = ''

## Number of people answer class
class CrowdNumber:
    ## Create a crowd_number class object.
    #
	def __init__(self):
		## Type of person to be counted
		self.object = ''
		## Location of the people to count
		self.location = ''
		## Property of object
		self.properties = 'None'

## Age of people answer class
class CrowdAge:
    ## Create a crowd_age class object.
    #
	def __init__(self):
		## Person to assume age
		self.person = ''

## Crowd gender answer class
class CrowdGender:
    ## Create a crowd_gender class object.
    #
	def __init__(self):
		## Person to assume gender
		self.object = ''
		## Property of object
		self.properties = 'None'
