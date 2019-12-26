import numpy as np
from gtsam import *	
from gpmp2 import *

def plotPlanarMobileBase(figure, axis, robot, pose, vehsize, color, width):
	#PLOTPLANARMOBILEBASE Summary of this function goes here
	#   Detailed explanation goes here

	# vehicle corners
	corner1 = pose.transform_from(Point2(vehsize[0]/2, vehsize[1]/2))
	corner2 = pose.transform_from(Point2(-vehsize[0]/2, vehsize[1]/2))
	corner3 = pose.transform_from(Point2(-vehsize[0]/2, -vehsize[1]/2))
	corner4 = pose.transform_from(Point2(vehsize[0]/2, -vehsize[1]/2))

	# vehicle base black lines
	axis.plot([corner1.x() corner2.x() corner3.x() corner4.x() corner1.x()], \
	    [corner1.y() corner2.y() corner3.y() corner4.y() corner1.y()], 'k-')
