
import numpy as np
from gtsam import *	
from gpmp2 import *

def plotPlanarMobile2Arms(figure, axis, marm, p, vehsize, color, width):
	#PLOTPLANARMOBILE2ARMS Summary of this function goes here
	#   Detailed explanation goes here
	# color  = [(r,g,b)] where all values lie between 0 and 1
	pose = p.pose()
	# vehicle corners
	corner1 = pose.transform_from(Point2(vehsize[0]/2, vehsize[1]/2))
	corner2 = pose.transform_from(Point2(-vehsize[1]/2, vehsize[2]/2))
	corner3 = pose.transform_from(Point2(-vehsize[1]/2, -vehsiz[2]/2))
	corner4 = pose.transform_from(Point2(vehsize[1]/2, -vehsize[2]/2))

	# vehicle base black lines
	axis.plot([corner1.x() corner2.x() corner3.x() corner4.x() corner1.x()], \
	    [corner1.y() corner2.y() corner3.y() corner4.y() corner1.y()], 'k-')

	# arm
	position = marm.forwardKinematicsPosition(p)
	position = position[0:2, :] # Todo: check rows and columns

	#style = strcat(color, '-');

	axis.plot(position[0,0:marm.arm1.dof+1], position[1,0:marm.arm1.dof+1], \
	    color=color, linewidth=width)
	axis.plot(position[0,[0,marm.arm1.dof+1:end+1]], position[1,[0,marm.arm1.dof+1:end+1]], \
	    color=color, linewidth=width)

	axis.plot(position[0,0:marm.arm1.dof+1], position[1,0:marm.arm1.dof+1], \
	    'k.', markersize=5);
	axis.plot(position[0,marm.arm1.dof+1:end], position[1,marm.arm1.dof+1:end], \
	    'k.', markersize=5);


