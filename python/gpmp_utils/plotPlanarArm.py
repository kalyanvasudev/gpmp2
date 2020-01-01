import numpy as np
def plotPlanarArm(figure, axis, arm, conf, color, width):
	# %PLOTPLANARARM Plot Arm class in 2D
	# %
	# %   Usage: PLOTPLANARARM(arm, conf, color, width)
	# %   @arm    Arm object
	# %   @conf   arm configuration vector
	# %   @color  color string, use plot convention, e.g. 'r' is red
	# %   @width  line width

	# TODO: check if rows arnd colums are correct
	position = arm.forwardKinematicsPosition(conf)
	position = position[0:2, :]
	position = np.append(np.asarray([0,0]).reshape(2,1), position, axis=1)

	# marker='-'
	axis.plot(position[0,:], position[1,:],color=color, linewidth=width)
	axis.plot(position[0,:], position[1, :], 'k.', markersize=20)