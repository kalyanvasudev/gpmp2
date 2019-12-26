def plotArm(figure, axis, arm, conf, color, width):
	# PLOTARM Plot Arm class in 3D
	#
	#   Usage: PLOTARM(arm, conf, color, width)
	#   @arm    Arm object
	#   @conf   arm configuration vector
	#   @color  color string, use plot convention, e.g. 'r' is red
	#   @width  line width

	position = arm.forwardKinematicsPosition(conf)
	# marker='-'
	axis.plot(position[0,:], position[1,:], position[2,:], color=color, linewidth=width)

	axis.plot(position[0,0:end], position[1,0:end], position[2,0:end], 'k.', markersize=10*width)

