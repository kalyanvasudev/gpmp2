import numpy as np
def plotSphere(figure, axis, radius, center, color):
	# Make data
	u = np.linspace(0, 2 * np.pi, 100)
	v = np.linspace(0, np.pi, 100)
	x = radius * np.outer(np.cos(u), np.sin(v)) + center[0]
	y = radius * np.outer(np.sin(u), np.sin(v)) + center[1]
	z = radius * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]

	# Plot the surface
	axis.plot_surface(x, y, z, rstride=4, cstride=4, color=color)



def plotRobotModel(figure, axis, robot, conf, color_rgb=[(0.4, 0.4, 0.4)]):
	#plotRobotModel Plot RobotModel class in 3D, visualize the body spheres
	#   also it can plot any child class of RobotModelm like ArmModel
	# 
	#   Usage: plotRobotModel(robot, conf, color_rgb)
	#   @robot      RobotModel(or child) object
	#   @conf       robot configuration vector
	#   @color_rgb  optional color RGB values, default is gray [0.4 0.4 0.4]

	# points
	body_points = robot.sphereCentersMat(conf)

	for i in range(robot.nr_body_spheres()):
		# TODO: check if it is body_points[:,i] or body_point[i,:]
		plotSphere(figure, axis, robot.sphere_radius[i], body_points[:,i], color=color_rgb)

