import numpy as np
def plotSphere(figure, axis, radius, center, color):
	# Make data
	u = np.linspace(0, 2 * np.pi, 100)
	
	x = radius * np.cos(u) + center[0]
	y = radius * np.sin(u) + center[1]
	

	# Plot the surface
	axis.plot(x, y, color=color)



def plotRobotModel2D(figure, axis, robot, conf, color_rgb=[0.4, 0.4, 0.4]):
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
		plotSphere(figure, axis, robot.sphere_radius(i), body_points[:,i], color=color_rgb)

