import numpy as np

def plotPointRobot2D(figure, axis, robot, conf, color_rgb=[0.4, 0.4, 0.4]):
	# %plotPointRobot2D Plot PointRobotModel in 2D
	# %
	# %   Usage: plotRobotModel(robot, conf, color_rgb)
	# %   @robot      PointRobotModel object
	# %   @conf       robot configuration vector
	# %   @color_rgb  optional color RGB values, default is gray [0.4 0.4 0.4]

	# points
	body_points = robot.sphereCentersMat(conf)
	r = robot.sphere_radius(0)

	theta = np.linspace(0,2*np.pi, num=40)
	x = r * np.cos(theta) + body_points[0,:]
	y = r * np.sin(theta) + body_points[1,:]
	axis.plot(x, y, color=color_rgb)


