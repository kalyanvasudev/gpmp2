import numpy as np
from gtsam import *	
from gpmp2 import *
from gpmp_utils.generate2Ddataset import generate2Ddataset
from gpmp_utils.signedDistanceField2D import signedDistanceField2D
from gpmp_utils.generateArm import generateArm
from matplotlib import plt

def plotSignedDistanceField3D(figure, axis, field, origin, cell_size, epsilon_dist=0, marker_size=10):
	# %PLOTSIGNEDDISTANCEFIELD3D plot 3D SignedDistanceField
	# %
	# %   Usage: PLOTSIGNEDDISTANCEFIELD3D(field, origin, cell_size, epsilon_dist)
	# %   @field                  field 3D matrix
	# %   @origin                 origin of the map
	# %   @cell_size              cell size
	# %   @epsilon_dist           optional plot obstacle safety distance, default = 0
	# %   @marker_size            marker size, default = 0
	# % Here, note that axis = figure.gca(projection='3d')

	# get X-Y coordinates
	grid_rows = field.shape[0]
	grid_cols = field.shape[1]
	grid_z = field.shape[2]
	grid_corner_x = origin(0) + (grid_cols-1)*cell_size
	grid_corner_y = origin(1) + (grid_rows-1)*cell_size
	grid_corner_z = origin(2) + (grid_z-1)*cell_size
	grid_X = np.linspace(origin[0], grid_corner_x, num=grid_cols)  
	grid_Y = np.linspace(origin[1], grid_corner_y, num=grid_rows)  
	grid_Z = np.linspace(origin[2], grid_corner_z, num=grid_z)  


	indexes =  np.nonzero(filed < epsilon_dist)
	x, y, z = indexes[0,:], indexes[1,:], indexes[2,:]

	# Todo: Test this
	axis.scatter(grid_X(y), grid_Y(x), grid_Z(z), '.', 'r', markersize=marker_size)



	#axis equal
	axis.axis([origin(0)-cell_size/2, grid_corner_x+cell_size/2,
	    origin(1)-cell_size/2, grid_corner_y+cell_size/2,
	    origin(2)-cell_size/2, grid_corner_z+cell_size/2])

	axis.set_xlabel('X/m')
	axis.set_ylabel('Y/m')
	axis.set_zlabel('Z/m')
	axis.set_title('Signed Distance Field')