import numpy as np
from gtsam import *	
from gpmp2 import *
from gpmp_utils.generate2Ddataset import generate2Ddataset
from gpmp_utils.signedDistanceField2D import signedDistanceField2D
from gpmp_utils.generateArm import generateArm
#from matplotlib import plt

def plotSignedDistanceField2D(figure, axis, field, origin_x, origin_y, cell_size, epsilon_dist=0):
	# %PLOTSIGNEDDISTANCEFIELD2D plot 2D SignedDistanceField
	# %
	# %   Usage: PLOTSIGNEDDISTANCEFIELD2D(field, origin_x, origin_y, cell_size, epsilon_dist)
	# %   @field                  field matrix
	# %   @origin_x, origin_y     origin (down-left) corner of the map
	# %   @cell_size              cell size
	# %   @epsilon_dist           optional plot obstacle safety distance, default = 0



	# get X-Y coordinates
	grid_rows = field.shape[0]
	grid_cols = field.shape[1]
	grid_corner_x = origin_x + (grid_cols-1)*cell_size
	grid_corner_y = origin_y + (grid_rows-1)*cell_size

	grid_X =  np.linspace(origin_x, grid_corner_x, num=grid_cols)  
	grid_Y =  np.linspace(origin_y, grid_corner_y, num=grid_rows) 

	z_min = np.amin(field)
	z_max = np.amax(field)
	c = axis.pcolor(grid_X, grid_Y, field, cmap='RdBu', vmin=z_min, vmax=z_max)
	figure.colorbar(c, ax=axis) # add colorbar

	#set(gca,'YDir','normal')
	axis.invert_yaxis() # TODO: check this again! same as set(gca,'YDir','normal')

	axis.axis('equal')
	axis.axis([origin_x-cell_size/2, grid_corner_x+cell_size/2,
	    origin_y-cell_size/2, grid_corner_y+cell_size/2])

	#colorbar
	axis.set_xlabel('X/m')
	axis.set_ylabel('Y/m')
	axis.set_title('Signed Distance Field')


