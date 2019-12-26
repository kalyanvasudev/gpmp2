import numpy as np

def plotEvidenceMap2D(figure, axis, prob_grid, origin_x, origin_y, cell_size):
	#PLOTEVIDENCEMAP2D Plot 2D evidence grid map, for 2D dataset visualization
	#
	#   Usage: PLOTEVIDENCEMAP2D(prob_grid, origin_x, origin_y, cell_size)
	#   @prob_grid              evidence grid matrix
	#   @origin_x, origin_y     origin (down-left) corner of the map
	#   @cell_size              cell size

	# map display setting
	#colormap([0.3 0.3 0.3; 0.7 0.7 0.7; 1 1 1]);

	# get X-Y coordinates
	grid_rows = prob_grid.shape[0]
	grid_cols = prob_grid.shape[1]
	grid_corner_x = origin_x + (grid_cols-1)*cell_size
	grid_corner_y = origin_y + (grid_rows-1)*cell_size

	grid_X =  np.linspace(origin_x, grid_corner_x, num=grid_cols)  
	grid_Y =  np.linspace(origin_y, grid_corner_y, num=grid_rows) 

	temp = (1-prob_grid)*2+1
	z_min = np.amin(temp)
	z_max = np.amax(temp)	

	c = axis.pcolor(grid_X, grid_Y, temp, vmin=z_min, vmax=z_max)
	figure.colorbar(c, ax=axis) # add colorbar

	axis.invert_yaxis() # TODO: check this again! same as set(gca,'YDir','normal')
	#axis equal
	axis.axis('equal')
	axis.axis([origin_x-cell_size/2, grid_corner_x+cell_size/2,
	    origin_y-cell_size/2, grid_corner_y+cell_size/2])
