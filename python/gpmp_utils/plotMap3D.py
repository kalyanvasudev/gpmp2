def plotMap3D(figure, axis, corner_idx, origin, cell_size):
    #PLOTMAP3D Plot 3D evidence grid map, for visualization of 3D dataset
    #
    #   Usage: PLOTMAP3D(corner_idx, origin, cell_size)
    #   @corner_idx   corner index [x1, x2, y1, y2, z1, z2], generated by generate3Ddataset
    #   @origin       origin (down-left) corner of the map
    #   @cell_size    cell size

    # for each obj

    #colormap([0.4, 0.4, 0.4])

    for i in range(corner_idx.shape[0]):
        x1 = corner_idx[i, 0]*cell_size + origin[0]
        x2 = corner_idx[i, 1]*cell_size + origin[0]
        y1 = corner_idx[i, 2]*cell_size + origin[1]
        y2 = corner_idx[i, 3]*cell_size + origin[1]
        z1 = corner_idx[i, 4]*cell_size + origin[2]
        z2 = corner_idx[i, 5]*cell_size + origin[2]
        axis.plot_surface([x1, x2; x1, x2], [y1, y1; y2, y2], [z1, z1; z1, z1])
        axis.plot_surface([x1, x2; x1, x2], [y1, y1; y2, y2], [z2, z2; z2, z2])
        axis.plot_surface([x1, x1; x1, x1], [y1, y1; y2, y2], [z1, z2; z1, z2])
        axis.plot_surface([x2, x2; x2, x2], [y1, y1; y2, y2], [z1, z2; z1, z2])
        axis.plot_surface([x1, x2; x1, x2], [y1, y1; y1, y1], [z1, z1; z2, z2])
        axis.plot_surface([x1, x2; x1, x2], [y2, y2; y2, y2], [z1, z1; z2, z2])

    axis.axis('equal')
