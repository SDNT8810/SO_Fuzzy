import numpy as np

def Read_Lidar(X, X_g, Lidar_Range, map_):
    x = X[0]
    y = X[2]
    t = X[4]
    l = Lidar_Range
    s = map_.shape
    m2p = s[0] / X_g[0]
    dl = l * m2p
    rdl = int(np.floor(dl + 1))
    
    # Augment map with extra space around boundaries
    map_augmented = np.ones((2 * rdl + s[0] + 4, 2 * rdl + s[1] + 4))
    map_augmented[rdl:rdl + s[0], rdl:rdl + s[1]] *= map_
    
    # Robot's position in augmented map coordinates
    px = rdl + x * m2p
    py = rdl + y * m2p
    
    # Initialize lidar points array
    Points360 = np.zeros(360)
    
    for i in range(360):
        nt = t + i  # Current angle
        for j in range(1, int(dl) + 1):
            # Compute new positions
            nx = x + l * (j / dl) * np.cos(np.radians(nt))
            ny = y + l * (j / dl) * np.sin(np.radians(nt))
            
            # Map to pixel coordinates
            nxp = max(int(np.floor(rdl + nx * m2p)), 0)
            nyp = max(int(np.floor(rdl + ny * m2p)), 0)
            
            nxp = min(nxp, map_augmented.shape[0] - 1)
            nyp = min(nyp, map_augmented.shape[1] - 1)
            
            # Calculate distance from robot to the point
            dxp = nxp - px
            dyp = nyp - py
            
            # Check if the point intersects an obstacle
            if map_augmented[nxp, nyp] == 0:
                Points360[i] = (1 - j / dl) * l
                break

    return Points360
