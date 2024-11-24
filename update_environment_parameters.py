import numpy as np
import Read_Lidar
from init_params import *

# Read lidar data
Points360 = Read_Lidar(X[:, Step_Counter], X_g[:, Step_Counter], Lidar_Range, map)

# Compute goal vector
Goal_Vector[:, Step_Counter] = np.array([
    X_g[0, Step_Counter] - X[0, Step_Counter],
    X_g[1, Step_Counter] - X[2, Step_Counter]
])

# Calculate goal angle and direction
Goal_angle = np.degrees(np.arctan2(Goal_Vector[0, Step_Counter], Goal_Vector[0, Step_Counter]))
Goal_Direction = Goal_angle - Robot.Heading

# Compute membership function values for Lidar data
MF_Lid = MF_Lidar(Points360)

# Calculate angle differences
Phi2Goal = MF_Lidar_Angle - Goal_Direction

# Saturate angles to the range [-180, 180]
SaturatedPHI2Goal = Phi2Goal + (360 * (Phi2Goal < -180)) - (360 * (Phi2Goal > 180))

# Generate weight membership functions using generalized bell function
weight_MF = gbellmf(SaturatedPHI2Goal, [bell_size, bell_coff, 0])

# Calculate preference membership function
Preference_MF = (weight_MF * 0.7 + 0.3) * (1 - MF_Lid)
