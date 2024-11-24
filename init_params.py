import numpy as np
import imageio.v3 as iio
import matplotlib.pyplot as plt
from scipy.stats import norm  # For Gaussian Membership Function

# Time and Counter Parameters
T_s = 0.05  # Time step
T_f = 10  # Final Time
T_b = 0  # Break Time
T_k = 0.1  # Window Time
max_expected_size = round(T_f / T_s)
Run_Timer = np.zeros(max_expected_size)  # Time vector
Step_Counter = 0
Window_Size = round(T_k / T_s)

# Initial State
x_0, x_dot_0, y_0, y_dot_0 = 0, 2, 0, 2
theta_0, theta_dot_0 = 70, 0
X0 = np.array([x_0, x_dot_0, y_0, y_dot_0, theta_0, theta_dot_0])

V = np.sqrt(x_dot_0**2 + y_dot_0**2)
Omega = theta_dot_0

# Init State Recorder Matrices
X = np.tile(X0[:, np.newaxis], max_expected_size)
X_g = np.tile(np.array([300, 300, 0])[:, np.newaxis], max_expected_size)
Xd0 = np.array([X_g[0, 0], x_dot_0, X_g[1, 0], y_dot_0, X_g[2, 0], theta_dot_0])
Xd = np.tile(Xd0[:, np.newaxis], max_expected_size)

# Function to calculate distance to goal
def dist2goal(X, X_g):
    """
    Calculate the Euclidean distance between the current position and the goal.

    Parameters:
    X (list or array): Current position [x, y].
    X_g (list or array): Goal position [xg, yg].

    Returns:
    float: Distance to the goal.
    """
    x, y = X[0], X[1]
    xg, yg = X_g[0], X_g[1]
    return np.sqrt((x - xg)**2 + (y - yg)**2)

Dist2Goal = np.zeros(max_expected_size)
Dist2Goal[0] = dist2goal([X[0, 0], X[2, 0]], X_g[:, 0])

Xd_sim = np.tile(Xd0[:, np.newaxis], Window_Size)
Goal_Vector = np.zeros((2, max_expected_size))
Goal_Vector_sim = np.zeros((2, Window_Size))

# Robot Parameters
Lidar_Range = 30
m = 2
Robot = {
    "m": m,
    "Lidar_Range": Lidar_Range,
    "X": X0,
    "Xd": X0,
    "Heading": theta_0,
}

Robot_Sim = Robot.copy()
Robot_Sim["Window_Size"] = Window_Size
Robot_Sim["Xd_sim"] = Xd_sim

# Fuzzy Network Parameters
Number_of_Membership_Functions = 7
NMF = Number_of_Membership_Functions
Number_of_Rules = 0
Max_Number_of_Rules = 100
Number_of_Inputs = 6
Number_of_Outputs = 2
W = np.zeros((Number_of_Rules, Number_of_Outputs))
FN_Phi = np.zeros(Number_of_Rules)
Membership_Functions_Params = np.zeros((Number_of_Inputs, 2 * Number_of_Membership_Functions))
Gamma = 0.8
bell_size = 70
bell_coff = 3
sample = Robot["X"]

Dist_MF_L2F = 15
MF_Lidar_Angle = np.arange(0, 360, Dist_MF_L2F)

Num_MF_L2F = int(360 / Dist_MF_L2F)
Membership_Lidar = np.zeros((2 * Dist_MF_L2F + 1, Num_MF_L2F))
MF_Lidar_ = np.zeros(Num_MF_L2F)

# Gaussian Membership Function
def gaussmf(x, params):
    sigma, mean = params
    return norm.pdf(x, loc=mean, scale=sigma)

MF_L2F = gaussmf(np.arange(-Dist_MF_L2F, Dist_MF_L2F + 1), [(Dist_MF_L2F / 4) / np.sqrt(-2 * np.log(0.5)), 0])

# Augmented Lidar Function
def Lidar_Augmented(x):
    return np.concatenate([x[-Dist_MF_L2F:], x, x[:Dist_MF_L2F]])

Max_Lidar = np.sum(Lidar_Range * MF_L2F)

# Membership Function for Lidar
def MF_Lidar(Points360):
    return Lidar2Fuzzy(Points360, Lidar_Augmented, Membership_Lidar, MF_Lidar_, Dist_MF_L2F, Num_MF_L2F, MF_L2F, Max_Lidar)

MeanMat = []
VariMat = []
Var0 = 1
W = []
RulesNum = 0

# General Gaussian Membership Function
def MF(X, M, S):
    return gaussmf(X, [S, M])

# %% Environmental Parameters
map_rgb = iio.imread("maps/star_300.png")  # Read the map using imageio
plt.subplot(2, 2, (1, 3))
plt.imshow(map_rgb)
plt.axis("off")  # Hide axes for a cleaner display
# plt.show()
plt.savefig('plot_map.png')

