import numpy as np
import imageio.v3 as iio
import matplotlib.pyplot as plt

# %% Time and Counter Parameters
T_s = 0.05  # Time step
T_f = 10  # Final Time
T_b = 0  # Break Time
T_k = 0.1  # Window Time
max_expected_size = round(T_f / T_s)
Run_Timer = np.zeros(max_expected_size)  # Time vector
Step_Counter = 0
Window_Size = round(T_k / T_s)

# %% Initial State
x_0 = 0
x_dot_0 = 2
y_0 = 0
y_dot_0 = 2
theta_0 = 70
theta_dot_0 = 0
X0 = np.array([x_0, x_dot_0, y_0, y_dot_0, theta_0, theta_dot_0])

V = np.sqrt(x_dot_0**2 + y_dot_0**2)
Omega = theta_dot_0

# %% Init State Recorder Matrices
X = X0[:, None] + np.zeros((len(X0), max_expected_size))
X_g = np.array([300, 300, 0])[:, None] + np.zeros((3, max_expected_size))
Xd0 = np.array([X_g[0, 0], x_dot_0, X_g[1, 0], y_dot_0, X_g[2, 0], theta_dot_0])
Xd = Xd0[:, None] + np.zeros((len(X0), max_expected_size))
Dist2Goal = np.zeros(max_expected_size)
Xd_sim = np.zeros((len(X0), Window_Size)) + Xd0[:, None]
Goal_Vector = np.zeros((2, max_expected_size))
Goal_Vector_sim = np.zeros((2, Window_Size))

# %% Robot Parameters
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

# %% Environmental Parameters
map_rgb = iio.imread("maps/star_300.png")  # Read the map using imageio
plt.subplot(2, 2, (1, 3))
plt.imshow(map_rgb)
plt.axis("off")  # Hide axes for a cleaner display
plt.show()

# Initialize obstacles
X_Obstacles_0 = np.array([
    [1, 2],
    [2, 5],
    [3, 6],
    [4, 2],
    [5, 1],
    [5, 8],
])
X_Obstacles = np.zeros((X_Obstacles_0.shape[0], 2, max_expected_size)) + X_Obstacles_0[:, :, None]
X_Near_Obstacles = None  # Placeholder, depends on your implementation of obstacle detection
Points360 = np.zeros(360)
