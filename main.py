import time

# %% Initializing
start_time = time.time()

## Load initialization parameters (Replace with your actual initialization function)
import numpy as np
import imageio.v3 as iio
import matplotlib.pyplot as plt
from PIL import Image
import Read_Lidar

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

# Load the image using imageio
map_rgb = iio.imread("maps/star_300.png")

# Display the RGB image
plt.subplot(2, 2, (1, 3))
plt.imshow(map_rgb)
plt.axis("off")  # Hide axes for a cleaner display

# Convert NumPy array to PIL Image for grayscale conversion
map_rgb_pil = Image.fromarray(map_rgb)

# Convert to grayscale
map_gray = map_rgb_pil.convert('L')

# Convert grayscale to NumPy array
map_gray_array = np.array(map_gray)

# Binarize the grayscale image (threshold = 0.95)
threshold = int(0.95 * 255)
map_bin = map_gray_array > threshold

# Extract binary map matrix
map_matrix = map_bin.astype(int)

# Display the binary map
plt.subplot(2, 2, (2, 4))
plt.imshow(map_matrix, cmap='gray')
plt.axis("off")
plt.title("Binary Map")
plt.show()

# %% Live Loop
while Step_Counter < max_expected_size:
    Step_Counter += 1
    Run_Timer[Step_Counter - 1] = time.time() - start_time

    # Read simulation and environment parameters
    # Read lidar data
    Points360 = Read_Lidar(X[:, Step_Counter], X_g[:, Step_Counter], Lidar_Range, map_matrix)

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



    # Path planner and controller algorithm
    ManageFuzzySys()  # Replace with your fuzzy system management function
    kinodynamics()    # Replace with your kinematic and dynamic handling function

    # Simulation and send commands to robot
    update_dynamic_parameters()  # Replace with your dynamic parameter update function

    # Run RL and update network
    learning_function()  # Replace with your reinforcement learning function

    # Replot and update GUIs
    update_presentation()  # Replace with your GUI update function

    # End loop condition
    if Run_Timer[Step_Counter - 1] > T_f:
        break
    if Dist2Goal[Step_Counter - 1] < 0.2:
        T_b = Run_Timer[Step_Counter - 1]
        break

# End of loop
end_time = time.time()
print(f"Total elapsed time: {end_time - start_time} seconds")
