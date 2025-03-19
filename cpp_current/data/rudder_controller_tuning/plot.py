import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Load CSV file
search_type = "gradient_descent"
data = pd.read_csv(search_type + ".csv")
# data = pd.read_csv("exhaustive_search.csv")

# Extract columns
P = data['P']
I = data['I']
D = data['D']
error = data['error_avg']

# Find the minimum error and its corresponding P, I, D
min_error_idx = error.idxmin()
min_P, min_I, min_D, min_error = P[min_error_idx], I[min_error_idx], D[min_error_idx], error[min_error_idx]

# Create 3D scatter plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Scatter plot with color based on error
sc = ax.scatter(P, I, D, c=error, cmap='coolwarm', alpha=0.7)

# Highlight the minimum error point
ax.scatter(min_P, min_I, min_D, color='black', s=100, label=f'P={min_P:.2f}, I={min_I:.2f}, D={min_D:.2f}, Min Error: {min_error:.2f}')
ax.text(min_P, min_I, min_D, f"P={min_P:.2f}\nI={min_I:.2f}\nD={min_D:.2f}", color='black', fontsize=10, verticalalignment='bottom')

# Labels and title
ax.set_xlabel("P")
ax.set_ylabel("I")
ax.set_zlabel("D")
ax.set_title("PID Tuning Effect on Error")

# Add color bar
cbar = plt.colorbar(sc, ax=ax, shrink=0.5, aspect=10)
cbar.set_label("Error")

# Show legend
ax.legend()

# Show plot
plt.show()
plt.savefig(search_type + ".png")

# Print the optimal values
print(f"Minimum Error: {min_error:.4f} at P={min_P}, I={min_I}, D={min_D}")


# Aldo plot the waypoint navigation
data_path = pd.read_csv("waypoint_navigation.csv")

# Extract x, y columns
x = data_path['x']
y = data_path['y']

# Create 2D path plot
plt.figure(figsize=(8, 6))
plt.plot(x, y, marker='o', linestyle='-', color='b', alpha=0.7)
plt.xlabel("X Position")
plt.ylabel("Y Position")
plt.title("Vehicle Path")
plt.grid(True)
plt.show()
plt.savefig("waypoint_navigation.png")