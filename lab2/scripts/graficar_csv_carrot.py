#!/usr/bin/env python3
import os
import csv
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory


x_path, y_path = [], []
x_odom, y_odom = [], []

package_share = get_package_share_directory('lab2')
csv_path = os.path.join(package_share, 'temp_data', 'trayectoria_robot.csv')

with open(csv_path, 'r', newline='') as file:
    reader = csv.DictReader(file)
    for row in reader:
        origen = row['origen']
        x = float(row['x'])
        y = float(row['y'])
        
        if origen == 'path':
            x_path.append(x)
            y_path.append(y)
        elif origen == 'odom':
            x_odom.append(x)
            y_odom.append(y)

plt.figure(figsize=(7,7))

plt.plot(x_path, y_path, 'b--', label='Trayectoria referencia')
plt.plot(x_odom, y_odom, 'r-', label='Trayectoria real', alpha=0.7)

plt.scatter(
    x_path[-1], 
    y_path[-1], 
    color='black', 
    s=30, 
    zorder=5, 
    label='_nolegend_'
)

plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Trayectoria real vs referencia')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()