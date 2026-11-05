#!/usr/bin/env python3

import os
import csv
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory

package_share = get_package_share_directory('lab2')
output_dir = os.path.join(package_share, 'temp_data')

traj_csv_path = os.path.join(output_dir, 'trayectoria_robot.csv')
signals_csv_path = os.path.join(output_dir, 'senales_control.csv')
lab1_csv_path = os.path.join(output_dir, 'trayectoria_lab1.csv')

x_lab1, y_lab1 = [], []
x_odom, y_odom = [], []

with open(traj_csv_path, 'r', newline='') as file:
    reader = csv.DictReader(file)
    for row in reader:
        origen = row['origen']
        x = float(row['x'])
        y = float(row['y'])

        if origen == 'odom':
            x_odom.append(x)
            y_odom.append(y)

with open(lab1_csv_path, 'r', newline='') as file:
    reader = csv.DictReader(file)
    for row in reader:
        origen = row['origen']
        x = float(row['x'])
        y = float(row['y'])

        if origen == 'odom':
            x_lab1.append(x)
            y_lab1.append(y)

#lectura de datos de señales de control (lineal y angular)
t_lin, pref_lin, vs_lin, preal_lin = [], [], [], []
t_ang, pref_ang, vs_ang, preal_ang = [], [], [], []

controller_name = 'P'


with open(signals_csv_path, 'r', newline='') as file:
    reader = csv.DictReader(file)
    for row in reader:
        controller_name = row['controlador']
        t = float(row['timestamp'])
        modo = row['modo']
        pref = float(row['pref'])
        vs = float(row['vs'])
        preal = float(row['preal'])

        if modo == 'lineal':
            t_lin.append(t)
            pref_lin.append(pref)
            vs_lin.append(vs)
            preal_lin.append(preal)
        elif modo == 'angular':
            t_ang.append(t)
            pref_ang.append(pref)
            vs_ang.append(vs)
            preal_ang.append(preal)

#señales de cntrol lineal
if t_lin:
    t0_lin = t_lin[0]
    t_lin_norm = [t - t0_lin for t in t_lin]

    plt.figure(figsize=(8, 5))
    plt.plot(t_lin_norm, pref_lin, 'r--', label=r'$P_{ref}(t)$ (Referencia)')
    plt.plot(t_lin_norm, vs_lin, 'g-', label=r'$V_s(t)$ (Actuación)')
    plt.plot(t_lin_norm, preal_lin, 'b-', label=r'$P_{real}(t)$ (Salida Real)')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Posición / Desplazamiento [m]')
    plt.title(f'Control {controller_name} - Señales de control lineal')
    plt.legend()
    plt.grid(True)

#señales de control angular
if t_ang:
    t0_ang = t_ang[0]
    t_ang_norm = [t - t0_ang for t in t_ang]

    plt.figure(figsize=(8, 5))
    plt.plot(t_ang_norm, pref_ang, 'r--', label=r'$P_{ref}(t)$ (Referencia)')
    plt.plot(t_ang_norm, vs_ang, 'g-', label=r'$V_s(t)$ (Actuación)')
    plt.plot(t_ang_norm, preal_ang, 'b-', label=r'$P_{real}(t)$ (Salida Real)')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Orientación / Ángulo [rad]')
    plt.title(f'Control {controller_name} - Señales de control angular')
    plt.legend()
    plt.grid(True)

#trayectoria 2D
plt.figure(figsize=(7, 7))

if x_lab1 and y_lab1:
    plt.plot(x_lab1, y_lab1, 'b--', label='Trayectoria referencia (Lab 1)')
    plt.scatter(
        x_lab1[-1],
        y_lab1[-1],
        color='black',
        s=30,
        zorder=5,
        label='_nolegend_',
    )

if x_odom and y_odom:
    plt.plot(x_odom, y_odom, 'r-', label='Trayectoria controlada', alpha=0.7)

plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title(f'Trayectoria 2D - Control {controller_name}')
plt.legend()
plt.grid(True)
plt.axis('equal')

plt.show()