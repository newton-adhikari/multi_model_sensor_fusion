#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('fusion_log.csv')

# Convert Series to NumPy arrays
x = df['px'].to_numpy()
y = df['py'].to_numpy()

plt.plot(x, y, marker='o', linestyle='-')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Fused Trajectory')
plt.grid(True)
plt.axis('equal')
plt.show()
