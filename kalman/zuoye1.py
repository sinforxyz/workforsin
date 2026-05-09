#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

files = [
    ('zuoye1_1.csv', 'File 1'),
    ('zuoye1_2.csv', 'File 2'),
    ('zuoye1_3.csv', 'File 3'),
    ('zuoye1_4.csv', 'File 4')
]

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
axes = axes.flatten()

for i, (filename, title) in enumerate(files):
    try:
        df = pd.read_csv(filename)
        ax = axes[i]
        
        ax.plot(df['time'], df['original'], 'b.', markersize=0.5, alpha=0.3, label='Original')
        ax.plot(df['time'], df['filtered'], 'r-', linewidth=1.5, label='Kalman Filtered')
        
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
    except FileNotFoundError:
        axes[i].text(0.5, 0.5, f'{filename} not found', ha='center', va='center')
        axes[i].set_title(title)

plt.tight_layout()
plt.savefig('zuoye1.png', dpi=150)
print("Image saved: zuoye1.png")