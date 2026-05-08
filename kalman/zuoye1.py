#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

files = [
    ('zuoye1_1.csv', '文件1'),
    ('zuoye1_2.csv', '文件2'),
    ('zuoye1_3.csv', '文件3'),
    ('zuoye1_4.csv', '文件4')
]

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
axes = axes.flatten()

for i, (filename, title) in enumerate(files):
    try:
        df = pd.read_csv(filename)
        ax = axes[i]
        
        ax.plot(df['time'], df['original'], 'b.', markersize=0.5, alpha=0.3, label='原始数据')
        ax.plot(df['time'], df['filtered'], 'r-', linewidth=1.5, label='卡尔曼滤波')
        ax.plot(df['time'], df['fitted'], 'g--', linewidth=1.5, label='拟合直线')
        
        ax.set_xlabel('时间')
        ax.set_ylabel('数值')
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
    except FileNotFoundError:
        axes[i].text(0.5, 0.5, f'{filename} 不存在', ha='center', va='center')
        axes[i].set_title(title)

plt.tight_layout()
plt.savefig('zuoye1_result.png', dpi=150)
print("图片已保存: zuoye1.png")