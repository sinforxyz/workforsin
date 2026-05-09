#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('zuoye2_result.csv')

hist = df[df['prediction'].isna()]
pred = df[~df['prediction'].isna()]

plt.figure(figsize=(14, 7))
plt.plot(hist['day'], hist['original'], 'b.', markersize=3, alpha=0.5, label='origin price')
plt.plot(hist['day'], hist['filtered'], 'r-', linewidth=1.5, label='kalman')

if len(pred) > 0:
    plt.plot(pred['day'], pred['prediction'], 'g--', linewidth=1.5, marker='o', markersize=4, label='predict price')
    plt.axvline(x=pred['day'].iloc[0] - 0.5, color='gray', linestyle=':', alpha=0.7)

plt.xlabel('day')
plt.ylabel('price')
plt.title('zuoye 2')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('zuoye2_result.png', dpi=150)
print("图片已保存: zuoye2_result.png")