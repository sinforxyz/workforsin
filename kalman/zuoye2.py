#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('homework2_result.csv')

hist = df[df['prediction'].isna()]
pred = df[~df['prediction'].isna()]

plt.figure(figsize=(14, 7))
plt.plot(hist['day'], hist['original'], 'b.', markersize=3, alpha=0.5, label='原始价格')
plt.plot(hist['day'], hist['filtered'], 'r-', linewidth=1.5, label='卡尔曼滤波')

if len(pred) > 0:
    plt.plot(pred['day'], pred['prediction'], 'g--', linewidth=1.5, marker='o', markersize=4, label='预测价格')
    plt.axvline(x=pred['day'].iloc[0] - 0.5, color='gray', linestyle=':', alpha=0.7)

plt.xlabel('天数')
plt.ylabel('价格')
plt.title('作业二：股票价格预测')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('zuoye2_result.png', dpi=150)
print("图片已保存: zuoye2_result.png")