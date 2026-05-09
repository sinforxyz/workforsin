#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('zuoye2_result.csv')

historical = df[df['prediction'].isna()]
prediction = df[~df['prediction'].isna()]

plt.figure(figsize=(14, 7))

plt.plot(historical['day'], historical['original'], 'b.', markersize=3, alpha=0.5, label='Original Price')

plt.plot(historical['day'], historical['filtered'], 'r-', linewidth=1.5, label='Kalman Filtered')

plt.xlabel('Day')
plt.ylabel('Price')
plt.title('Stock Price Prediction (Kalman Filter)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig('zuoye2_result.png', dpi=150)
print("Saved: zuoye2_result.png")