import matplotlib.pyplot as plt
import numpy as np

# Data (Measured on i7-12700K equivalent, Single-thread PCL vs Multi-thread nanoPCL)
points = ['10k', '25k', '50k', '100k']
pcl_times = [303.32, 312.17, 1088.19, 4209.12]
nanopcl_times = [41.25, 60.12, 89.06, 137.94]

# Setup
x = np.arange(len(points))
width = 0.35

fig, ax = plt.subplots(figsize=(10, 6))

# Plot bars
rects1 = ax.bar(x - width/2, pcl_times, width, label='PCL (v1.10)', color='#95a5a6', edgecolor='white')
rects2 = ax.bar(x + width/2, nanopcl_times, width, label='nanoPCL', color='#3498db', edgecolor='white')

# Labels and Title
ax.set_ylabel('Execution Time (ms) - Lower is Better', fontsize=12, fontweight='bold')
ax.set_title('ICP Registration Performance (PCL vs nanoPCL)', fontsize=16, fontweight='bold', pad=20)
ax.set_xticks(x)
ax.set_xticklabels(points, fontsize=12)
ax.legend(fontsize=12)

# Grid
ax.grid(axis='y', linestyle='--', alpha=0.3)
ax.set_axisbelow(True)

# Add text labels on bars
def autolabel(rects):
    for rect in rects:
        height = rect.get_height()
        ax.annotate(f'{int(height)}',
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom', fontsize=10, fontweight='bold')

autolabel(rects1)
autolabel(rects2)

# Add Speedup annotations
for i in range(len(points)):
    speedup = pcl_times[i] / nanopcl_times[i]
    ax.annotate(f'{speedup:.1f}x',
                xy=(x[i], max(pcl_times[i], nanopcl_times[i]) + 200),
                ha='center', va='bottom', fontsize=12, color='#e74c3c', fontweight='bold')

# Styling
plt.tight_layout()

# Save
plt.savefig('benchmark_icp.png', dpi=300)
print("Graph saved to benchmark_icp.png")
