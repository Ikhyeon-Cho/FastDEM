import matplotlib.pyplot as plt
import numpy as np

# Data
points = ['10k', '25k', '50k', '100k']
pcl_times = [303.32, 312.17, 1088.19, 4209.12]
nanopcl_times = [41.25, 60.12, 89.06, 137.94]

# Style Settings
plt.style.use('dark_background')
fig, ax = plt.subplots(figsize=(10, 5))
fig.patch.set_facecolor('#0d1117') # GitHub Dark Dimmed background
ax.set_facecolor('#0d1117')

x = np.arange(len(points))
width = 0.35

# Colors (Modern Neon)
color_pcl = '#ff6b6b'      # Soft Red
color_nano = '#4ec9b0'     # VS Code Cyan (TypeScript color)

# Plot Bars
rects1 = ax.bar(x - width/2, pcl_times, width, label='PCL 1.10', color=color_pcl, alpha=0.8)
rects2 = ax.bar(x + width/2, nanopcl_times, width, label='nanoPCL', color=color_nano, alpha=1.0)

# Remove borders (spines)
for spine in ax.spines.values():
    spine.set_visible(False)

# Labels
ax.set_ylabel('Time (ms)', fontsize=10, color='#8b949e')
ax.set_title('ICP Registration Speed (Lower is Better)', fontsize=14, color='white', fontweight='bold', pad=20)
ax.set_xticks(x)
ax.set_xticklabels(points, fontsize=12, color='white')
ax.tick_params(axis='y', colors='#8b949e')
ax.grid(axis='y', linestyle='--', alpha=0.1)

# Legend
legend = ax.legend(frameon=False, fontsize=11)
plt.setp(legend.get_texts(), color='#c9d1d9')

# Annotations (Speedup Chips)
for i in range(len(points)):
    speedup = pcl_times[i] / nanopcl_times[i]
    
    # Calculate position
    height_pcl = pcl_times[i]
    height_nano = nanopcl_times[i]
    
    # Draw arrow or line indicating difference? No, just text is cleaner.
    # Place text above the higher bar (PCL)
    ax.text(x[i], height_pcl + 100, f'{speedup:.1f}x Faster', 
            ha='center', va='bottom', fontsize=11, fontweight='bold', color=color_nano)

# Add values on top of nanoPCL bars only (Focus on how fast nanoPCL is)
for rect in rects2:
    height = rect.get_height()
    ax.text(rect.get_x() + rect.get_width()/2., height + 20,
            f'{int(height)}ms',
            ha='center', va='bottom', color=color_nano, fontsize=10, fontweight='bold')

plt.tight_layout()
plt.savefig('benchmark_icp_dark.png', dpi=300, facecolor='#0d1117')
print("Stylish graph saved to benchmark_icp_dark.png")
