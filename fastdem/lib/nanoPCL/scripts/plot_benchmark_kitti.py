import matplotlib.pyplot as plt
import numpy as np

# Data (KITTI Scan-to-Scan, ~125k points)
methods = ['PCL ICP', 'nanoPCL (P2P)', 'nanoPCL (P2Plane)', 'nanoPCL (VGICP)']
times = [307.72, 12.03, 12.56, 15.68]
colors = ['#bdc3c7', '#2980b9', '#3498db', '#5dade2'] # Gray vs Blues

# Style Settings
plt.style.use('default')
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial', 'Helvetica', 'DejaVu Sans']

fig, ax = plt.subplots(figsize=(10, 6))

y_pos = np.arange(len(methods))

# Plot Horizontal Bars
rects = ax.barh(y_pos, times, align='center', color=colors)
ax.set_yticks(y_pos)
ax.set_yticklabels(methods, fontsize=12, fontweight='bold', color='#34495e')
ax.invert_yaxis()  # labels read top-to-bottom

# Remove spines
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
ax.spines['bottom'].set_visible(False)
ax.spines['left'].set_visible(False)

# Labels
ax.set_xlabel('Processing Time (ms) - Lower is Better', fontsize=12, fontweight='bold', color='#7f8c8d')
ax.set_title('Real-world Registration Speed (KITTI, ~125k pts)', fontsize=16, fontweight='bold', color='#2c3e50', pad=20)
ax.tick_params(axis='x', colors='#7f8c8d')
ax.tick_params(axis='y', length=0) # Hide y ticks

# Add values to bars
for i, rect in enumerate(rects):
    width = rect.get_width()
    label_x_pos = width + 5
    color = '#2c3e50'
    
    # Value
    ax.text(label_x_pos, rect.get_y() + rect.get_height()/2, 
            f'{width:.1f} ms', 
            va='center', fontsize=11, fontweight='bold', color=color)
    
    # Speedup badge for nanoPCL
    if i > 0: 
        speedup = times[0] / width
        ax.text(label_x_pos + 80, rect.get_y() + rect.get_height()/2, 
                f'({speedup:.1f}x Faster)', 
                va='center', fontsize=11, fontweight='bold', color='#e74c3c')

plt.tight_layout()
plt.savefig('benchmark_kitti_clean.png', dpi=300)
print("Graph saved to benchmark_kitti_clean.png")
