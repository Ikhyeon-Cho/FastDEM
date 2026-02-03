import matplotlib.pyplot as plt
import numpy as np

# Data (Measured on i7-12700K, Random Data)
points = ['10k', '25k', '50k', '100k']
pcl_times = [303.32, 312.17, 1088.19, 4209.12]
nanopcl_times = [41.25, 60.12, 89.06, 137.94]

# Style Settings (Clean & Professional)
plt.style.use('default')
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial', 'Helvetica', 'DejaVu Sans']

fig, ax = plt.subplots(figsize=(10, 6))

x = np.arange(len(points))
width = 0.35

# Colors (Corporate/Trustworthy)
color_pcl = '#bdc3c7'      # Gray
color_nano = '#2980b9'     # Strong Blue

# Plot Bars
rects1 = ax.bar(x - width/2, pcl_times, width, label='PCL 1.10 (Single-thread)', color=color_pcl)
rects2 = ax.bar(x + width/2, nanopcl_times, width, label='nanoPCL (Multi-thread)', color=color_nano)

# Remove top and right spines
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)

# Labels
ax.set_ylabel('Execution Time (ms)', fontsize=12, fontweight='bold', color='#2c3e50')
ax.set_title('ICP Registration Performance', fontsize=16, fontweight='bold', color='#2c3e50', pad=20)
ax.set_xticks(x)
ax.set_xticklabels(points, fontsize=12)
ax.tick_params(axis='both', colors='#34495e')

# Grid
ax.grid(axis='y', linestyle='--', alpha=0.3)
ax.set_axisbelow(True)

# Legend
ax.legend(fontsize=11, frameon=False)

# Annotations
def autolabel(rects, is_pcl=False):
    for rect in rects:
        height = rect.get_height()
        xytext = (0, 3) if not is_pcl else (0, 1)
        color = '#2c3e50' if not is_pcl else '#7f8c8d'
        ax.annotate(f'{int(height)}',
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=xytext,
                    textcoords="offset points",
                    ha='center', va='bottom', fontsize=10, fontweight='bold', color=color)

autolabel(rects1, is_pcl=True)
autolabel(rects2)

# Speedup Chips (Badges)
for i in range(len(points)):
    speedup = pcl_times[i] / nanopcl_times[i]
    
    # Position above the PCL bar
    y_pos = pcl_times[i] + (pcl_times[i] * 0.05)
    if i == 3: y_pos += 200 # Adjust for last bar to avoid crop
    
    ax.text(x[i], y_pos, f'{speedup:.1f}x Faster', 
            ha='center', va='bottom', fontsize=11, fontweight='bold', 
            color='#c0392b', bbox=dict(facecolor='#fff0f0', edgecolor='#c0392b', boxstyle='round,pad=0.3', alpha=0.8))

plt.tight_layout()
plt.savefig('benchmark_icp_clean.png', dpi=300)
print("Clean graph saved to benchmark_icp_clean.png")
