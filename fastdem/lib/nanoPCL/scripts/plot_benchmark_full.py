import matplotlib.pyplot as plt
import numpy as np

# Set style
plt.style.use('default')
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial', 'Helvetica', 'DejaVu Sans']

# Create figure with 3 subplots
fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(15, 6))
fig.suptitle('nanoPCL Performance Benchmarks', fontsize=20, fontweight='bold', color='#2c3e50', y=0.98)

# Colors
color_pcl = '#95a5a6'      # Gray
color_nano = '#2980b9'     # Blue
color_nano_light = '#3498db'
color_nano_lighter = '#5dade2'

# --- Subplot 1: Registration Speed (KITTI 120k) ---
reg_labels = ['PCL ICP', 'nano P2P', 'nano Plane', 'nano VGICP']
reg_times = [307.7, 12.0, 12.6, 15.7]  # ms
reg_colors = [color_pcl, color_nano, color_nano_light, color_nano_lighter]

bars1 = ax1.bar(reg_labels, reg_times, color=reg_colors, width=0.6)
ax1.set_title('Registration Speed\n(KITTI ~125k pts)', fontsize=14, fontweight='bold', pad=10)
ax1.set_ylabel('Time (ms)', fontsize=12, fontweight='bold', color='#7f8c8d')
ax1.grid(axis='y', linestyle='--', alpha=0.3)
ax1.spines['top'].set_visible(False)
ax1.spines['right'].set_visible(False)

# Add values
for i, rect in enumerate(bars1):
    height = rect.get_height()
    ax1.text(rect.get_x() + rect.get_width()/2., height + 5,
             f'{height:.1f}', ha='center', va='bottom', fontsize=11, fontweight='bold')
    if i == 1: # nanoPCL P2P
        ax1.text(rect.get_x() + rect.get_width()/2., height + 40,
                 '25x\nFaster', ha='center', va='bottom', fontsize=12, fontweight='bold', color='#e74c3c')

# --- Subplot 2: VoxelGrid Filtering (KITTI 120k) ---
filt_labels = ['PCL (0.1m)', 'nano (0.1m)', 'PCL (0.5m)', 'nano (0.5m)']
filt_times = [33.7, 4.2, 24.6, 2.9] # ms
filt_colors = [color_pcl, color_nano, color_pcl, color_nano]

bars2 = ax2.bar(filt_labels, filt_times, color=filt_colors, width=0.6)
ax2.set_title('Downsampling Speed\n(KITTI ~125k pts)', fontsize=14, fontweight='bold', pad=10)
# ax2.set_ylabel('Time (ms)', fontsize=12, fontweight='bold', color='#7f8c8d')
ax2.grid(axis='y', linestyle='--', alpha=0.3)
ax2.spines['top'].set_visible(False)
ax2.spines['right'].set_visible(False)

# Add values
for i, rect in enumerate(bars2):
    height = rect.get_height()
    ax2.text(rect.get_x() + rect.get_width()/2., height + 1,
             f'{height:.1f}', ha='center', va='bottom', fontsize=11, fontweight='bold')
    if i == 1: # nano 0.1m
        ax2.text(rect.get_x() + rect.get_width()/2., height + 8,
                 '8x', ha='center', va='bottom', fontsize=12, fontweight='bold', color='#e74c3c')

# --- Subplot 3: Build Time (The Killer Feature) ---
build_labels = ['PCL', 'nanoPCL']
build_times = [60, 1.5] # seconds (Conservative 60s for PCL)
build_colors = [color_pcl, color_nano]

bars3 = ax3.bar(build_labels, build_times, color=build_colors, width=0.5)
ax3.set_title('Compile Time\n(minimal example)', fontsize=14, fontweight='bold', pad=10)
ax3.set_ylabel('Time (s)', fontsize=12, fontweight='bold', color='#7f8c8d')
ax3.grid(axis='y', linestyle='--', alpha=0.3)
ax3.spines['top'].set_visible(False)
ax3.spines['right'].set_visible(False)

# Add values
for i, rect in enumerate(bars3):
    height = rect.get_height()
    val = f'{int(height)}s' if height >= 10 else f'{height}s'
    if i == 0: val = '> 60s'
    ax3.text(rect.get_x() + rect.get_width()/2., height + 1,
             val, ha='center', va='bottom', fontsize=11, fontweight='bold')
    if i == 1:
        ax3.text(rect.get_x() + rect.get_width()/2., height + 10,
                 'Instant', ha='center', va='bottom', fontsize=12, fontweight='bold', color='#27ae60')

plt.tight_layout(pad=3.0)
plt.savefig('benchmark_full.png', dpi=300)
print("Full benchmark graph saved to benchmark_full.png")
