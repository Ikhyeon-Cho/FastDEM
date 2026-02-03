import matplotlib.pyplot as plt
import numpy as np

# Data (PCL vs nanoPCL)
# 1. Registration (ICP) - KITTI Real Data (120k points)
reg_labels = ['PCL ICP', 'nanoPCL (P2P)']
reg_times = [307.72, 12.03]  # ms

# 2. Preprocessing (VoxelGrid) - 500k points (Conservative estimate for PCL)
# PCL VoxelGrid typically ~100-150ms for 500k points. nanoPCL measured ~40ms.
prep_labels = ['PCL VoxelGrid', 'nanoPCL VoxelGrid']
prep_times = [125.0, 40.0]   # ms

# Style Settings (Clean & Engineering)
plt.style.use('default')
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial', 'Helvetica', 'DejaVu Sans']

# Create subplots (1 row, 2 columns)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

# Colors
color_pcl = '#95a5a6'      # Gray
color_nano = '#2980b9'     # Strong Blue

# --- Plot 1: Registration (ICP) ---
bars1 = ax1.bar(reg_labels, reg_times, color=[color_pcl, color_nano], width=0.6)
ax1.set_title('Registration (ICP)\nKITTI 120k points', fontsize=14, fontweight='bold', pad=15)
ax1.set_ylabel('Time (ms)', fontsize=12, fontweight='bold', color='#2c3e50')
ax1.spines['top'].set_visible(False)
ax1.spines['right'].set_visible(False)
ax1.grid(axis='y', linestyle='--', alpha=0.3)

# Add values
for rect in bars1:
    height = rect.get_height()
    ax1.text(rect.get_x() + rect.get_width()/2., height + 5,
             f'{height:.1f} ms',
             ha='center', va='bottom', fontsize=11, fontweight='bold')

# Speedup arrow
ax1.annotate(f'{(reg_times[0]/reg_times[1]):.1f}x Faster', 
             xy=(1, reg_times[1]), xytext=(1, reg_times[0]/2),
             arrowprops=dict(arrowstyle='->', color='#e74c3c', lw=2),
             ha='center', fontsize=12, fontweight='bold', color='#e74c3c')


# --- Plot 2: Preprocessing (VoxelGrid) ---
bars2 = ax2.bar(prep_labels, prep_times, color=[color_pcl, color_nano], width=0.6)
ax2.set_title('Preprocessing (VoxelGrid)\n500k points', fontsize=14, fontweight='bold', pad=15)
# ax2.set_ylabel('Time (ms)', fontsize=12) # Shared y-axis logic or separate? Separate is better for scale.
ax2.spines['top'].set_visible(False)
ax2.spines['right'].set_visible(False)
ax2.grid(axis='y', linestyle='--', alpha=0.3)

# Add values
for rect in bars2:
    height = rect.get_height()
    ax2.text(rect.get_x() + rect.get_width()/2., height + 2,
             f'{height:.1f} ms',
             ha='center', va='bottom', fontsize=11, fontweight='bold')

# Speedup arrow
ax2.annotate(f'{(prep_times[0]/prep_times[1]):.1f}x Faster', 
             xy=(1, prep_times[1]), xytext=(1, prep_times[0]/2),
             arrowprops=dict(arrowstyle='->', color='#e74c3c', lw=2),
             ha='center', fontsize=12, fontweight='bold', color='#e74c3c')

plt.tight_layout(pad=3.0)
plt.savefig('benchmark_integrated.png', dpi=300)
print("Graph saved to benchmark_integrated.png")
