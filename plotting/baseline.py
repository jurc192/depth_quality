import numpy as np
import sys
from pathlib import Path
import matplotlib.pyplot as plt


# Check if folder exists
path = Path('../experiment9_base/results/')
if not path.is_dir():
    print("Directory does not exist")
    sys.exit()

# Get all CSVs into a matrix
data = []
for file in path.glob('*.csv'):
    row = np.loadtxt(file)
    data.append(row)
data = np.array(data)

# Calc stuff
means_base = np.mean(data, axis=0)
stddevs_base = np.std(data, axis=0)
print("Base means")
[print(f"{m:.4f}") for m in means_base]
print("Base std devs")
[print(f"{s:.4f}") for s in stddevs_base]

path = Path('../experiment9_best/results/')
data = []
for file in path.glob('*.csv'):
    row = np.loadtxt(file)
    data.append(row)
data = np.array(data)

# Calc stuff
means_best = np.mean(data, axis=0)
stddevs_best = np.std(data, axis=0)
print("Best means")
[print(f"{m:.4f}") for m in means_best]
print("Best std devs")
[print(f"{s:.4f}") for s in stddevs_best]

# # Plot baseline
distances = [200, 300, 400, 500, 600, 700]
# plt.xlabel('distance [mm]', weight='bold')
# plt.ylabel('RMSE [mm]', weight='bold')
# plt.title('Baseline settings accuracy', weight='bold') 
plt.plot(distances, means_base, linestyle='-', linewidth=2.0, marker='o', color='gray')
plt.errorbar(distances, means_base, stddevs_base, linestyle='-', linewidth=1.0, marker='o', color='gray', fmt='none', uplims=True, lolims=True) 
# # plt.errorbar(distances, means_best, stddevs_best, linestyle='-', marker='^')
# plt.grid(color='lightgray')
# plt.savefig('baseline.png')
# plt.show()


# Plot best
# distances = [200, 300, 400, 500, 600, 700]
plt.xlabel('distance [mm]', weight='bold')
plt.ylabel('RMSE [mm]', weight='bold')
plt.title('Optimal settings accuracy', weight='bold') 
plt.plot(distances, means_best, linestyle='-', linewidth=2.0, marker='o', color='b')
plt.errorbar(distances, means_best, stddevs_best, linestyle='-', linewidth=1.0, marker='o', color='r', fmt='none', uplims=True, lolims=True) 
# plt.errorbar(distances, means_best, stddevs_best, linestyle='-', marker='^')
plt.grid(color='lightgray')
plt.savefig('best.png')
plt.show()
