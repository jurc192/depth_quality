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
print(means_base)
print(stddevs_base)

path = Path('../experiment9_best/results/')
data = []
for file in path.glob('*.csv'):
    row = np.loadtxt(file)
    data.append(row)
data = np.array(data)

# Calc stuff
means_best = np.mean(data, axis=0)
stddevs_best = np.std(data, axis=0)
print(means_best)
print(stddevs_best)

# Plot
distances = [200, 300, 400, 500, 600, 700]
plt.errorbar(distances, means_base, stddevs_base, linestyle='-', marker='^')
plt.errorbar(distances, means_best, stddevs_best, linestyle='-', marker='^')
plt.show()