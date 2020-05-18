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
means = np.mean(data, axis=0)
stddevs = np.std(data, axis=0)
print(means)
print(stddevs)

# Plot
distances = [200, 300, 400, 500, 600, 700]
plt.errorbar(distances, means, stddevs, linestyle='-', marker='^')
plt.show()