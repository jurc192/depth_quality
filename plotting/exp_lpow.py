import numpy as np
import seaborn as sb
import matplotlib.pyplot as plt

# EXPERIMENT 8: EXPOSURE VS LASERPOWER HEATMAP

avg20 = np.loadtxt('../experiment8/20/results/avg_exp_lpow.csv')
avg20 = avg20 * 1000    # MICROMETERS

avg50 = np.loadtxt('../experiment8/50/results/avg_exp_lpow.csv')
avg50 = avg50 * 1000    # MICROMETERS

avg80 = np.loadtxt('../experiment8/80/results/avg_exp_lpow.csv')
avg80 = avg80 * 1000    # MICROMETERS

exposures = [8500, 7500, 6500, 5500, 4500, 3500, 2500, 1500]
lpowers   = [150, 180, 210, 240]

# ax = sb.heatmap(avg20, annot=True, fmt=".2f", cmap="Oranges", xticklabels=exposures, yticklabels=lpowers)
# # ax.set(xlabel='Exposures [us]', ylabel='Laser powers [mW]', weight='bold')
# plt.title('Laser power vs Exposure at distance 200 mm')
# plt.xlabel('Exposures [us]', weight='bold')
# plt.ylabel('Laser powers [mW]', weight='bold')
# plt.show()

fig, axs = plt.subplots(1, 3)
sb.heatmap(avg20, ax=axs[0], annot=True, fmt=".2f", cmap="Oranges", cbar=False, xticklabels=exposures, yticklabels=lpowers, square=True)
axs[0].set_xlabel('Exposures [us]', weight='bold')
axs[0].set_ylabel('Laser power [mW]', weight='bold')
axs[0].set_title('RMSE [um] at 200 mm', weight='bold')

sb.heatmap(avg50, ax=axs[1],annot=True, fmt=".2f", cmap="Oranges", cbar=False,xticklabels=exposures, yticklabels=lpowers, square=True)
axs[1].set_xlabel('Exposures [us]', weight='bold')
axs[1].set_ylabel('Laser power [mW]', weight='bold')
axs[1].set_title('RMSE [um] at 500 mm', weight='bold')

sb.heatmap(avg80, ax=axs[2],annot=True, fmt=".2f", cmap="Oranges", cbar=False,xticklabels=exposures, yticklabels=lpowers, square=True)
axs[2].set_xlabel('Exposures [us]', weight='bold')
axs[2].set_ylabel('Laser power [mW]', weight='bold')
axs[2].set_title('RMSE [um] at 800 mm', weight='bold')

# axs[0].set_aspect('equal')
# axs[1].set_aspect('equal')
# axs[2].set_aspect('equal')
plt.savefig('exp_lpow.png')
plt.show()