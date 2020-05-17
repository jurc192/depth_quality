import csv
import os
import sys
import matplotlib.pyplot as plt

if len(sys.argv) < 2 or not os.path.isfile(sys.argv[1]):
    print("Usage: ./plotter.py <filename.csv>")
    print("Is the filename correct?")
    exit(1)

distance = []
planefitRMS = []

with open(sys.argv[1]) as csvfile:
    csvreader = csv.reader(csvfile, delimiter='\t')
    for row in csvreader:
        distance.append(row[0])
        planefitRMS.append(row[2])

plt.plot(distance, planefitRMS)
plt.xlabel('Distance (cm)')
plt.ylabel('Plane-fit RMS (%)')
plt.show()

