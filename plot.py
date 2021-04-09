import matplotlib.pyplot as plt
import csv
import numpy as np

rows = []

with open("output.txt") as outfile:
    filereader = csv.reader(outfile, delimiter=',')
    for line in filereader:
        ls = [float(l) for l in line[:-1]]
        rows = rows + [ls]

rows = np.array(rows)
rows = np.transpose(rows)
fig = plt.figure()
for i in range(7):
    norm = np.linalg.norm(rows[i])
    plt.plot(rows[i] / norm)
plt.legend(['Ia','II','Ib','Force','Length','Velocity','Acceleration'])
plt.show()