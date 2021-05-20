from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import random


fig = pyplot.figure()
ax = Axes3D(fig)

#sequence_containing_x_vals = list(range(0, 100))
#sequence_containing_y_vals = list(range(0, 100))
#sequence_containing_z_vals = list(range(0, 100))
Xv=[]
Yv=[]
Zv=[]
#random.shuffle(sequence_containing_x_vals)
#random.shuffle(sequence_containing_y_vals)
#random.shuffle(sequence_containing_z_vals)
f=open("outputfilelinux.txt", "r")
lines=f.readlines()
for x in lines:
	mn=x.split()
	Xv.append(float(mn[0]))
	Yv.append(float(mn[1]))
	Zv.append(float(mn[2]))
ax.set_xlim3d(-20, 20)
ax.set_ylim3d(-20, 20)
ax.set_zlim3d(-20, 20)
ax.scatter(Xv, Yv, Zv)
pyplot.show()
