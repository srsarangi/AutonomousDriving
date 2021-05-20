import numpy as np
f=open("outputfilelinux.txt", "r")
lines=f.readlines()
pts=[]
for x in lines:
	tmp=[]
	mn=x.split()
	tmp.append(-float(mn[2]))
	tmp.append(-float(mn[0]))
	tmp.append(float(mn[1]))
	tmp.append(float(0))
	pts.append(tmp)
np_pts=np.array(pts, dtype='float32')
print(np_pts.shape)
np.save('my_data3.npy', np_pts)
