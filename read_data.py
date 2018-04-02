import rosbag
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
from itertools import cycle
from collections import defaultdict, OrderedDict
import pickle
from extractor import load_data
import matplotlib.pyplot as plt
import time


# ---------------------------------------------------
# ----------HELPER FUNCTIONS-------------------------
# ---------------------------------------------------
def find_heading(data, plot):
	xh = []
	yh = []
	for k in range(40):
		hedge = data[k,1][0]
		if hedge.flags == 2:
			xh.append(hedge.x_m)
			yh.append(hedge.y_m)
	xh = np.array(xh)
	yh = np.array(yh)
	A = np.vstack((xh,np.ones(xh.shape)))
	A = np.transpose(A)

	bm = np.transpose(yh)
	m = np.linalg.lstsq(A,bm)

	a = m[0][0]
	b = m[0][1]
	
	x_avg = np.average(xh)
	y_avg = np.average(yh)
	
	if plot == True:
		x = np.linspace(3,4,5)
		y = a*x + b
		plt.plot(xh,yh,'b',x,y,'r',x_avg,y_avg,'bo')
		plt.title('determining the heading')
		plt.show()
	return (a, b, x_avg, y_avg)

def reformat(data, metadata, a, b, x0, y0, plot):
	theta = np.arctan(a)
	rot = np.array([[np.cos(-theta),-np.sin(-theta)],[np.sin(-theta),np.cos(-theta)]])	
	xt,yt = 0,0
	formatted_data = {}
	for j in range(30,len(data[:,0]),1):
		lidar = data[j,1][2]
		hpos = data[j,1][0]

		if hpos.flags != 2:
			hpos = np.array([hpos.x_m - x0+0.3,hpos.y_m - y0 + 0.1])
			hpos = np.dot(rot,hpos)
			xt = hpos[0]
			yt = hpos[1]

		x = []
		y = []
		angle = metadata['/scan']['angle_min']
		increment = metadata['/scan']['angle_increment']
		for i in lidar:
			x.append(i*np.cos(angle))
			y.append(i*np.sin(angle))
			angle += increment

		formatted_data[data[j,0]] = [(xt,yt), (x,y), data[j,1][1]]

		if plot == True:
			plt.cla()
			
			plt.plot(x,y,'.',xt,yt,'ro',0,0,'ko')
			plt.axis((-4,4,-4,4))
			plt.title('time = ' + str(data[j,0]) + ' (ms)')
			plt.legend(('Lidar Scan', 'vehicle position', 'observer vehicle'))
			plt.pause(0.1)

	return formatted_data
			
		
# --------------------------------------------------------------------
# -----------------------DATA FORMATTING------------------------------
# --------------------------------------------------------------------	

data, metadata = load_data('','2016-02-18-14-10-44.bag')
a,b,x0,y0 = find_heading(data, False)
formatted_data = reformat(data, metadata, a,b,x0,y0, False)

#  **********DATA IS STORED IN DICTIONARY********
#  Key = Timestamp, Value = LIST
# Value[0] = tuple ground truth relative x,y coordinate of target car
# Value[1] = tuple with np arrays containing x and y coordinates of lidar data
# Value[2] = np array with camara image (rgb8)
# *************************************************

# save the data:
with open('formatted_data.pickle', 'wb') as handle: pickle.dump(formatted_data,handle,protocol=pickle.HIGHEST_PROTOCOL)



# Use these commands to open the file!!!
# $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

#with open('formatted_data.pickle', 'rb') as handle: b = pickle.load(handle)

#for key in b: 
#	print(type(b))
#	print(type(b[key][0]))
#	print(type(b[key][1]))
#	print(type(b[key][2]))
#	print(key)
#	break


	


