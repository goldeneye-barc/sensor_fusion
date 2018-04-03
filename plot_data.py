from data_utils import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

data = load_formatted_data()

for i in range(data.shape[0]):
	lidar = get_lidar(i,data)
	hedge = get_hedge(i,data)
	plt.cla()
	plt.axis((-4,4,-4,4))
	plt.plot(lidar[0],lidar[1],'.b',hedge[0],hedge[1],'.r',0,0,'k.')
	plt.title('time in ms: ' + str(get_timestamp(i,data)))
	plt.xlabel('x position in meters')
	plt.ylabel('y position in meters')
	plt.legend(('lidar scan','vehicle gps','observer vehicle'))
	plt.pause(0.01)
