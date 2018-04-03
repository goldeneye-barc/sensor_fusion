import sys
import cv2
import numpy as np
from itertools import cycle
from collections import defaultdict, OrderedDict
import pickle
import matplotlib.pyplot as plt
import time

def load_formatted_data():
	return np.load('formatted_data_np.npy')

def get_image(index, data):
	return data[index,3]

def get_hedge(index, data):
	return data[index,1]

def get_lidar(index,data):
	return data[index,2]

def get_timestamp(index,data):
	return data[index,0]	