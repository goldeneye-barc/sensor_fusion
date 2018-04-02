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

def load_formatted_data():
	with open('formatted_data.pickle', 'rb') as handle: b = pickle.load(handle)
	return b

def get_image(timestamp, data):
	return data[timestamp][2]

def get_hedge(timestamp, data):
	return data[timestamp][0]

def get_lidar(timestamp,data):
	return data[timestamp][1]
	