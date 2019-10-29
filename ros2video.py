#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Oct 29 11:55:41 2019

@author: pan
"""


from rosbag import Bag
from argparse import ArgumentParser
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

parser = ArgumentParser()
parser.add_argument('file', type=str)
parser.add_argument('--topic','-t', type=str, default='/axis_front/image_raw/compressed')
parser.add_argument('--output', '-o', type=str, default='output')
parser.add_argument('--fps', type=int, default=30)
parser.add_argument('--show','-s', action='store_true')
args = parser.parse_args()

file = args.file
topic = args.topic
output = args.output
fps = args.fps
show = args.show

isInitialized = False

print('output video file: '+output+'.avi')

try:
    print('loading bag file...')
    bag = Bag(file)
    print('loaded!')
except:
    print('loading bag file failed, check if file is broken')
    quit()

vwriter = cv.VideoWriter()
bridge = CvBridge()
for topic, msg, t in bag.read_messages(topic):
    npmsg = bridge.compressed_imgmsg_to_cv2(msg)
    if show:
        cv.imshow('img', npmsg)
        if cv.waitKey(20) == ord('q'):
            show = False
            cv.destroyWindow('img')
    if not isInitialized:
        size = (npmsg.shape[1], npmsg.shape[0])
        vwriter.open(output+'.avi', cv.VideoWriter_fourcc(*'MJPG'), fps, size)
        vwriter.write(npmsg)
        isInitialized = True
    else:
        vwriter.write(npmsg)

vwriter.release()
bag.close()
print('Done!')
