#!/usr/bin/env python
# Imports
import tensorflow as tf
import cv2
import sys
import os
import time
import math
import numpy as np
import collections

# Set environment variables for TensorFlow threading
def set_tf_config(ncpu):
    os.environ["OMP_NUM_THREADS"] = str(ncpu)
    os.environ["TF_NUM_INTRAOP_THREADS"] = str(ncpu)
    os.environ["TF_NUM_INTEROP_THREADS"] = str(ncpu)
    tf.config.threading.set_inter_op_parallelism_threads(ncpu)
    tf.config.threading.set_intra_op_parallelism_threads(ncpu)
    tf.config.set_soft_device_placement(True)

# Radian <-> Degree conversion functions
def deg2rad(deg):
    return deg * math.pi / 180.0

def rad2deg(rad):
    return 180.0 * rad / math.pi

# Get the number of cores to be used by TensorFlow
if len(sys.argv) > 1:
    NCPU = int(sys.argv[1])
else:
    NCPU = 1

batch_size = max(16, NCPU*4)  # Larger batch size to better utilize multiple CPUs
print(f"Using batch size: {batch_size}")

# Set up TensorFlow configuration
physical_devices = tf.config.list_physical_devices('CPU')
if len(physical_devices) > 0:
    tf.config.experimental.set_visible_devices(physical_devices[0], 'CPU')
    print(f"Using CPU device: {physical_devices[0]}")

print(f"Trying to use {NCPU} CPUs")
set_tf_config(NCPU)

# Import the model
from model import create_model

# Load the model
model = create_model(input_shape=(66, 200, 3))
model.load_weights("model/model.h5")

#    and the number of frames already processed
NFRAMES = 1000
curFrame = 0

# Periodic task options
period = 50
is_periodic = True
# Create lists for tracking operation timings
cap_time_list = []
prep_time_list = []
pred_time_list = []
tot_time_list = []

print('---------- Processing video for epoch 1 ----------')

# Open the video file
vid_path = 'epoch-1.avi'
assert os.path.isfile(vid_path)
cap = cv2.VideoCapture(vid_path)

# Process the video while recording the operation execution times
print('Performing inference...')
time_start = time.time()
first_frame = True

while curFrame < NFRAMES:
    batch_frames = []
    batch_times = []
    
    # Collect frames for a batch
    for _ in range(batch_size):
        if curFrame >= NFRAMES:
            break
            
        cam_start = time.time()
        ret, img = cap.read()
        if not ret:
            break
            
        prep_start = time.time()
        
        # Preprocess the input frame
        img = cv2.resize(img, (200, 66))
        img = img / 255.0
        
        batch_frames.append(img)
        batch_times.append((cam_start, prep_start))
        
    if not batch_frames:
        break
        
    # Convert list to numpy array for batch prediction
    batch_input = np.array(batch_frames)
    
    # Perform batch prediction
    pred_start = time.time()
    predictions = model.predict(batch_input, verbose=1)
    pred_end = time.time()
    
    # Process prediction results
    for i, prediction in enumerate(predictions):
        if i == 0 and first_frame:
            first_frame = False
            continue
            
        rad = prediction[0]
        deg = rad2deg(rad)
        
        cam_start, prep_start = batch_times[i]
        
        # Calculate the timings for each step
        cam_time = (prep_start - cam_start) * 1000
        prep_time = (pred_start - prep_start) * 1000
        
        # Distribute prediction time proportionally
        pred_time_per_frame = (pred_end - pred_start) * 1000 / len(batch_frames)
        pred_time = pred_time_per_frame
        
        # Total time includes capture, preprocessing, and a portion of prediction
        tot_time = cam_time + prep_time + pred_time
        
        print(f'pred: {deg:0.2f} deg. took: {tot_time:0.2f} ms | cam={cam_time:0.2f} prep={prep_time:0.2f} pred={pred_time:0.2f}')
        
        # Add timings to lists
        if not (i == 0 and first_frame):
            tot_time_list.append(tot_time)
            curFrame += 1
            
        # Wait for next period (only for the last frame in batch)
        if i == len(predictions) - 1 and is_periodic:
            wait_time = (period - tot_time) / 1000
            if wait_time > 0:
                time.sleep(wait_time)

cap.release()

# Calculate and output FPS/frequency
fps = curFrame / (time.time() - time_start)
print('completed inference, total frames: {}, average fps: {} Hz'.format(curFrame, round(fps, 1)))

# Calculate and display statistics of the total inferencing times
print("count: {}".format(len(tot_time_list)))
print("mean: {}".format(np.mean(tot_time_list)))
print("99.999pct: {}".format(np.percentile(tot_time_list, 99.999)))
print("99.99pct: {}".format(np.percentile(tot_time_list, 99.99)))
print("99.9pct: {}".format(np.percentile(tot_time_list, 99.9)))
print("99pct: {}".format(np.percentile(tot_time_list, 99)))
