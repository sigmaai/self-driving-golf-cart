
"""
manage the dataset I made in ROS
delete the dropped frames.

"""

import numpy as np
import pandas as pd
import csv
import os

ds_dir = ""
ds_path = ""
df_truth = pd.read_csv(ds_path, usecols=['frame_id', 'steering_angle'], index_col=None)
real_path = []

for i in range(df_truth.size()):

    img_path = ds_dir + df_truth['frame_id'].loc[i]
    if os.path.isfile(img_path):
        real_path.append(df_truth.values[i])