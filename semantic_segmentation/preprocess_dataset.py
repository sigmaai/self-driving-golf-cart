import cv2
import pickle as cPickle
import os
import numpy as np
import tensorflow as tf
from collections import namedtuple
import random
import csv

data_dir = "/Volumes/Personal_Drive/Datasets/CityScapes"

new_img_height = 512 # (the height all images fed to the model will be resized to)
new_img_width = 1024 # (the width all images fed to the model will be resized to)
no_of_classes = 20 # (number of object classes (road, sidewalk, car etc.))

train_imgs_dir = data_dir + "/leftImg8bit/val/"
train_gt_dir = data_dir + "/gtCoarse/train_extra/"

# train_dirs = ["jena/", "zurich/", "weimar/", "ulm/", "tubingen/", "stuttgart/",
#             "strasbourg/", "monchengladbach/", "krefeld/", "hanover/",
#             "hamburg/", "erfurt/", "dusseldorf/", "darmstadt/", "cologne/",
#             "bremen/", "bochum/", "aachen/"]

# train_extra = ['augsburg', 'bad-honnef', 'bamberg', 'bayreuth', 'dortmund', 'dresden',
#                'duisburg', 'erlangen', 'freiburg', 'heidelberg', 'heilbronn', 'karlsruhe',
#                'konigswinter', 'konstanz', 'mannheim', 'muhlheim-ruhr', 'nuremberg', 'oberhausen',
#                'saarbrucken', 'schweinfurt', 'troisdorf', 'wuppertal', 'wurzburg']

val_dirs = ["frankfurt/", "munster/", "lindau/"]

# get the path to all training images and their corresponding label image:
train_paths = []

for dir_step, dir in enumerate(val_dirs):

    img_dir = train_imgs_dir + dir

    file_names = os.listdir(img_dir)
    for step, file_name in enumerate(file_names):
        if step % 10 == 0:
            print (("train dir %d/%d, step %d/%d" % (dir_step, len(val_dirs)-1,
                        step, len(file_names)-1)))

        row = []
        gt_img = file_name.replace("_leftImg8bit", "_gtCoarse_color")
        row.append("/leftImg8bit/val/" + dir + file_name)
        row.append("/gtCoarse/train_extra/" + dir + "/" + gt_img)
        train_paths.append(row)


print(len(train_paths))
print(train_paths[90])

csvfile = "./val_labels.csv"

#Assuming res is a flat list
with open(csvfile, "w") as output:
    writer = csv.writer(output, lineterminator='\n')
    for val in train_paths:
        writer.writerow(val)