import cv2
import os
import numpy as np
import utils
import csv


# create a function mapping id to trainId:
id_to_trainId = {label.id: label.trainId for label in utils.labels}
id_to_trainId_map_func = np.vectorize(id_to_trainId.get)

no_of_classes = len(utils.labels) # (number of object classes (road, sidewalk, car etc.))

cityscapes_dir = utils.data_path + "cityscapes/"

train_imgs_dir = cityscapes_dir + "leftImg8bit/train/"
train_gt_dir = cityscapes_dir + "gtFine/train/"

val_imgs_dir = cityscapes_dir + "leftImg8bit/val/"
val_gt_dir = cityscapes_dir + "gtFine/val/"

train_dirs = ["jena/", "zurich/", "weimar/", "ulm/", "tubingen/", "stuttgart/",
            "strasbourg/", "monchengladbach/", "krefeld/", "hanover/",
            "hamburg/", "erfurt/", "dusseldorf/", "darmstadt/", "cologne/",
            "bremen/", "bochum/", "aachen/"]
val_dirs = ["frankfurt/", "munster/", "lindau/"]


# get the path to all training images and their corresponding label image:
train_img_paths = []
train_trainId_label_paths = []
for dir_step, dir in enumerate(train_dirs):
    img_dir = train_imgs_dir + dir

    file_names = os.listdir(img_dir)
    for step, file_name in enumerate(file_names):
        if step % 10 == 0:
            print ("train dir %d/%d, step %d/%d" % (dir_step, len(train_dirs)-1,
                        step, len(file_names)-1))



