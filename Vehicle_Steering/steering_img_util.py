
from __future__ import absolute_import
from __future__ import print_function
import os
import cv2
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import glob

def augment_brightness_camera_images(image):

    ### Augment brightness
    image1 = cv2.cvtColor(image,cv2.COLOR_RGB2HSV)
    random_bright = .25+np.random.uniform()
    #print(random_bright)
    image1[:,:,2] = image1[:,:,2]*random_bright
    image1 = cv2.cvtColor(image1,cv2.COLOR_HSV2RGB)
    return image1

def trans_image(image,bb_boxes_f,trans_range):
    # Translation augmentation
    bb_boxes_f = bb_boxes_f.copy(deep=True)

    tr_x = trans_range*np.random.uniform()-trans_range/2
    tr_y = trans_range*np.random.uniform()-trans_range/2

    Trans_M = np.float32([[1,0,tr_x],[0,1,tr_y]])
    rows,cols,channels = image.shape
    bb_boxes_f['xmin'] = bb_boxes_f['xmin'] + tr_x
    bb_boxes_f['xmax'] = bb_boxes_f['xmax'] + tr_x
    bb_boxes_f['ymin'] = bb_boxes_f['ymin'] + tr_y
    bb_boxes_f['ymax'] = bb_boxes_f['ymax'] + tr_y

    image_tr = cv2.warpAffine(image,Trans_M,(cols,rows))

    return image_tr,bb_boxes_f

def stretch_image(img,bb_boxes_f,scale_range):
    # Stretching augmentation

    bb_boxes_f = bb_boxes_f.copy(deep=True)

    tr_x1 = scale_range*np.random.uniform()
    tr_y1 = scale_range*np.random.uniform()
    p1 = (tr_x1,tr_y1)
    tr_x2 = scale_range*np.random.uniform()
    tr_y2 = scale_range*np.random.uniform()
    p2 = (img.shape[1]-tr_x2,tr_y1)

    p3 = (img.shape[1]-tr_x2,img.shape[0]-tr_y2)
    p4 = (tr_x1,img.shape[0]-tr_y2)

    pts1 = np.float32([[p1[0],p1[1]],
                   [p2[0],p2[1]],
                   [p3[0],p3[1]],
                   [p4[0],p4[1]]])
    pts2 = np.float32([[0,0],
                   [img.shape[1],0],
                   [img.shape[1],img.shape[0]],
                   [0,img.shape[0]] ]
                   )

    M = cv2.getPerspectiveTransform(pts1,pts2)
    img = cv2.warpPerspective(img,M,(img.shape[1],img.shape[0]))
    img = np.array(img,dtype=np.uint8)

    bb_boxes_f['xmin'] = (bb_boxes_f['xmin'] - p1[0]) / (p2[0]-p1[0]) * img.shape[1]
    bb_boxes_f['xmax'] = (bb_boxes_f['xmax'] - p1[0]) / (p2[0]-p1[0]) * img.shape[1]
    bb_boxes_f['ymin'] = (bb_boxes_f['ymin'] - p1[1]) / (p3[1]-p1[1]) * img.shape[0]
    bb_boxes_f['ymax'] = (bb_boxes_f['ymax'] - p1[1]) / (p3[1]-p1[1]) * img.shape[0]

    return img,bb_boxes_f

def get_image_name(df,ind,size=(640,480),augmentation=False, trans_range=20, scale_range=20):
    ### Get image by name
    
    file_name = df['filename'][ind]
    img = cv2.imread(file_name, 1)
    img_size = np.shape(img)

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img,size)
    name_str = file_name.split('/')
    name_str = name_str[-1]

    img_size_post = np.shape(img)

    if augmentation == True:
        img,bb_boxes = trans_image(img,bb_boxes,trans_range)
        img,bb_boxes = stretch_image(img,bb_boxes,scale_range)
        img = augment_brightness_camera_images(img)

    return name_str, img
