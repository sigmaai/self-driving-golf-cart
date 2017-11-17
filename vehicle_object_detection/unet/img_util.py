
from __future__ import absolute_import
from __future__ import print_function
import os
import cv2
import matplotlib.pyplot as plt
import numpy as np


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

def get_image_name(df,ind,size=(640,300),augmentation=False, trans_range=20, scale_range=20):
    ### Get image by name
    
    file_name = df['File_Path'][ind]
    img = cv2.imread(file_name, 1)
    img_size = np.shape(img)

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img,size)
    name_str = file_name.split('/')
    name_str = name_str[-1]
    bb_boxes = df[df['Frame'] == name_str].reset_index()
    img_size_post = np.shape(img)

    if augmentation == True:
        img,bb_boxes = trans_image(img,bb_boxes,trans_range)
        img,bb_boxes = stretch_image(img,bb_boxes,scale_range)
        img = augment_brightness_camera_images(img)

    bb_boxes['xmin'] = np.round((bb_boxes['xmin'] / img_size[1] * img_size_post[1]).astype(np.float32))
    bb_boxes['xmax'] = np.round((bb_boxes['xmax'] / img_size[1] * img_size_post[1]).astype(np.float32))
    bb_boxes['ymin'] = np.round((bb_boxes['ymin'] / img_size[0] * img_size_post[0]).astype(np.float32))
    bb_boxes['ymax'] = np.round((bb_boxes['ymax'] / img_size[0] * img_size_post[0]).astype(np.float32))
    bb_boxes['Area'] = (bb_boxes['xmax'] - bb_boxes['xmin'])*(bb_boxes['ymax'] - bb_boxes['ymin'])
    #bb_boxes = bb_boxes[bb_boxes['Area']>400]

    return name_str, img, bb_boxes


def get_image_path(path,size=(640,300)):

    img = cv2.imread(path, 1)
    img = cv2.resize(img, size)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    return img


def get_mask_seg(img,bb_boxes_f):

    # Get mask
    img_mask = np.zeros_like(img[:, :, 0])
    for i in range(len(bb_boxes_f)):
        #plot_bbox(bb_boxes,i,'g')
        bb_box_i = [bb_boxes_f.iloc[i]['xmin'],
                    bb_boxes_f.iloc[i]['ymin'],
                    bb_boxes_f.iloc[i]['xmax'],
                    bb_boxes_f.iloc[i]['ymax']]

        img_mask[int(bb_box_i[1]): int(bb_box_i[3]), int(bb_box_i[0]): int(bb_box_i[2])] = 1.
        img_mask = np.reshape(img_mask, (np.shape(img_mask)[0], np.shape(img_mask)[1],1))
    return img_mask


def plot_image_mask(im,im_mask):
    ### Function to plot image mask

    im = np.array(im,dtype=np.uint8)
    im_mask = np.array(im_mask,dtype=np.uint8)
    plt.subplot(1,3,1)
    plt.imshow(im)
    plt.axis('off')
    plt.subplot(1,3,2)
    plt.imshow(im_mask[:,:,0])
    plt.axis('off')
    plt.subplot(1,3,3)
    plt.imshow(cv2.bitwise_and(im,im,mask=im_mask))
    plt.axis('off')
    plt.show()


def plot_bbox(bb_boxes,ind_bb,color='r',linewidth=2):
    ### Plot bounding box

    bb_box_i = [bb_boxes.iloc[ind_bb]['xmin'],
                bb_boxes.iloc[ind_bb]['ymin'],
                bb_boxes.iloc[ind_bb]['xmax'],
                bb_boxes.iloc[ind_bb]['ymax']]
    plt.plot([bb_box_i[0],bb_box_i[2],bb_box_i[2],
                  bb_box_i[0],bb_box_i[0]],
             [bb_box_i[1],bb_box_i[1],bb_box_i[3],
                  bb_box_i[3],bb_box_i[1]],
             color,linewidth=linewidth)


def plot_image_bbox(im,bb_boxes):
    ### Plot image and bounding box
    plt.imshow(im)
    for i in range(len(bb_boxes)):
        plot_bbox(bb_boxes,i,'g')

        bb_box_i = [bb_boxes.iloc[i]['xmin'],bb_boxes.iloc[i]['ymin'],
                bb_boxes.iloc[i]['xmax'],bb_boxes.iloc[i]['ymax']]
        plt.plot(bb_box_i[0],bb_box_i[1],'rs')
        plt.plot(bb_box_i[2],bb_box_i[3],'bs')
    plt.axis('off')
