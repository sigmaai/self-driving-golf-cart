import numpy as np
import img_util
import os
import cv2

img_rows = 640
img_cols = 960

# Training generator, generates augmented images
def generate_train_batch(data,batch_size = 32):

    batch_images = np.zeros((batch_size, img_rows, img_cols, 3))
    batch_masks = np.zeros((batch_size, img_rows, img_cols, 1))
    while 1:
        for i_batch in range(batch_size):
            i_line = np.random.randint(len(data)-2000)
            name_str,img,bb_boxes = img_util.get_image_name(data,i_line,
                                                   size=(img_cols, img_rows),
                                                  augmentation=True,
                                                   trans_range=50,
                                                   scale_range=50
                                                  )
            img_mask = img_util.get_mask_seg(img,bb_boxes)
            batch_images[i_batch] = img
            batch_masks[i_batch] =img_mask
        yield batch_images, batch_masks


# Testing generator, generates augmented images
def generate_test_batch(data,batch_size = 32):

    batch_images = np.zeros((batch_size, img_rows, img_cols, 3))
    batch_masks = np.zeros((batch_size, img_rows, img_cols, 1))
    while 1:
        for i_batch in range(batch_size):
            i_line = np.random.randint(2000)
            i_line = i_line+len(data)-2000
            name_str,img,bb_boxes = img_util.get_image_name(data,i_line,
                                                   size=(img_cols, img_rows),
                                                  augmentation=False,
                                                   trans_range=0,
                                                   scale_range=0
                                                  )
            img_mask = img.util.get_mask_seg(img,bb_boxes)
            batch_images[i_batch] = img
            batch_masks[i_batch] = img_mask
        yield batch_images, batch_masks