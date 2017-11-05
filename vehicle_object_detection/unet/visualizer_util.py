import numpy as np
import os, sys
import cv2
import matplotlib.pyplot as plt
from scipy.ndimage.measurements import label

#### Function for drawing bounding boxes, taken from Ryan's code on Udacity
img_rows = 640
img_cols = 960

def draw_labeled_bboxes(img, labels):
    # Iterate through all detected cars
    for car_number in range(1, labels[1]+1):
        # Find pixels with each car_number label value
        nonzero = (labels[0] == car_number).nonzero()
        # Identify x and y values of those pixels
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Define a bounding box based on min/max x and y
        if ((np.max(nonzeroy)-np.min(nonzeroy)>50) & (np.max(nonzerox)-np.min(nonzerox)>50)):
            bbox = ((np.min(nonzerox), np.min(nonzeroy)), (np.max(nonzerox), np.max(nonzeroy)))
            # Draw the box on the image
            cv2.rectangle(img, bbox[0], bbox[1], (0,0,255),6)
    # Return the image
    return img

def test_new_img(img, model):
    img = cv2.resize(img,(img_cols, img_rows))
    img = np.array([img])
    pred = model.predict(img)
    return pred,img[0]

def run_predictions(img, model):
    # Take in RGB image
    seg_img ,img = test_new_img(img=img, model=model)
    img  = np.array(img,dtype= np.uint8)
    img_pred = np.array(255*seg_img[0],dtype=np.uint8)
    heatmap = img_pred[:,:,0]
    labels = label(heatmap)
    bb_img = draw_labeled_bboxes(np.copy(img), labels)

    # seg_img = np.array(255 * seg_img, dtype=np.uint8)
    # rgb_mask_pred = cv2.cvtColor(seg_img, cv2.COLOR_GRAY2RGB)
    # img_pred = cv2.addWeighted(rgb_mask_pred, 0.5, img, 0.5, 0)

    return bb_img, seg_img

### Test on new image