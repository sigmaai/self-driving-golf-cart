# ----------------------------------------------
# utils.py | lane detection | Created by Neil Nie
# & Michael Meng utilities for lane detection
# (c) Neil Nie 2017, Please refer to the license
# ----------------------------------------------

import cv2
import numpy as np
import glob
from detection.lane import configs


def get_test_images():
    test1 = cv2.imread('./test_images/test1.jpg')
    test2 = cv2.imread('./test_images/test2.jpg')
    test3 = cv2.imread('./test_images/test3.jpg')
    test4 = cv2.imread('./test_images/test4.jpg')
    test5 = cv2.imread('./test_images/test5.jpg')
    test6 = cv2.imread('./test_images/test6.jpg')
    test = [test1, test2, test3, test4, test5, test6]
    return test


def sobelx(img, ker=3, xmin=15, xmax=50):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Convolute the image with Sobel kernel in the horizontal direction
    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=ker)  # Convolve
    abs_sobelx = np.absolute(sobelx)  # Take absolute value to accentuate lines away from horizontal
    scaled_sobelx = np.uint8(255 * abs_sobelx / np.max(abs_sobelx))  # Convert image to 8 bits for various image formats

    # Threshold the absolute value
    sxbinary = np.zeros_like(scaled_sobelx)
    sxbinary[(scaled_sobelx >= xmin) & (scaled_sobelx <= xmax)] = 1

    return sxbinary


def shift(left_fit,right_fit,y,width):
    left_point = left_fit[0]*(y*configs.ym_per_pix)**2 + left_fit[1]*(y*configs.ym_per_pix) + left_fit[2]
    right_point = right_fit[0]*(y*configs.ym_per_pix)**2 + right_fit[1]*(y*configs.ym_per_pix) + right_fit[2]
    mid = (left_point + right_point)/2
    image_mid = width*configs.xm_per_pix
    return mid - image_mid/2


def curvature(left_fit,right_fit,y):
    left_curvature = ((1 + (2*left_fit[0]*y*configs.ym_per_pix + left_fit[1])**2)**1.5) / np.absolute(2*left_fit[0])
    right_curvature = ((1 + (2*right_fit[0]*y*configs.ym_per_pix + right_fit[1])**2)**1.5) / np.absolute(2*right_fit[0])
    avg = (left_curvature + right_curvature)/2
    return int(avg)


def fit_rest(img,prev_fit,swidth=100):
    # find all white in img
    white_y_arr,white_x_arr = img.nonzero()
    # bound white searching within search width of prev-fit
    ind= (white_x_arr > (prev_fit[0]*((white_y_arr)**2) + prev_fit[1]*(white_y_arr) + prev_fit[2] - swidth)) & \
        (white_x_arr < (prev_fit[0]*((white_y_arr)**2) + prev_fit[1]*(white_y_arr) + prev_fit[2] + swidth))
    fit = np.polyfit(white_y_arr[ind], white_x_arr[ind],2)
    fit_m = np.polyfit(white_y_arr[ind]*configs.ym_per_pix, white_x_arr[ind]*configs.xm_per_pix, 2)
    return fit,fit_m


def histogram(img):
    return np.sum(img[img.shape[0]//2:,:], axis=0)


def base(img):

    hist = histogram(img)
    left_oct = np.int(hist.shape[0]/8)
    midpoint = np.int(hist.shape[0]/2)
    right_oct = np.int(7*hist.shape[0]/8)
    left_base = np.argmax(hist[left_oct:midpoint])+left_oct
    right_base = np.argmax(hist[midpoint:right_oct]) + midpoint

    return left_base,right_base


def fit_first(img, base, winwidth=100, minpix=50, step=18):

    imgheight = img.shape[0]
    winheight = int(imgheight / step)

    # current x is the middle of the current window in x direction
    # current y is the bottom of the current window in y direction
    current_x = base
    current_x_arr = [current_x]
    current_y = imgheight
    current_y_arr = [current_y]

    white_pos_x_arr = []
    white_pos_y_arr = []

    # shift window upward for each step
    for i in range(step):
        # find current left and right window
        window = img[(current_y - winheight):current_y, int(current_x - winwidth / 2):int(current_x + winwidth / 2)]
        # find white pixels
        white_pos_y, white_pos_x = window.nonzero()
        # adjust to find their coordinate in the original image
        shift_x = int(current_x - winwidth / 2)
        shift_y = current_y - winheight
        white_pos_x += shift_x
        white_pos_y += shift_y
        # update current_x
        if len(white_pos_x) > minpix:
            current_x = int(np.mean(white_pos_x, axis=0))
        # update current_y(shift window upwards)
        current_y -= winheight
        # append
        white_pos_x_arr.append(white_pos_x)
        white_pos_y_arr.append(white_pos_y)
        current_x_arr.append(current_x)
        current_y_arr.append(current_y)
    # fit poly
    white_pos_x = np.concatenate(white_pos_x_arr)
    white_pos_y = np.concatenate(white_pos_y_arr)
    fit = np.polyfit(white_pos_y, white_pos_x, 2)
    fit_m = np.polyfit(white_pos_y * configs.ym_per_pix, white_pos_x * configs.xm_per_pix, 2)

    return fit, fit_m


def threshold(img, smin=160, smax=255):
    # convert the image from BGR to HLS and take the saturation and lightness channel
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    sat = hls[:, :, 2]
    hue = hls[:, :, 0]
    lig = hls[:, :, 1]
    # create a black and white image, white at points whose saturation is within the threshold
    binary = np.zeros_like(sat)
    # !!! since the larger the lightness, the smaller the saturation, here we compensate that by dividing the lightness from the threshold
    lower = np.minimum(smin / (0.5 + lig / 255), np.full(lig.shape, smax))
    higher = np.full(lig.shape, smax)
    binary[(sat > lower) & (sat < higher)] = 1

    return binary


def edge(img):
    # apply
    sbinary = threshold(img)
    sxbinary = sobelx(img)
    # combine
    binary = np.zeros_like(sbinary)
    binary[(sbinary == 1) | (sxbinary == 1)] = 1

    return binary


def undistort(img, objpoints, imgpoints):

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).shape[::-1], None, None)
    return cv2.undistort(img, mtx, dist, None, mtx)


def perspective(img):
    width = img.shape[1]
    height = img.shape[0]

    # create source
    src = np.array([[(190, 720), (599, 445), (679, 445), (1124, 720)]], dtype=np.float32)
    # create destiny
    dst = np.array([[(200, 720), (200, 0), (970, 0), (970, 720)]], dtype=np.float32)

    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(img, M, (width, height))

    return warped, M


def calibrate_camera():

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6 * 9, 3), np.float32)
    objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

    # Arrays to store object points and image points from all the images.
    objpoints = []  # 3d points in real world space
    imgpoints = []  # 2d points in image plane.

    # Make a list of calibration images
    images = glob.glob('camera_cal/calibration*.jpg')

    # Step through the list and search for chessboard corners
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Find the chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, (9, 6), None)

        # If found, add object points, image points
        if ret == True:
            objpoints.append(objp)
            imgpoints.append(corners)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (9, 6), corners, ret)
            cv2.imshow('img',img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    return objpoints, imgpoints