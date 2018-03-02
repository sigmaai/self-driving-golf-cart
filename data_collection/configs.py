# Master config file for the vehicle
# ------------ AGC -----------------
# Neil Nie & Michaeling Meng
# Deerfield Academy, 2018

st_fac = 20
verbose = True
default_img_size = (480, 640)
segmentation_size = (512, 512)

cruise = False
navigation = False
segmentation = True
left_vid_src = 2
cent_vid_src = 1
right_vid_src = 3
# set to true if want to use default serial port
default_st_port = False
CV_CAP_STR = "nvcamerasrc ! video/x-raw(memory:NVMM), width=(int)720, height=(int)576,format=(string)I420, framerate=(fraction)1/1 ! nvvidconv flip-method=0 ! video/x-raw, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink"
