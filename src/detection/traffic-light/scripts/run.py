from net.build import TFNet
from utils import loader
import cv2
import glob
import time
import numpy as np

options = {"test": "test/", "model": "cfg/tiny-yolo-udacity.cfg", "backup": "ckpt/","load": 8987, "gpu": 1.0}

tfnet = TFNet(options)

images = glob.glob('./test/*.jpg')
i = 0
average = []
for image in images:
    t = time.time()
    imgcv = cv2.imread(image)
    name = image
    result = tfnet.return_predict(imgcv,name)
    t2 = time.time()
    average.append(t2-t)
    i += 1
    print(i,'time1:',(t2-t))
final = np.mean(average)
print(i,'images processed in avg: ', round(final,6), 'seconds per image')
