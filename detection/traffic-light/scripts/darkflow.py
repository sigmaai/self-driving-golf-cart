#!/usr/bin/env python

from net.build import TFNet
import cv2
import rospy
from tlight_node import TLightNode
import argparse


def process(model, img):
    result = model.return_predict(img[None, :, :, :])
    return result

def getModel():
    options = {"model": "./cfg/tiny-yolo-udacity.cfg", "backup": "./ckpt/","load": 8987, "gpu": 1.0}
    model = TFNet(options)
    return model

def main():
    parser = argparse.ArgumentParser(description='Script for running yolo_light node')
    args = parser.parse_args()
    node = TLightNode(lambda: getModel(), process)
    rospy.spin()

if __name__ == '__main__':
    main()
