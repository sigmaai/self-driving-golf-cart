#!/usr/bin/env python
#
#  THE KITTI VISION BENCHMARK SUITE: ROAD BENCHMARK
#
#  Copyright (C) 2013
#  Honda Research Institute Europe GmbH
#  Carl-Legien-Str. 30
#  63073 Offenbach/Main
#  Germany
#
#  UNPUBLISHED PROPRIETARY MATERIAL.
#  ALL RIGHTS RESERVED.
#
#  Authors: Tobias Kuehnl <tkuehnl@cor-lab.uni-bielefeld.de>
#           Jannik Fritsch <jannik.fritsch@honda-ri.de>
#

from BirdsEyeView import BirdsEyeView
from glob import glob
import os,sys
import cv2 # OpenCV

#########################################################################
# function that does the transformation: Image --> BirdsEyeView
#########################################################################
def main(dataFiles, pathToCalib, outputPath, calib_end  = '.txt'):
    '''
    Main method of transform2BEV
    :param dataFiles: the files you want to transform to BirdsEyeView, e.g., /home/elvis/kitti_road/data/*.png
    :param pathToCalib: containing calib data as txt-files, e.g., /home/elvis/kitti_road/calib/
    :param outputPath: where the BirdsEyeView data will be saved, e.g., /home/elvis/kitti_road/data_bev
    :param calib_end: file extension of calib-files (OPTIONAL)
    '''
    
    
    # Extract path of data
    pathToData = os.path.split(dataFiles)[0]
    assert os.path.isdir(pathToData), "The directory containig the input data seems to not exist!"
    assert os.path.isdir(pathToCalib), "Error <PathToCalib> does not exist"
      
    # BEV class
    bev = BirdsEyeView()
    
    #check
    if not os.path.isdir(outputPath):
        os.makedirs(outputPath)  
    
    # get filelist
    fileList_data = glob(dataFiles)
    assert len(fileList_data), 'Could not find files in: %s' %pathToData
    
    # Loop over all files
    for aFile in fileList_data:
        assert os.path.isfile(aFile), '%s is not a file' %aFile
        
        file_key = aFile.split('/')[-1].split('.')[0]
        print "Transforming file %s to Birds Eye View " %file_key 
        tags = file_key.split('_')
        data_end = aFile.split(file_key)[-1]
        
        #calibration filename
        calib_file = os.path.join(pathToCalib, file_key + calib_end)
        
        if not os.path.isfile(calib_file) and len(tags)==3:
            # exclude lane or road from filename!
            calib_file = os.path.join(pathToCalib, tags[0]+ '_' + tags[2] + calib_end)
        
        # Check if calb file exist!
        if not os.path.isfile(calib_file):
            print "Cannot find calib file: %s" %calib_file
            print "Attention: It is assumed that input data and calib files have the same name (only different extension)!" 
            sys.exit(1)
            
        # Update calibration for Birds Eye View
        bev.setup(calib_file)
        
        # Read image
        data = cv2.imread(aFile, cv2.CV_LOAD_IMAGE_UNCHANGED)
        
        # Compute Birds Eye View
        data_bev = bev.compute(data)
        
        # Write output (BEV)
        fn_out = os.path.join(outputPath,file_key + data_end)
        if (cv2.imwrite(fn_out, data_bev)):
            print "done ..."
        else:
            print "saving to %s failed ... (permissions?)"%outputPath
            return
       
    print "BirdsEyeView was stored in: %s" %outputPath
    
#########################################################################
# transformation script Image --> BirdsEyeView
#########################################################################       
if __name__ == "__main__":
    
    print sys.argv
    # check for correct number of arguments.
    if len(sys.argv)!=4:
        print "Usage: python transform2BEV.py <InputFiles> <PathToCalib> <OutputPath> "
        print "<InputFiles>: the files you want to transform to BirdsEyeView, e.g., '/home/elvis/kitti_road/data/*.png' (use quotes!)"
        print "<PathToCalib>: containing calib data as calib-files, e.g., /home/elvis/kitti_road/calib/" 
        print "<OutputPath>: where the BirdsEyeView data will be saved, e.g., /home/elvis/kitti_road/data_bev"
        print "ATTENTION: It is assumed that input data and calib files have the same name (only different extension)!"
        print  "Your provided parameters: ", sys.argv
        sys.exit(1)
    
    # parse parameters
    dataFiles = sys.argv[1]
    pathToCalib = sys.argv[2]
    outputPath = sys.argv[3]
    
    # Excecute main fun
    main(dataFiles, pathToCalib, outputPath)
    