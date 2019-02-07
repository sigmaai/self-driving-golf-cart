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

import logging
import numpy as np
import os

class BevParams(object):
    '''

    '''
    # Param 
    bev_size = None
    bev_res = None
    bev_xLimits = None
    bev_zLimits = None
    imSize = None
    imSize_back = None

    def __init__(self, bev_res, bev_xLimits, bev_zLimits, imSize):
        '''

        @param bev_size:
        @param bev_res:
        @param bev_xLimits:
        @param bev_zLimits:
        @param imSize:
        '''
        bev_size = (round((bev_zLimits[1] - bev_zLimits[0]) / bev_res), \
                    round((bev_xLimits[1] - bev_xLimits[0]) / bev_res))
        self.bev_size = bev_size
        self.bev_res = bev_res
        self.bev_xLimits = bev_xLimits
        self.bev_zLimits = bev_zLimits
        self.imSize = imSize

    def px2meter(self, px_in):
        '''

        @param px_in:
        '''
        return px_in * self.bev_res

    def meter2px(self, meter_in):
        '''

        @param meter_in:
        '''
        return meter_in / self.bev_res

    def convertPositionMetric2Pixel(self, YXpointArrays):
        '''

        @param YXpointArrays:
        '''
        allY = YXpointArrays[:, 0]
        allX = YXpointArrays[:, 1]
        allYconverted = self.bev_size[0] - self.meter2px(allY - self.bev_zLimits[0])
        allXconverted = self.meter2px(allX - self.bev_xLimits[0])
        return np.array(np.append(allYconverted.reshape((len(allYconverted), 1)), allXconverted.reshape((len(allXconverted), 1)), axis = 1))
        #    return [self.meter2px(inputTupleZ + self.bev_zLimits[0]), self.meter2px(inputTupleX + self.bev_xLimits[0])]


    def convertPositionPixel2Metric(self, YXpointArrays):
        '''

        @param YXpointArrays:
        '''
        allY = YXpointArrays[:, 0]
        allX = YXpointArrays[:, 1]
        allYconverted = self.px2meter(self.bev_size[0] - allY) + self.bev_zLimits[0]
        allXconverted = self.px2meter(allX) + self.bev_xLimits[0]
        return np.array(np.append(allYconverted.reshape((len(allYconverted), 1)), allXconverted.reshape((len(allXconverted), 1)), axis = 1))



    def convertPositionPixel2Metric2(self, inputTupleY, inputTupleX):
        '''

        @param inputTupleY:
        @param inputTupleX:
        '''

        return (self.px2meter(self.bev_size[0] - inputTupleY) + self.bev_zLimits[0], self.px2meter(inputTupleX) + self.bev_xLimits[0])

def readKittiCalib(filename, dtype = 'f8'):
    '''
    
    :param filename:
    :param dtype:
    '''
    outdict = dict()
    output = open(filename, 'rb')
    allcontent = output.readlines()
    output.close()
    for contentRaw in allcontent:
        content = contentRaw.strip()
        if content=='':
            continue
        if content[0]!='#' :
            tmp = content.split(':')
            assert len(tmp)==2, 'wrong file format, only one : per line!'
            var = tmp[0].strip()
            values = np.array(tmp[-1].strip().split(' '), dtype)

            outdict[var] = values

    return outdict


class KittiCalibration(object):
    calib_dir = None
    calib_end = None
    R0_rect = None
    P2 = None
    Tr33 = None
    Tr = None
    Tr_cam_to_road = None

    def __init__(self):
        '''
        '''
        pass

    def readFromFile(self, filekey = None, fn = None):
        '''

        @param fn:
        '''


        if filekey != None:
            fn = os.path.join(self.calib_dir, filekey + self.calib_end)

        assert fn != None, 'Problem! fn or filekey must be != None'
        cur_calibStuff_dict = readKittiCalib(fn)
        self.setup(cur_calibStuff_dict)

    def setup(self, dictWithKittiStuff, useRect = False):
        '''

        @param dictWithKittiStuff:
        '''
        dtype_str = 'f8'
        #dtype = np.float64

        self.P2 = np.matrix(dictWithKittiStuff['P2']).reshape((3,4))

        if useRect:
            #
            R2_1 = self.P2
            #self.R0_rect = None

        else:
            R0_rect_raw = np.array(dictWithKittiStuff['R0_rect']).reshape((3,3))
            # Rectification Matrix
            self.R0_rect = np.matrix(np.hstack((np.vstack((R0_rect_raw, np.zeros((1,3), dtype_str))), np.zeros((4,1), dtype_str))))
            self.R0_rect[3,3]=1.
            # intermediate result
            R2_1 = np.dot(self.P2, self.R0_rect)

        Tr_cam_to_road_raw = np.array(dictWithKittiStuff['Tr_cam_to_road']).reshape(3,4)
        # Transformation matrixs
        self.Tr_cam_to_road = np.matrix(np.vstack((Tr_cam_to_road_raw, np.zeros((1,4), dtype_str))))
        self.Tr_cam_to_road[3,3]=1.

        self.Tr = np.dot(R2_1,  self.Tr_cam_to_road.I)
        self.Tr33 =  self.Tr[:,[0,2,3]]

    def get_matrix33(self):
        '''

        '''
        assert self.Tr33 != None
        return self.Tr33


class BirdsEyeView(object):
    '''

    '''
    imSize = None
    bevParams = None
    invalid_value = float('-INFINITY')
    im_u_float = None
    im_v_float = None
    bev_x_ind = None
    bev_z_ind = None
    def __init__(self, bev_res= 0.05, bev_xRange_minMax = (-10, 10), bev_zRange_minMax = (6, 46)):
        '''
        
        :param bev_res:
        :param bev_xRange_minMax:
        :param bev_zRange_minMax:
        '''


        self.calib = KittiCalibration()
        bev_res = bev_res
        bev_xRange_minMax = bev_xRange_minMax
        bev_zRange_minMax = bev_zRange_minMax
        self.bevParams = BevParams(bev_res, bev_xRange_minMax, bev_zRange_minMax, self.imSize)


    def world2image(self, X_world, Y_world, Z_world):
        '''

        @param X_world:
        @param Y_world:
        @param Z_world:
        '''

        if not type(Y_world) == np.ndarray:
            # Y can be a scalar
            Y_world = np.ones_like(Z_world)*Y_world
        y = np.vstack((X_world, Y_world, Z_world, np.ones_like(Z_world)))
        test  = self.world2image_uvMat(np.vstack((X_world, Z_world, np.ones_like(Z_world) )))

        self.xi1 = test[0,:]#vec2[0,:]
        self.yi1 = test[1,:]#vec2[1,:]

        assert  self.imSize != None
        condition = ~((self.yi1 >= 1) & (self.xi1 >= 1) & (self.yi1 <= self.imSize[0]) & (self.xi1 <= self.imSize[1]))
        if isinstance(condition, np.ndarray):
            # Array
            self.xi1[condition] = self.invalid_value
            self.yi1[condition] = self.invalid_value
        elif condition == True:
            # Scalar
            self.xi1 = self.invalid_value
            self.yi1 = self.invalid_value

    def world2image_uvMat(self, uv_mat):
        '''

        @param XYZ_mat: is a 4 or 3 times n matrix
        '''
        if uv_mat.shape[0] == 2:
            if len(uv_mat.shape)==1:
                uv_mat = uv_mat.reshape(uv_mat.shape + (1,))
            uv_mat = np.vstack((uv_mat, np.ones((1, uv_mat.shape[1]), uv_mat.dtype)))
        result = np.dot(self.Tr33, uv_mat)
        #w0 = -(uv_mat[0]* self.Tr_inv_33[1,0]+ uv_mat[1]* self.Tr_inv[1,1])/self.Tr_inv[1,2]
        resultB = np.broadcast_arrays(result, result[-1,:])
        return resultB[0] / resultB[1]

    def setup(self, calib_file):
        '''
        
        :param calib_file:
        '''

        self.calib.readFromFile(fn= calib_file)
        self.set_matrix33(self.calib.get_matrix33())

    def set_matrix33(self, matrix33):
        '''

        @param matrix33:
        '''
        self.Tr33 = matrix33

    def compute(self, data):
        '''
        Compute BEV
        :param data:
        '''
        self.imSize = data.shape
        self.computeBEVLookUpTable()
        return self.transformImage2BEV(data, out_dtype = data.dtype)

    def compute_reverse(self, data, imSize):
        '''
        Compute BEV
        :param data:
        '''
        self.imSize = imSize
        self.computeBEVLookUpTable_reverse()
        return self.transformBEV2Image(data, out_dtype = data.dtype)

    def computeBEVLookUpTable_reverse(self, imSize = None):
        '''

        '''

        mgrid = np.lib.index_tricks.nd_grid()

        #[y_im,x_im]=ndgrid(1:camParam.imSize_org(1),1:camParam.imSize_org(2));
        if imSize == None:
            # Take default imSize!
            imSize = self.imSize
        self.imSize_back = (imSize[0], imSize[1],)
        yx_im = (mgrid[1:self.imSize_back[0] + 1, 1:self.imSize_back[1] + 1]).astype('i4')
        # Equation of pinhole camera
        y_im = yx_im[0, :, :]
        x_im = yx_im[1, :, :]

        dim =self.imSize_back[0]*self.imSize_back[1]
        uvMat = np.vstack((x_im.flatten(), y_im.flatten(), np.ones((dim,), 'f4')))
        xzMat = self.image2world_uvMat(uvMat)
        X = xzMat[0,:].reshape(x_im.shape)
        Z = xzMat[1,:].reshape(x_im.shape)

        XBevInd_reverse_all = np.round((X - self.bevParams.bev_xLimits[0]) / self.bevParams.bev_res).astype('i4')
        ZBevInd_reverse_all = np.round(self.bevParams.bev_size[0] - (Z - self.bevParams.bev_zLimits[0]) / self.bevParams.bev_res).astype('i4')
        # Valid?
        self.validMapIm_reverse = (XBevInd_reverse_all >= 1) & (XBevInd_reverse_all <= self.bevParams.bev_size[1]) & (ZBevInd_reverse_all >= 1) & (ZBevInd_reverse_all <= self.bevParams.bev_size[0])

        self.XBevInd_reverse = XBevInd_reverse_all[self.validMapIm_reverse] - 1
        self.ZBevInd_reverse = ZBevInd_reverse_all[self.validMapIm_reverse] - 1

        self.xImInd_reverse = x_im[self.validMapIm_reverse] - 1
        self.yImInd_reverse = y_im[self.validMapIm_reverse] - 1

    def image2world_uvMat(self, uv_mat):
        '''

        @param XYZ_mat: is a 4 or 3 times n matrix
        '''
        #assert self.transformMode == 'kitti' or self.transformMode == 'angles'
        if uv_mat.shape[0] == 2:
            if len(uv_mat.shape)==1:
                uv_mat = uv_mat.reshape(uv_mat.shape + (1,))
            uv_mat = np.vstack((uv_mat, np.ones((1, uv_mat.shape[1]), uv_mat.dtype)))
        result = np.dot(self.Tr33.I, uv_mat)
        #w0 = -(uv_mat[0]* self.Tr_inv_33[1,0]+ uv_mat[1]* self.Tr_inv[1,1])/self.Tr_inv[1,2]
        resultB = np.broadcast_arrays(result, result[-1,:])
        return resultB[0] / resultB[1]

    def computeBEVLookUpTable(self, cropping_ul = None, cropping_size = None):
        '''

        @param cropping_ul:
        @param cropping_size:
        '''

        # compute X,Z mesh from BEV params
        mgrid = np.lib.index_tricks.nd_grid()

        res = self.bevParams.bev_res

        x_vec = np.arange(self.bevParams.bev_xLimits[0] + res / 2, self.bevParams.bev_xLimits[1], res)
        z_vec = np.arange(self.bevParams.bev_zLimits[1] - res / 2, self.bevParams.bev_zLimits[0], -res)
        XZ_mesh = np.meshgrid(x_vec, z_vec)


        assert XZ_mesh[0].shape == self.bevParams.bev_size


        Z_mesh_vec = (np.reshape(XZ_mesh[1], (self.bevParams.bev_size[0] * self.bevParams.bev_size[1]), order = 'F')).astype('f4')
        X_mesh_vec = (np.reshape(XZ_mesh[0], (self.bevParams.bev_size[0] * self.bevParams.bev_size[1]), order = 'F')).astype('f4')


        self.world2image(X_mesh_vec, 0, Z_mesh_vec)
        # output-> (y, x)
        if (cropping_ul is not None):
            valid_selector = np.ones((self.bevParams.bev_size[0] * self.bevParams.bev_size[1],), dtype = 'bool')
            valid_selector = valid_selector & (self.yi1 >= cropping_ul[0]) & (self.xi1 >= cropping_ul[1])
            if (cropping_size is not None):
                valid_selector = valid_selector & (self.yi1 <= (cropping_ul[0] + cropping_size[0])) & (self.xi1 <= (cropping_ul[1] + cropping_size[1]))
            # using selector to delete invalid pixel
            selector = (~(self.xi1 == self.invalid_value)).reshape(valid_selector.shape) & valid_selector  # store invalid value positions
        else:
            # using selector to delete invalid pixel
            selector = ~(self.xi1 == self.invalid_value)  # store invalid value positions

        y_OI_im_sel = self.yi1[selector]# without invalid pixel
        x_OI_im_sel = self.xi1[selector]# without invalid pixel


        # indices for bev positions for the LookUpTable
        ZX_ind = (mgrid[1:self.bevParams.bev_size[0] + 1, 1:self.bevParams.bev_size[1] + 1]).astype('i4')
        Z_ind_vec = np.reshape(ZX_ind[0], selector.shape, order = 'F')
        X_ind_vec = np.reshape(ZX_ind[1], selector.shape, order = 'F')

        # Select
        Z_ind_vec_sel = Z_ind_vec[selector] # without invalid pixel
        X_ind_vec_sel = X_ind_vec[selector] # without invalid pixel

        # Save stuff for LUT in BEVParams
        self.im_u_float = x_OI_im_sel
        self.im_v_float = y_OI_im_sel
        self.bev_x_ind = X_ind_vec_sel.reshape(x_OI_im_sel.shape)
        self.bev_z_ind = Z_ind_vec_sel.reshape(y_OI_im_sel.shape)

    def transformImage2BEV(self, inImage, out_dtype = 'f4'):
        '''
        
        :param inImage:
        '''
        assert self.im_u_float != None
        assert self.im_v_float != None
        assert self.bev_x_ind != None
        assert self.bev_z_ind != None


        if len(inImage.shape) > 2:
            outputData = np.zeros(self.bevParams.bev_size + (inImage.shape[2],), dtype = out_dtype)
            for channel in xrange(0, inImage.shape[2]):
                outputData[self.bev_z_ind-1, self.bev_x_ind-1, channel] = inImage[self.im_v_float.astype('u4')-1, self.im_u_float.astype('u4')-1, channel]
        else:
            outputData = np.zeros(self.bevParams.bev_size, dtype = out_dtype)
            outputData[self.bev_z_ind-1, self.bev_x_ind-1] = inImage[self.im_v_float.astype('u4')-1, self.im_u_float.astype('u4')-1]

        return  outputData

    def transformBEV2Image(self, bevMask, out_dtype = 'f4'):
        '''

        @param bevMask:
        '''
        assert self.xImInd_reverse != None
        assert self.yImInd_reverse != None
        assert self.XBevInd_reverse != None
        assert self.ZBevInd_reverse != None
        assert self.imSize_back != None
        if len(bevMask.shape) > 2:
            outputData = np.zeros(self.imSize_back + (bevMask.shape[2],), dtype = out_dtype)
            for channel in xrange(0, bevMask.shape[2]):
                outputData[self.yImInd_reverse, self.xImInd_reverse, channel] = bevMask[self.ZBevInd_reverse, self.XBevInd_reverse, channel]
        else:
            outputData = np.zeros(self.imSize_back, dtype = out_dtype)
            outputData[self.yImInd_reverse, self.xImInd_reverse] = bevMask[self.ZBevInd_reverse, self.XBevInd_reverse]
        # Return result
        return outputData        
