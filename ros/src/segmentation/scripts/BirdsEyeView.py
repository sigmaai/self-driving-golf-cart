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

import numpy as np
import os


class BevParams(object):

    # Param 
    bev_size = None
    bev_res = None
    bev_xLimits = None
    bev_zLimits = None
    imSize = None
    imSize_back = None

    def __init__(self, bev_res, bev_xLimits, bev_zLimits, im_size):
        """

        :param bev_res:
        :param bev_xLimits:
        :param bev_zLimits:
        :param imSize:
        """
        bev_size = (round((bev_zLimits[1] - bev_zLimits[0]) / bev_res), round((bev_xLimits[1] - bev_xLimits[0]) / bev_res))
        self.bev_size = bev_size
        self.bev_res = bev_res
        self.bev_xLimits = bev_xLimits
        self.bev_zLimits = bev_zLimits
        self.imSize = im_size

    def px2meter(self, px_in):
        """

        :param px_in:
        :return:
        """
        return px_in * self.bev_res

    def meter2px(self, meter_in):
        """

        :param meter_in:
        :return:
        """
        return meter_in / self.bev_res

    def convert_position_metric2pixel(self, YXpointArrays):
        """

        :param YXpointArrays:
        :return:
        """
        allY = YXpointArrays[:, 0]
        allX = YXpointArrays[:, 1]
        allYconverted = self.bev_size[0] - self.meter2px(allY - self.bev_zLimits[0])
        allXconverted = self.meter2px(allX - self.bev_xLimits[0])
        return np.array(np.append(allYconverted.reshape((len(allYconverted), 1)),
                                  allXconverted.reshape((len(allXconverted), 1)),
                                  axis=1))

    def convert_position_pixel2metric(self, YXpointArrays):
        """

        :param YXpointArrays:
        :return:
        """
        allY = YXpointArrays[:, 0]
        allX = YXpointArrays[:, 1]
        allYconverted = self.px2meter(self.bev_size[0] - allY) + self.bev_zLimits[0]
        allXconverted = self.px2meter(allX) + self.bev_xLimits[0]
        return np.array(
            np.append(allYconverted.reshape((len(allYconverted), 1)), allXconverted.reshape((len(allXconverted), 1)),
                      axis=1))

    def convert_position_pixel2metric2(self, inputTupleY, inputTupleX):
        '''

        @param inputTupleY:
        @param inputTupleX:
        '''

        return (self.px2meter(self.bev_size[0] - inputTupleY) + self.bev_zLimits[0],
                self.px2meter(inputTupleX) + self.bev_xLimits[0])


def readKittiCalib(filename, dtype='f8'):
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
        if content == '':
            continue
        if content[0] != '#':
            tmp = content.split(':')
            assert len(tmp) == 2, 'wrong file format, only one : per line!'
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

        pass

    def read_from_file(self, filekey=None, fn=None):
        """

        :param filekey:
        :param fn:
        :return:
        """

        if filekey is not None:
            fn = os.path.join(self.calib_dir, filekey + self.calib_end)

        assert fn is not None, 'Problem! fn or filekey must be != None'
        cur_calibStuff_dict = readKittiCalib(fn)
        self.setup(cur_calibStuff_dict)

    def setup(self, dictWithKittiStuff, useRect=False):
        """

        :param dictWithKittiStuff:
        :param useRect:
        :return:
        """

        dtype_str = 'f8'
        # dtype = np.float64

        self.P2 = np.matrix(dictWithKittiStuff['P2']).reshape((3, 4))

        if useRect:
            #
            R2_1 = self.P2
            # self.R0_rect = None

        else:
            R0_rect_raw = np.array(dictWithKittiStuff['R0_rect']).reshape((3, 3))
            # Rectification Matrix
            self.R0_rect = np.matrix(
                np.hstack((np.vstack((R0_rect_raw, np.zeros((1, 3), dtype_str))), np.zeros((4, 1), dtype_str))))
            self.R0_rect[3, 3] = 1.
            # intermediate result
            R2_1 = np.dot(self.P2, self.R0_rect)

        Tr_cam_to_road_raw = np.array(dictWithKittiStuff['Tr_cam_to_road']).reshape(3, 4)
        # Transformation matrixs
        self.Tr_cam_to_road = np.matrix(np.vstack((Tr_cam_to_road_raw, np.zeros((1, 4), dtype_str))))
        self.Tr_cam_to_road[3, 3] = 1.

        self.Tr = np.dot(R2_1, self.Tr_cam_to_road.I)
        self.Tr33 = self.Tr[:, [0, 2, 3]]

    def get_matrix33(self):

        assert self.Tr33.all() is not None
        return self.Tr33


class BirdsEyeView(object):

    imSize = None
    bevParams = None
    invalid_value = float('-INFINITY')
    im_u_float = None
    im_v_float = None
    bev_x_ind = None
    bev_z_ind = None

    def __init__(self, bev_res=0.05, bev_x_range_min_max=(-16, 16), bev_z_range_min_max=(2, 35)):
        """

        :param bev_res:
        :param bev_x_range_min_max:
        :param bev_z_range_min_max:
        """

        self.calib = KittiCalibration()
        bev_res = bev_res
        bev_xRange_minMax = bev_x_range_min_max
        bev_zRange_minMax = bev_z_range_min_max
        self.bevParams = BevParams(bev_res, bev_xRange_minMax, bev_zRange_minMax, self.imSize)

    def world2image(self, X_world, Y_world, Z_world):
        """

        :param X_world:
        :param Y_world:
        :param Z_world:
        :return:
        """

        if not type(Y_world) == np.ndarray:
            # Y can be a scalar
            Y_world = np.ones_like(Z_world) * Y_world
        y = np.vstack((X_world, Y_world, Z_world, np.ones_like(Z_world)))
        test = self.world2image_uvMat(np.vstack((X_world, Z_world, np.ones_like(Z_world))))

        self.xi1 = test[0, :]  # vec2[0,:]
        self.yi1 = test[1, :]  # vec2[1,:]

        assert self.imSize != None
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
        """

        :param uv_mat:
        :return:
        """
        if uv_mat.shape[0] == 2:
            if len(uv_mat.shape) == 1:
                uv_mat = uv_mat.reshape(uv_mat.shape + (1,))
            uv_mat = np.vstack((uv_mat, np.ones((1, uv_mat.shape[1]), uv_mat.dtype)))
        result = np.dot(self.Tr33, uv_mat)
        # w0 = -(uv_mat[0]* self.Tr_inv_33[1,0]+ uv_mat[1]* self.Tr_inv[1,1])/self.Tr_inv[1,2]
        resultB = np.broadcast_arrays(result, result[-1, :])
        return resultB[0] / resultB[1]

    def setup(self, calib_file):
        """

        :param calib_file:
        :return:
        """

        self.calib.read_from_file(fn=calib_file)
        self.set_matrix33(self.calib.get_matrix33())

    def set_matrix33(self, matrix33):
        """

        :param matrix33:
        :return:
        """
        self.Tr33 = matrix33

    def compute(self, data):
        """

        :param data:
        :return:
        """
        self.imSize = data.shape
        self.compute_BEV_look_up_table()
        return self.transform_image_2BEV(data, out_dtype=data.dtype)

    def compute_reverse(self, data, imSize):
        """

        :param data:
        :param imSize:
        :return:
        """
        self.imSize = imSize
        self.compute_BEV_look_up_table_reverse()
        return self.transformBEV2Image(data, out_dtype=data.dtype)

    def compute_BEV_look_up_table_reverse(self, imSize=None):
        """

        :param imSize:
        :return:
        """

        mgrid = np.lib.index_tricks.nd_grid()

        # [y_im,x_im]=ndgrid(1:camParam.imSize_org(1),1:camParam.imSize_org(2));
        if imSize is None:
            # Take default imSize!
            imSize = self.imSize
        self.imSize_back = (imSize[0], imSize[1],)
        yx_im = (mgrid[1:self.imSize_back[0] + 1, 1:self.imSize_back[1] + 1]).astype('i4')
        # Equation of pinhole camera
        y_im = yx_im[0, :, :]
        x_im = yx_im[1, :, :]

        dim = self.imSize_back[0] * self.imSize_back[1]
        uvMat = np.vstack((x_im.flatten(), y_im.flatten(), np.ones((dim,), 'f4')))
        xzMat = self.image2world_uv_mat(uvMat)
        X = xzMat[0, :].reshape(x_im.shape)
        Z = xzMat[1, :].reshape(x_im.shape)

        XBevInd_reverse_all = np.round((X - self.bevParams.bev_xLimits[0]) / self.bevParams.bev_res).astype('i4')
        ZBevInd_reverse_all = np.round(
            self.bevParams.bev_size[0] - (Z - self.bevParams.bev_zLimits[0]) / self.bevParams.bev_res).astype('i4')
        # Valid?
        self.validMapIm_reverse = (XBevInd_reverse_all >= 1) & (XBevInd_reverse_all <= self.bevParams.bev_size[1]) & (
                    ZBevInd_reverse_all >= 1) & (ZBevInd_reverse_all <= self.bevParams.bev_size[0])

        self.XBevInd_reverse = XBevInd_reverse_all[self.validMapIm_reverse] - 1
        self.ZBevInd_reverse = ZBevInd_reverse_all[self.validMapIm_reverse] - 1

        self.xImInd_reverse = x_im[self.validMapIm_reverse] - 1
        self.yImInd_reverse = y_im[self.validMapIm_reverse] - 1

    def image2world_uv_mat(self, uv_mat):
        """

        :param uv_mat:
        :return:
        """
        if uv_mat.shape[0] == 2:
            if len(uv_mat.shape) == 1:
                uv_mat = uv_mat.reshape(uv_mat.shape + (1,))
            uv_mat = np.vstack((uv_mat, np.ones((1, uv_mat.shape[1]), uv_mat.dtype)))
        result = np.dot(self.Tr33.I, uv_mat)
        # w0 = -(uv_mat[0]* self.Tr_inv_33[1,0]+ uv_mat[1]* self.Tr_inv[1,1])/self.Tr_inv[1,2]
        resultB = np.broadcast_arrays(result, result[-1, :])
        return resultB[0] / resultB[1]

    def compute_BEV_look_up_table(self, cropping_ul=None, cropping_size=None):
        """

        :param cropping_ul:
        :param cropping_size:
        :return:
        """
        # compute X,Z mesh from BEV params
        mgrid = np.lib.index_tricks.nd_grid()

        res = self.bevParams.bev_res

        x_vec = np.arange(self.bevParams.bev_xLimits[0] + res / 2, self.bevParams.bev_xLimits[1], res)
        z_vec = np.arange(self.bevParams.bev_zLimits[1] - res / 2, self.bevParams.bev_zLimits[0], -res)
        XZ_mesh = np.meshgrid(x_vec, z_vec)

        assert XZ_mesh[0].shape == self.bevParams.bev_size

        Z_mesh_vec = (np.reshape(XZ_mesh[1], int(self.bevParams.bev_size[0] * self.bevParams.bev_size[1]), order='F')).astype('f4')
        X_mesh_vec = (np.reshape(XZ_mesh[0], int(self.bevParams.bev_size[0] * self.bevParams.bev_size[1]), order='F')).astype('f4')

        self.world2image(X_mesh_vec, 0, Z_mesh_vec)
        # output-> (y, x)
        if (cropping_ul is not None):
            valid_selector = np.ones((self.bevParams.bev_size[0] * self.bevParams.bev_size[1],), dtype='bool')
            valid_selector = valid_selector & (self.yi1 >= cropping_ul[0]) & (self.xi1 >= cropping_ul[1])
            if (cropping_size is not None):
                valid_selector = valid_selector & (self.yi1 <= (cropping_ul[0] + cropping_size[0])) & (
                            self.xi1 <= (cropping_ul[1] + cropping_size[1]))
            # using selector to delete invalid pixel
            selector = (~(self.xi1 == self.invalid_value)).reshape(valid_selector.shape) & valid_selector  # store invalid value positions
        else:
            # using selector to delete invalid pixel
            selector = ~(self.xi1 == self.invalid_value)  # store invalid value positions

        y_OI_im_sel = self.yi1[selector]  # without invalid pixel
        x_OI_im_sel = self.xi1[selector]  # without invalid pixel

        # indices for bev positions for the LookUpTable
        ZX_ind = (mgrid[1:self.bevParams.bev_size[0] + 1, 1:self.bevParams.bev_size[1] + 1]).astype('i4')
        Z_ind_vec = np.reshape(ZX_ind[0], selector.shape, order='F')
        X_ind_vec = np.reshape(ZX_ind[1], selector.shape, order='F')

        # Select
        Z_ind_vec_sel = Z_ind_vec[selector]  # without invalid pixel
        X_ind_vec_sel = X_ind_vec[selector]  # without invalid pixel

        # Save stuff for LUT in BEVParams
        self.im_u_float = x_OI_im_sel
        self.im_v_float = y_OI_im_sel
        self.bev_x_ind = X_ind_vec_sel.reshape(x_OI_im_sel.shape)
        self.bev_z_ind = Z_ind_vec_sel.reshape(y_OI_im_sel.shape)

    def transform_image_2BEV(self, inImage, out_dtype='f4'):

        assert self.im_u_float.all() != None
        assert self.im_v_float.all() != None
        assert self.bev_x_ind.all() != None
        assert self.bev_z_ind.all() != None

        if len(inImage.shape) > 2:
            shape = self.bevParams.bev_size + (inImage.shape[2],)
            outputData = np.zeros([int(shape[0]), int(shape[1]), int(shape[2])], dtype=out_dtype)
            for channel in xrange(0, inImage.shape[2]):
                outputData[self.bev_z_ind - 1, self.bev_x_ind - 1, channel] = inImage[
                    self.im_v_float.astype('u4') - 1, self.im_u_float.astype('u4') - 1, channel]
        else:
            outputData = np.zeros(self.bevParams.bev_size, dtype=out_dtype)
            outputData[self.bev_z_ind - 1, self.bev_x_ind - 1] = inImage[
                self.im_v_float.astype('u4') - 1, self.im_u_float.astype('u4') - 1]

        return outputData

    def transformBEV2Image(self, bevMask, out_dtype='f4'):
        """

        :param bevMask:
        :param out_dtype:
        :return:
        """
        assert self.xImInd_reverse is not None
        assert self.yImInd_reverse is not None
        assert self.XBevInd_reverse is not None
        assert self.ZBevInd_reverse is not None
        assert self.imSize_back is not None
        if len(bevMask.shape) > 2:
            outputData = np.zeros(self.imSize_back + (bevMask.shape[2],), dtype=out_dtype)
            for channel in xrange(0, bevMask.shape[2]):
                outputData[self.yImInd_reverse, self.xImInd_reverse, channel] = bevMask[
                    self.ZBevInd_reverse, self.XBevInd_reverse, channel]
        else:
            outputData = np.zeros(self.imSize_back, dtype=out_dtype)
            outputData[self.yImInd_reverse, self.xImInd_reverse] = bevMask[self.ZBevInd_reverse, self.XBevInd_reverse]
        # Return result
        return outputData        
