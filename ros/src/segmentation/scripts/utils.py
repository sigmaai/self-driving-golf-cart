# 
# utilities for semantic segmentation
# autonomous golf cart project
# (c) Yongyang Nie, Michael Meng
#

import cv2
import configs as configs
import numpy as np
import pandas
from collections import namedtuple
import os
import glob
import random
import json
import gc

from keras.utils import to_categorical
from keras.callbacks import Callback
from keras.utils.data_utils import Sequence


#--------------------------------------------------------------------------------
# Definitions
#--------------------------------------------------------------------------------

# a label and all meta information
Label = namedtuple( 'Label' , [

    'name'        , # The identifier of this label, e.g. 'car', 'person', ... .
                    # We use them to uniquely name a class

    'id'          , # An integer ID that is associated with this label.
                    # The IDs are used to represent the label in ground truth images
                    # An ID of -1 means that this label does not have an ID and thus
                    # is ignored when creating ground truth images (e.g. license plate).
                    # Do not modify these IDs, since exactly these IDs are expected by the
                    # evaluation server.

    'trainId'     , # Feel free to modify these IDs as suitable for your method. Then create
                    # ground truth images with train IDs, using the tools provided in the
                    # 'preparation' folder. However, make sure to validate or submit results
                    # to our evaluation server using the regular IDs above!
                    # For trainIds, multiple labels might have the same ID. Then, these labels
                    # are mapped to the same class in the ground truth images. For the inverse
                    # mapping, we use the label that is defined first in the list below.
                    # For example, mapping all void-type classes to the same ID in training,
                    # might make sense for some approaches.
                    # Max value is 255!

    'category'    , # The name of the category that this label belongs to

    'categoryId'  , # The ID of this category. Used to create ground truth images
                    # on category level.

    'hasInstances', # Whether this label distinguishes between single instances or not

    'ignoreInEval', # Whether pixels having this class as ground truth label are ignored
                    # during evaluations or not

    'color'       , # The color of this label
    ] )


#--------------------------------------------------------------------------------
# A list of all labels
#--------------------------------------------------------------------------------

# Please adapt the train IDs as appropriate for your approach.
# Note that you might want to ignore labels with ID 255 during training.
# Further note that the current train IDs are only a suggestion. You can use whatever you like.
# Make sure to provide your results using the original IDs and not the training IDs.
# Note that many IDs are ignored in evaluation and thus you never need to predict these!

labels = [
    #       name                     id    trainId   category            catId     hasInstances   ignoreInEval   color
    Label(  'unlabeled'            ,  0 ,      255 , 'void'            , 0       , False        , True         , (  0,  0,  0) ),
    Label(  'ego vehicle'          ,  1 ,      255 , 'void'            , 0       , False        , True         , (  0,  0,  0) ),
    Label(  'rectification border' ,  2 ,      255 , 'void'            , 0       , False        , True         , (  0,  0,  0) ),
    Label(  'out of roi'           ,  3 ,      255 , 'void'            , 0       , False        , True         , (  0,  0,  0) ),
    Label(  'static'               ,  4 ,      255 , 'void'            , 0       , False        , True         , (  0,  0,  0) ),
    Label(  'dynamic'              ,  5 ,      255 , 'void'            , 0       , False        , True         , (111, 74,  0) ),
    Label(  'ground'               ,  6 ,      255 , 'void'            , 0       , False        , True         , ( 81,  0, 81) ),
    Label(  'road'                 ,  7 ,        0 , 'flat'            , 1       , False        , False        , (128, 64,128) ),
    Label(  'sidewalk'             ,  8 ,        1 , 'flat'            , 1       , False        , False        , (244, 35,232) ),
    Label(  'parking'              ,  9 ,      255 , 'flat'            , 1       , False        , True         , (250,170,160) ),
    Label(  'rail track'           , 10 ,      255 , 'flat'            , 1       , False        , True         , (230,150,140) ),
    Label(  'building'             , 11 ,        2 , 'construction'    , 2       , False        , False        , ( 70, 70, 70) ),
    Label(  'wall'                 , 12 ,        3 , 'construction'    , 2       , False        , False        , (102,102,156) ),
    Label(  'fence'                , 13 ,        4 , 'construction'    , 2       , False        , False        , (190,153,153) ),
    Label(  'guard rail'           , 14 ,      255 , 'construction'    , 2       , False        , True         , (180,165,180) ),
    Label(  'bridge'               , 15 ,      255 , 'construction'    , 2       , False        , True         , (150,100,100) ),
    Label(  'tunnel'               , 16 ,      255 , 'construction'    , 2       , False        , True         , (150,120, 90) ),
    Label(  'pole'                 , 17 ,        5 , 'object'          , 3       , False        , False        , (153,153,153) ),
    Label(  'polegroup'            , 18 ,      255 , 'object'          , 3       , False        , True         , (153,153,153) ),
    Label(  'traffic light'        , 19 ,        6 , 'object'          , 3       , False        , False        , (250,170, 30) ),
    Label(  'traffic sign'         , 20 ,        7 , 'object'          , 3       , False        , False        , (220,220,  0) ),
    Label(  'vegetation'           , 21 ,        8 , 'nature'          , 4       , False        , False        , (107,142, 35) ),
    Label(  'terrain'              , 22 ,        9 , 'nature'          , 4       , False        , False        , (152,251,152) ),
    Label(  'sky'                  , 23 ,       10 , 'sky'             , 5       , False        , False        , ( 70,130,180) ),
    Label(  'person'               , 24 ,       11 , 'human'           , 6       , True         , False        , (220, 20, 60) ),
    Label(  'rider'                , 25 ,       12 , 'human'           , 6       , True         , False        , (255,  0,  0) ),
    Label(  'car'                  , 26 ,       13 , 'vehicle'         , 7       , True         , False        , (  0,  0, 142) ),
    Label(  'truck'                , 27 ,       14 , 'vehicle'         , 7       , True         , False        , (  0,  0, 70) ),
    Label(  'bus'                  , 28 ,       15 , 'vehicle'         , 7       , True         , False        , (  0, 60,100) ),
    Label(  'caravan'              , 29 ,      255 , 'vehicle'         , 7       , True         , True         , (  0,  0, 90) ),
    Label(  'trailer'              , 30 ,      255 , 'vehicle'         , 7       , True         , True         , (  0,  0,110) ),
    Label(  'train'                , 31 ,       16 , 'vehicle'         , 7       , True         , False        , (  0, 80,100) ),
    Label(  'motorcycle'           , 32 ,       17 , 'vehicle'         , 7       , True         , False        , (  0,  0,230) ),
    Label(  'bicycle'              , 33 ,       18 , 'vehicle'         , 7       , True         , False        , (119, 11, 32) ),
    Label(  'license plate'        , -1 ,       -1 , 'vehicle'         , 7       , False        , True         , (  0,  0,142) ),
]


def bc_img(img, s = 1.0, m = 0.0):
    img = img.astype(np.int)
    img = img * s + m
    img[img > 255] = 255
    img[img < 0] = 0
    img = img.astype(np.uint8)
    return img


def load_image(path):

    img = cv2.imread(path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (configs.img_width, configs.img_height))

    return img


def convert_class_to_rgb(image_labels, threshold=0.25):

    # convert any pixel > threshold to 1
    # convert any pixel < threshold to 0
    # then use bitwise_and

    output = np.zeros((configs.img_height, 1024, 3), dtype=np.uint8)

    for i in range(len(labels)):

        split = image_labels[:, :, i]
        split[split > threshold] = 1
        split[split < threshold] = 0
        split[:] *= 255
        split = split.astype(np.uint8)

        bg = np.zeros((configs.img_height, 2014, 3), dtype=np.uint8)
        bg[:, :, 0].fill(labels[i][2][0])
        bg[:, :, 1].fill(labels[i][2][1])
        bg[:, :, 2].fill(labels[i][2][2])

        res = cv2.bitwise_and(bg, bg, mask=split)

        output = cv2.addWeighted(output, 1.0, res, 1.0, 0)

    return output

# The new training generator
def generator(df, crop_shape, n_classes=34, batch_size=1, resize_shape=None, horizontal_flip=False,
                    vertical_flip=False, brightness=0.1, rotation=0.0, zoom=0.0, training=True):

    X = np.zeros((batch_size, crop_shape[1], crop_shape[0], 3), dtype='float32')
    Y1 = np.zeros((batch_size, crop_shape[1] // 4, crop_shape[0] // 4, n_classes), dtype='float32')

    while 1:
        j = 0

        for index in np.random.permutation(len(df)):

            image_path = df[index][0]
            label_path = df[index][1]
            image = cv2.imread(image_path, 1)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            label = cv2.imread(label_path, 0)

            # TODO: fix this stupid patch.
            # must rotate label...
            # label = cv2.rotate(label, cv2.ROTATE_90_COUNTERCLOCKWISE)

            if resize_shape:
                image = cv2.resize(image, resize_shape)
                label = cv2.resize(label, resize_shape)

            # Do augmentation (only if training)
            if training:
                if horizontal_flip and random.randint(0, 1):
                    image = cv2.flip(image, 1)
                    label = cv2.flip(label, 1)
                if vertical_flip and random.randint(0, 1):
                    image = cv2.flip(image, 0)
                    label = cv2.flip(label, 0)
                if brightness and random.randint(0, 1):
                    factor = 1.0 + abs(random.gauss(mu=0.0, sigma=brightness))
                    if random.randint(0, 1):
                        factor = 1.0 / factor
                    table = np.array([((i / 255.0) ** factor) * 255 for i in np.arange(0, 256)]).astype(np.uint8)
                    image = cv2.LUT(image, table)

                # get rotation or zoom
                if rotation and random.randint(0, 1):
                    angle = random.gauss(mu=0.0, sigma=rotation)
                else:
                    angle = 0.0
                if zoom and random.randint(0, 1):
                    scale = random.gauss(mu=1.0, sigma=zoom)
                else:
                    scale = 1.0

                # perform rotation or zoom
                if rotation or zoom:
                    M = cv2.getRotationMatrix2D((image.shape[1] // 2, image.shape[0] // 2), angle, scale)
                    image = cv2.warpAffine(image, M, (image.shape[1], image.shape[0]))
                    label = cv2.warpAffine(label, M, (label.shape[1], label.shape[0]))

            X[j] = image

            # only keep the useful classes
            # y1 = _filter_labels(to_categorical(cv2.resize(label, (label.shape[1] // 4, label.shape[0] // 4)), n_classes)).transpose()

            y1 = to_categorical(cv2.resize(label, (label.shape[1] // 4, label.shape[0] // 4)), n_classes)# .transpose()

            Y1[j] = y1

            j += 1
            if j == batch_size:
                break

        yield X, Y1


##############################################################
################ City Scape Generator ########################
##############################################################

# * Not working currently

class CityScapeGenerator(Sequence):

    def __init__(self, csv_path, mode='training', n_classes=34, batch_size=1, resize_shape=None, crop_shape=(640, 320),
                 horizontal_flip=False, vertical_flip=False, brightness=0.1, rotation=0.0, zoom=0.0):

        """
        Init method for the CityScape dataset generator. This can be used for
        Any type of Keras models (Not tested). Currently under development
        for ICNet architecture.

        :param csv_path: the path of the csv file which contains paths to labels
        :param mode: mode of the generator
        :param n_classes: number of classes in segmentation. CityScape default 34
        :param batch_size: generator batch size
        :param resize_shape: you can either resize the img or crop
        :param crop_shape: you can either resize the img or crop. cropping is random
        :param horizontal_flip: whether or not to perform hori flip
        :param vertical_flip: whether or not to perform vert flip
        :param brightness: for data augmentation. If != 0, adjust brightness of image.
        :param rotation: For data augmentation. If != 0, rotate input image.
        :param zoom: For data augmentation. If != 0, zooms in.

        """
        self.image_path_list, self.label_path_list = _load_data(csv_path)

        self.mode = mode
        self.n_classes = n_classes
        self.batch_size = batch_size
        self.resize_shape = resize_shape
        self.crop_shape = crop_shape
        self.horizontal_flip = horizontal_flip
        self.vertical_flip = vertical_flip
        self.brightness = brightness
        self.rotation = rotation
        self.zoom = zoom

        # Preallocate memory
        if mode == 'training' and self.crop_shape:
            self.X = np.zeros((batch_size, crop_shape[1], crop_shape[0], 3), dtype='float32')
            self.Y1 = np.zeros((batch_size, crop_shape[1] // 4, crop_shape[0] // 4, self.n_classes), dtype='float32')
            self.Y2 = np.zeros((batch_size, crop_shape[1] // 8, crop_shape[0] // 8, self.n_classes), dtype='float32')
            self.Y3 = np.zeros((batch_size, crop_shape[1] // 16, crop_shape[0] // 16, self.n_classes), dtype='float32')
        elif self.resize_shape:
            self.X = np.zeros((batch_size, resize_shape[1], resize_shape[0], 3), dtype='float32')
            self.Y1 = np.zeros((batch_size, resize_shape[1] // 4, resize_shape[0] // 4, self.n_classes), dtype='float32')
            self.Y2 = np.zeros((batch_size, resize_shape[1] // 8, resize_shape[0] // 8, self.n_classes), dtype='float32')
            self.Y3 = np.zeros((batch_size, resize_shape[1] // 16, resize_shape[0] // 16, self.n_classes), dtype='float32')
        else:
            raise Exception('No image dimensions specified!')

    def __len__(self):
        return len(self.image_path_list) // self.batch_size

    def __getitem__(self, i):

        for n, (image_path, label_path) in enumerate(zip(self.image_path_list[i * self.batch_size:(i + 1) * self.batch_size],
                                                         self.label_path_list[i * self.batch_size:(i + 1) * self.batch_size])):

            image = cv2.imread(image_path, 1)
            label = cv2.imread(label_path, 0)

            if self.resize_shape:
                image = cv2.resize(image, self.resize_shape)
                label = cv2.resize(label, self.resize_shape)

            # Do augmentation (only if training)
            if self.mode == 'training':
                if self.horizontal_flip and random.randint(0, 1):
                    image = cv2.flip(image, 1)
                    label = cv2.flip(label, 1)
                if self.vertical_flip and random.randint(0, 1):
                    image = cv2.flip(image, 0)
                    label = cv2.flip(label, 0)
                if self.brightness:
                    factor = 1.0 + abs(random.gauss(mu=0.0, sigma=self.brightness))
                    if random.randint(0, 1):
                        factor = 1.0 / factor
                    table = np.array([((i / 255.0) ** factor) * 255 for i in np.arange(0, 256)]).astype(np.uint8)
                    image = cv2.LUT(image, table)
                if self.rotation:
                    angle = random.gauss(mu=0.0, sigma=self.rotation)
                else:
                    angle = 0.0
                if self.zoom:
                    scale = random.gauss(mu=1.0, sigma=self.zoom)
                else:
                    scale = 1.0
                if self.rotation or self.zoom:
                    M = cv2.getRotationMatrix2D((image.shape[1] // 2, image.shape[0] // 2), angle, scale)
                    image = cv2.warpAffine(image, M, (image.shape[1], image.shape[0]))
                    label = cv2.warpAffine(label, M, (label.shape[1], label.shape[0]))
                if self.crop_shape:
                    image, label = _random_crop(image, label, self.crop_shape)

            self.X[n] = image
            # only keep the useful classes

            y1 = _filter_labels(to_categorical(cv2.resize(label, (label.shape[1] // 4, label.shape[0] // 4)), self.n_classes)).transpose()
            y2 = _filter_labels(to_categorical(cv2.resize(label, (label.shape[1] // 8, label.shape[0] // 8)), self.n_classes)).transpose()
            y3 = _filter_labels(to_categorical(cv2.resize(label, (label.shape[1] // 16, label.shape[0] // 16)), self.n_classes)).transpose()

            self.Y1[n] = y1.reshape((label.shape[0] // 4, label.shape[1] // 4, -1))
            self.Y2[n] = y2.reshape((label.shape[0] // 8, label.shape[1] // 8, -1))
            self.Y3[n] = y3.reshape((label.shape[0] // 16, label.shape[1] // 16, -1))

        return self.X, [self.Y1, self.Y2, self.Y3]

    def on_epoch_end(self):
        # Shuffle dataset for next epoch
        c = list(zip(self.image_path_list, self.label_path_list))
        random.shuffle(c)
        self.image_path_list, self.label_path_list = zip(*c)

        # Fix memory leak (Keras bug)
        gc.collect()


class Visualization(Callback):

    def __init__(self, resize_shape=(640, 320), batch_steps=10, n_gpu=1, **kwargs):
        super(Visualization, self).__init__(**kwargs)
        self.resize_shape = resize_shape
        self.batch_steps = batch_steps
        self.n_gpu = n_gpu
        self.counter = 0

        # TODO: Remove this lazy hardcoded paths
        self.test_images_list = glob.glob('datasets/mapillary/testing/images/*')
        with open('datasets/mapillary/config.json') as config_file:
            config = json.load(config_file)
        self.labels = config['labels']

    def on_batch_end(self, batch, logs={}):
        self.counter += 1

        if self.counter == self.batch_steps:
            self.counter = 0

            test_image = cv2.resize(cv2.imread(random.choice(self.test_images_list), 1), self.resize_shape)

            inputs = [test_image] * self.n_gpu
            output, _, _ = self.model.predict(np.array(inputs), batch_size=self.n_gpu)

            cv2.imshow('input', test_image)
            cv2.waitKey(1)
            cv2.imshow('output', apply_color_map(np.argmax(output[0], axis=-1), self.labels))
            cv2.waitKey(1)


class PolyDecay:
    def __init__(self, initial_lr, power, n_epochs):
        self.initial_lr = initial_lr
        self.power = power
        self.n_epochs = n_epochs

    def scheduler(self, epoch):
        return self.initial_lr * np.power(1.0 - 1.0 * epoch / self.n_epochs, self.power)


class ExpDecay:
    def __init__(self, initial_lr, decay):
        self.initial_lr = initial_lr
        self.decay = decay

    def scheduler(self, epoch):
        return self.initial_lr * np.exp(-self.decay * epoch)


# Taken from Mappillary Vistas demo.py
def apply_color_map(image_array, labels):
    color_array = np.zeros((image_array.shape[0], image_array.shape[1], 3), dtype=np.uint8)

    for label_id, label in enumerate(labels):
        # set all pixels with the current label to the color of the current label
        color_array[image_array == label_id] = label["color"]

    return color_array


def _random_crop(image, label, crop_shape):
    if (image.shape[0] != label.shape[0]) or (image.shape[1] != label.shape[1]):
        raise Exception('Image and label must have the same dimensions!')

    if (crop_shape[0] < image.shape[1]) and (crop_shape[1] < image.shape[0]):
        x = random.randrange(image.shape[1] - crop_shape[0])
        y = random.randrange(image.shape[0] - crop_shape[1])

        return image[y:y + crop_shape[1], x:x + crop_shape[0], :], label[y:y + crop_shape[1], x:x + crop_shape[0]]
    else:
        raise Exception('Crop shape exceeds image dimensions!')


def load_data():

    labels = pandas.read_csv(configs.labelid_path).values
    df = []
    count = 0
    for row in labels:
        if os.path.isfile(row[0]) and os.path.isfile(row[1]):
            count = count + 1
            df.append(row)

    print("data processing finished")
    print("data frame size: " + str(count))

    return df

def _load_data(csv_path):

    labels = pandas.read_csv(csv_path)
    img_list_initial = labels[labels.columns[0]].values
    label_list_initial = labels[labels.columns[0]].values

    img_list = []
    label_list = []
    count = 0
    for i in range(len(img_list)):
        if os.path.isfile(img_list_initial[i]) and os.path.isfile(label_list_initial[i]):
            count = count + 1
            img_list.append(img_list_initial[i])
            label_list.append(label_list_initial[i])

    print("data processing finished")
    print("data frame size: " + str(count))

    return img_list, label_list


def _filter_labels(categorical_labels):

    new_label = np.stack((categorical_labels[:, :, 0],
                          categorical_labels[:, :, 6],
                          categorical_labels[:, :, 7],
                          categorical_labels[:, :, 8],
                          categorical_labels[:, :, 11],
                          categorical_labels[:, :, 17],
                          categorical_labels[:, :, 21],
                          categorical_labels[:, :, 22],
                          categorical_labels[:, :, 23],
                          categorical_labels[:, :, 24],
                          categorical_labels[:, :, 26],
                          categorical_labels[:, :, 32],
                          categorical_labels[:, :, 33]))
    return new_label