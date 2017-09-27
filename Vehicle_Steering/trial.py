import csv

import cv2
import numpy as np

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

from keras.models import Sequential
from keras.layers import Reshape, Dense, Flatten, Dropout, Lambda, Cropping2D, Convolution2D,MaxPooling2D
from keras.layers.advanced_activations import LeakyReLU

# parameter
batch_size = 16
epochs = 5
dropout_rate = 0.2
leaky = 0.1

def get_train_valid_arr(path):
    arr = []
    with open(path) as f:
        reader = csv.reader(f)
        next(reader)
        for line in reader:
            arr.append(line)
    arr = arr[:data_range]
    train_arr, valid_arr = train_test_split(arr, test_size=0.2)
    return train_arr, valid_arr

def generator(arr):
    num = len(arr)
    while True:
        shuffle(arr)
        for i in range(0, num, batch_size):
            batch_lines = arr[i:i + batch_size]

            images = []
            steerings = []
            for line in batch_lines:
                center_path = line[0]
                steering = float(line[3])
                image = cv2.imread(center_path)
                images.append(preprocess(image))
                steerings.append(steering)

                image_flip=np.fliplr(image)
                images.append(preprocess(image_flip))
                steerings.append(-steering)

            X_train = np.array(images)
            y_train = np.array(steerings)

            yield shuffle(X_train, y_train)

def preprocess(img):
    return cv.resize(img,(0,0),fx=0.5,fy=0.5)

model = Sequential()
model.add(Lambda(lambda x: x / 255.0 - 0.5,input_shape=(240,320,3)))

relu1=LeakyReLU(alpha=leaky)
model.add(Convolution2D(16,(5,5),strides=(1,1),padding='valid'))
model.add(relu1)
model.add(Dropout(dropout_rate))
model.add(MaxPooling2D((2,2),strides=(2,2)))

relu2 = LeakyReLU(alpha=leaky)
model.add(Convolution2D(24, (5, 5), strides=(1, 1), padding='valid'))
model.add(relu2)
model.add(Dropout(dropout_rate))
model.add(MaxPooling2D((2, 2), strides=(2, 2)))

relu3 = LeakyReLU(alpha=leaky)
model.add(Convolution2D(32, (5, 5), strides=(1, 1), padding='valid'))
model.add(relu3)
model.add(Dropout(dropout_rate))
model.add(MaxPooling2D((2, 2), strides=(2, 2)))

relu4 = LeakyReLU(alpha=leaky)
model.add(Convolution2D(48, (5, 5), strides=(1, 1), padding='valid'))
model.add(relu4)
model.add(Dropout(dropout_rate))
model.add(MaxPooling2D((2, 2), strides=(2, 2)))

relu5 = LeakyReLU(alpha=leaky)
model.add(Convolution2D(60, (5, 5), strides=(1, 1), padding='valid'))
model.add(relu5)
model.add(Dropout(dropout_rate))
model.add(MaxPooling2D((2, 2), strides=(2, 2)))

model.add(Flatten())
model.add(Dense(256,activation='relu'))
model.add(Dropout(dropout_rate))
model.add(Dense(256,activation='relu'))
model.add(Dense(1))
model.summary()

#optimizer = Adam(lr=1e-3) # Using Adam instead of SGD to speed up training
#model.compile(loss='mse', optimizer=optimizer, metrics=['mse', 'acc'])

# train
#model.fit_generator(train_generator, steps_per_epoch=train_batch_num, epochs=epochs, verbose=1)
#model.save('my_model_2.h5') 
