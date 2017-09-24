import pandas as pd
import os, sys
import cv2
import numpy as np
import densenet

# parameter
batch_size = 16
epochs = 5
dropout_rate = 0.2
leaky = 0.1

# dataset
os.chdir("/home/ubuntu/dataset/udacity-driving")
training_set = pd.read_csv('interpolated.csv')
train_batch_num = int(len(training_set) / batch_size)

# generator
from sklearn.utils import shuffle
def generate_train_batch(data, batch_size = 16):
    img_rows = 480
    img_cols = 640
    
    batch_images = np.zeros((batch_size, img_rows, img_cols, 3))
    angles = np.zeros((batch_size, 1))
    while 1:
        shuffle(data)
        for i_batch in range(0,len(data),batch_size):
            for i in range(batch_size):
                i_line = i_batch + i
                
                # get image & label
                file_name = data.iloc[i_line]["filename"]
                img = cv2.imread(file_name)
                f = float(data.iloc[i_line]["angle"]) # float( * 180.00 / 3.14159265359 )

                batch_images[i] = img
                angles[i] = f
            yield batch_images, angles
            
train_generator = generate_train_batch(training_set,batch_size)

# build model
from keras.optimizers import Adam
from keras.models import Sequential
from keras.layers import Dense, Flatten, Dropout, Lambda, Cropping2D, Convolution2D,MaxPooling2D
from keras.layers.advanced_activations import LeakyReLU
model = Sequential()
model.add(Reshape((120,160,3), input_shape=(480,640,3))
model.add(Lambda(lambda x: x / 255.0 - 0.5))

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

model.add(Flatten())
model.add(Dense(512,activation='relu'))
model.add(Dropout(dropout_rate))
model.add(Dense(512,activation='relu'))
model.add(Dense(1))
model.summary()
#model = densenet.DenseNet(nb_classes=1, img_dim=(480,640,3), depth=10, nb_dense_block=3, growth_rate=12, nb_filter=32, dropout_rate=dropout_rate, weight_decay=1E-4)
#optimizer = Adam(lr=1e-3) # Using Adam instead of SGD to speed up training
#model.compile(loss='mse', optimizer=optimizer, metrics=['mse', 'acc'])

# train
#model.fit_generator(train_generator, steps_per_epoch=train_batch_num, epochs=epochs, verbose=1)
#model.save('my_model_2.h5') 
