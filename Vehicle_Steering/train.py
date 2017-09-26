import densenet
import cv2
import numpy as np
import pandas as pd
from sklearn.model_selection import train_test_split
import keras as K
from keras.models import load_model

from keras.optimizers import Adam

def rotate(img):
    row, col, channel = img.shape
    angle = np.random.uniform(-15, 15)
    rotation_point = (row / 2, col / 2)
    rotation_matrix = cv2.getRotationMatrix2D(rotation_point, angle, 1)
    rotated_img = cv2.warpAffine(img, rotation_matrix, (col, row))
    return rotated_img


def gamma(img):
    gamma = np.random.uniform(0.5, 1.2)
    invGamma = 1.0 / gamma
    table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    new_img = cv2.LUT(img, table)
    return new_img


def blur(img):
    r_int = np.random.randint(0, 2)
    odd_size = 2 * r_int + 1
    return cv2.GaussianBlur(img, (odd_size, odd_size), 0)

def random_transform(img):
    # There are total of 3 transformation
    # I will create an boolean array of 3 elements [ 0 or 1]
    a = np.random.randint(0, 2, [1, 3]).astype('bool')[0]
    if a[0] == 1:
        img = rotate(img)
    if a[1] == 1:
        img = blur(img)
    if a[2] == 1:
        img = gamma(img)
    return img


def generate_train_batch(data, batch_size=16):
    
    img_rows = 480
    img_cols = 640

    batch_images = np.zeros((batch_size, img_rows, img_cols, 3))
    angles = np.zeros((batch_size, 1))
    
    while 1:
        for i_batch in range(batch_size):
            i_line = np.random.randint(len(data))

            # get image & label
            file_name = data.iloc[i_line]["filename"]
            img = cv2.imread("/home/ubuntu/dataset/udacity-driving/" + file_name)
            f = float(data.iloc[i_line]["angle"])  # float( * 180.00 / 3.14159265359 )

            i = np.random.randint(1)
            if i == 0:
                flipped_image = cv2.flip(img, 1)
                f = f * -1.0
                img = random_transform(flipped_image)

            batch_images[i_batch] = img
            angles[i_batch] = f
        yield batch_images, angles

def get_dataset(path):
    steering_labels = pd.read_csv(path + "/interpolated.csv")
    print(steering_labels.shape)
    # steering_labels.head()
    return steering_labels

def train_test_split(data):

    x = data["filename"]
    y = data["angle"]

def root_mean_squared_error(y_true, y_pred):
        return K.backend.sqrt(K.backend.mean(K.backend.square(y_pred - y_true), axis=-1)) 
    
if __name__ == "__main__":

    train_data = get_dataset("/home/ubuntu/dataset/udacity-driving/")
    print(train_data.shape)
    print(len(train_data))
    depth = 10
    growth_rate = 12
    nb_filters = 24 # (2 * growth_rate)
    model = densenet.DenseNet(nb_classes=1, img_dim=(480, 640, 3), depth = depth, nb_dense_block = 3, growth_rate = growth_rate, nb_filter = nb_filters, dropout_rate=0.50)
    model.summary()


    model.compile(optimizer = "rmsprop", loss = root_mean_squared_error, 
              metrics =["accuracy"])

    train_generator = generate_train_batch(train_data, 1)
    history = model.fit_generator(train_generator, steps_per_epoch=500, epochs=10, verbose=1)
    
    

    model.save('my_model.h5')  # creates a HDF5 file 'my_model.h5'