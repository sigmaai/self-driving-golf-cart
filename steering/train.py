import pandas as pd
import cv2
import numpy as np
import train_util
import model

def clean_steering_label(steering_labels):
    
    center_labels = np.array([steering_labels[2]])
    print("input length")
    print(len(steering_labels))
    
    for i in range(2, len(steering_labels)):
        
        if steering_labels[i][4] == "center_camera":
            item = np.array([steering_labels[i]])
            center_labels = np.concatenate((center_labels, item), axis=0)
        if (i%10000) == 0:
            print(".")
    print(center_labels.shape)
    return center_labels
            

if __name__ == "__main__":

    dir = "/home/ubuntu/dataset/udacity-driving-testing-ds/"
    val_dir = "/home/ubuntu/dataset/small-testing-ds/"
    
    labels = pd.read_csv("/home/ubuntu/dataset/udacity-driving-testing-ds/interpolated.csv")
    val_labels = pd.read_csv(val_dir + "interpolated.csv")
    center_labels = clean_steering_label(labels.values)
    print(center_labels.shape)
        
    cnn = model.small_vgg_network()
    # cnn.load_weights("./trained5-v3.h5")
    cnn.summary()

    training_gen = train_util.batch_generator(dir, center_labels, 4, True)
    validation_gen = train_util.validation_generator(val_dir, val_labels, 2)
    
    cnn.fit_generator(training_gen,
                        steps_per_epoch=3000, epochs=8, verbose=1,
                        validation_data=validation_gen,
                        validation_steps=600)

    model.save('trained-vgg-v1.h5')

