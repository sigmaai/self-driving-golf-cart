import numpy as np
import pandas as pd
import pygame
import glob
import cv2
from keras.models import load_model

BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
BLUE =  (  0,   0, 255)
GREEN = (  0, 255,   0)
RED =   (255,   0,   0)

data_path = "/Volumes/Personal_Drive/Datasets/Udacity_Self-Driving-Car/small-testing-ds/"
true = pd.read_csv("/Volumes/Personal_Drive/Datasets/Udacity_Self-Driving-Car/small-testing-ds/interpolated.csv")
model_path = "/Users/yongyangnie/Documents/Developer/ALVNS/driving-simulator/trained5-v2.h5"

model = load_model(model_path)
model.compile("sgd", "mse")
model.summary()

pygame.init()
size = (640, 480)
pygame.display.set_caption("Data viewer")
screen = pygame.display.set_mode(size, pygame.DOUBLEBUF)
myfont = pygame.font.SysFont("monospace", 17)



for i in range(5000):

    for event in pygame.event.get():
            if event.type == pygame.QUIT:
                break

    path = data_path + "center/" + str(true["frame_id"][i]) + ".jpg"
    img = cv2.imread(path)
    b, g, r = cv2.split(img)  # get b,g,r
    img = cv2.merge([r, g, b])  # switch it to rgb
    image = np.array([img])  # the model expects 4D array

    angle = model.predict(image)[0][0] # radians
    true_angle = true["steering_angle"].iloc[i] # radians

    # add image to screen
    img = pygame.image.load(path)
    screen.blit(img, (0, 0))
    
    # add text
    pred_txt = myfont.render("Prediction:" + str(round(angle* 57.2958, 3)), 1, (255,255,0)) # angle in degrees
    true_txt = myfont.render("True angle:" + str(round(true_angle * 57.2958, 3)), 1, (255,255,0)) # angle in degrees
    screen.blit(pred_txt, (10, 280))
    screen.blit(true_txt, (10, 300))

    # draw steering wheel
    radius = 50
    pygame.draw.circle(screen, WHITE, [320, 300], radius, 2) 

    # draw cricle for true angle
    x = radius * np.cos(np.pi/2 + true_angle)
    y = radius * np.sin(np.pi/2 + true_angle)
    pygame.draw.circle(screen, WHITE, [320 + int(x), 300 - int(y)], 7)
    
    # draw cricle for predicted angle
    x = radius * np.cos(np.pi/2 + angle)
    y = radius * np.sin(np.pi/2 + angle)
    pygame.draw.circle(screen, BLACK, [320 + int(x), 300 - int(y)], 5) 
    
    pygame.display.flip()
    pygame.time.Clock().tick(60)
    