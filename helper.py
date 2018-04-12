from termcolor import colored
from configs import configs
import os
import time

def get_destination():
    var = input(colored("Please enter your destination:", "yellow"))
    return str(var)


def get_serial_port():
    var = input(colored("Please enter serial port number: ", "yellow"))
    return int(var)


def welcome():
    os.system("clear")
    print(colored("Welcome to the self-driving golf cart program", "green"))
    print(colored("By Michael Meng & Neil Nie", "green"))
    print(colored("Thanks for driving with us", "green"))
    print(colored("v.0.2.1", "green"))
    print(colored("---------------------------------------------", "green"))
    time.sleep(2)


def print_configs():

    print(colored("configs: ", "yellow"))
    print(colored("steering factor: {}".format(configs.st_fac), "yellow"))
    print(colored("image size: {}".format(configs.default_img_size), "yellow"))
    print(colored("segmentation size: {}".format(configs.segmentation_size), "yellow"))
    print(colored("-----------------------------", "yellow"))


def seg_init_response():
    # print(colored("-----------------", "green"))
    print(colored("segmentor created", "green"))
    # print(colored("-----------------", "green"))


def steering_init_response(model):
    print(colored("steering model ready", "green"))
    # print(colored(model.summary(), "green"))
