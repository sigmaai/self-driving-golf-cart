#
# About:
# If you want to start the self driving car program,
# run this file.
# Developed by Neil Nie | (c) 2018, MIT License
# All Rights Reserved
#
# For research and development purposes only
# you are responsible for your own safety.
# By downloading and running the code, you
# have agreed to the terms and conditions.
# 

from gui.DriveApp import DriverApp
import tkinter as tk

if __name__ == "__main__":

    root = tk.Tk()
    DriverApp(root).pack(side="top", fill="both", expand=True)
    root.mainloop()