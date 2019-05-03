#!/usr/bin/python

import tkinter
import subprocess
import os
# Here, we are creating our class, Window, and inheriting from the Frame
# class. Frame is a class from the tkinter module. (see Lib/tkinter/__init__)


class Window(tkinter.Frame):

    # Define settings upon initialization. Here you can specify
    def __init__(self, master=None):
        # parameters that you want to send through the Frame class.
        tkinter.Frame.__init__(self, master)

        # reference to the master widget, which is the tk window
        self.master = master

        # with that, we want to then run init_window, which doesn't yet exist

        self.project_path = None
        self.init_window()


    # Creation of init_window
    def init_window(self):
        # changing the title of our master widget
        self.master.title("self-driving golf cart")

        # allowing the widget to take the full space of the root window
        self.pack(fill=tkinter.BOTH, expand=1)

        T = tkinter.Entry(self.master)
        T.insert(0, "/home/neil/Workspace/self-driving-golf-cart")
        T.pack()
        T.place(x=300, y=45, anchor="center")
        self.project_path = T.get()

        # creating a button instance
        quitButton = tkinter.Button(self, text="Compile Project", command=self.compile_project)
        quitButton.place(x=300, y=75, anchor="center")

        welcomeLabel = tkinter.Label(self.master, text="Welcome to the self-driving golf cart project")
        welcomeLabel.place(x=300, y=20, anchor="center")

    def compile_project(self):
        print self.project_path
        os.system("cd {}/ros && catkin_make".format(self.project_path))
        os.system(". {}/ros/devel/setup.bash".format(self.project_path))
        print ("Finished Compiling")

    def client_exit(self):
        exit()


# root window created. Here, that would be the only window, but
# you can later have windows within windows.
root = tkinter.Tk()

root.geometry("600x500")

# creation of an instance
app = Window(root)

# mainloop
root.mainloop()