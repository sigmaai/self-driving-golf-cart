import tkinter as tk
from tkinter import messagebox
from driver import Driver


class DriverApp(tk.Frame):

    def __init__(self, parent, *args, **kwargs):

        tk.Frame.__init__(self, parent, *args, **kwargs)
        self.parent = parent

        # some basic setup
        self.winfo_toplevel().title("self driving golf cart")
        self.parent.resizable(width=False, height=False)
        self.parent.geometry('{}x{}'.format(300, 350))
        # self.parent.iconbitmap(r'/home/neil/Workspace/self-driving-golf-cart/media/icon.ico')
        self.create_label("Welcome to the autnomous vehicle program")

        # ---- drop down menu ------
        self.create_label("Please select steering model type")
        self.var = tk.StringVar(self)
        self.var.set("Own")
        self.option = tk.OptionMenu(self, self.var, "Own", "Komanda", "Autumn", "AutoPilot")
        self.option.pack()
        self.button = tk.Button(self, text="OK", command=self.set_steering_model, bg="blue")
        self.button.pack()

        # -----------
        self.create_label("Please specify run configeration")
        self.cc_bool = tk.IntVar()
        self.cb1 = tk.Checkbutton(self, text="Cruise Control", variable=self.cc_bool)
        self.cb1.pack()

        self.seg_vis = tk.IntVar()
        self.seg_vis.set(1)
        self.cb2 = tk.Checkbutton(self, text="Segmentation Visualization", variable=self.seg_vis)
        self.cb2.pack()

        self.detc_bool = tk.IntVar()
        self.detc_bool.set(1)
        self.cb3 = tk.Checkbutton(self, text="Object Detection", variable=self.detc_bool)
        self.cb3.pack()

        self.detc_vis = tk.IntVar()
        self.detc_vis.set(1)
        self.cb4 = tk.Checkbutton(self, text="Detection Visualization", variable=self.detc_vis)
        self.cb4.pack()

        self.gps_bool = tk.IntVar()
        self.cb5 = tk.Checkbutton(self, text="GPS", variable=self.gps_bool)
        self.cb5.pack()

        # --------
        self.beginButton = tk.Button(self, text="Begin", command=self.begin)
        self.beginButton.pack()

        # -- init driver -----
        self.steering_model = None


    def begin(self):

        if self.steering_model != None:

            self.driver = Driver(steering_model=self.steering_model,
                                 cruise_control=self.cc_bool.get(),
                                 seg_vis=self.seg_vis.get(),
                                 obj_det=self.detc_bool.get(),
                                 det_vis=self.detc_vis.get(),
                                 gps=self.gps_bool.get())


            self.driver.drive()
        else:
            messagebox.showerror("Error", "Please select a steering model and click OK")

    def set_steering_model(self):
        self.steering_model = self.var.get()
        print(self.steering_model)

    def create_label(self, str):
        string = tk.StringVar()
        label = tk.Label(self, textvariable=string, relief=tk.FLAT)
        string.set(str)
        label.pack()

