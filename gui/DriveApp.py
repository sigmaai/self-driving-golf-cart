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
        # self.parent.geometry('{}x{}'.format(430, 350))
        tk.Label(self, text="Welcome to the autnomous vehicle program").grid(row=0, sticky=tk.W)

        # ---- drop down menu ------
        tk.Label(self, text="Please select steering model type", fg="blue").grid(row=1, sticky=tk.W)
        self.var = tk.StringVar(self)
        self.var.set("Own")
        self.option = tk.OptionMenu(self, self.var, "Own", "Komanda", "Autumn", "AutoPilot").grid(row=2, pady=10)

        tk.Label(self, text="Steering Serial Port").grid(row=3, column=0, sticky=tk.W)
        tk.Label(self, text="Cruise Control Serial Port").grid(row=4, column=0, sticky=tk.W)

        self.entry1 = tk.Entry(self).grid(row=3, column=1, sticky=tk.W)
        self.entry2 = tk.Entry(self).grid(row=4, column=1, sticky=tk.W)

        # # -----------
        tk.Label(self, text="Please specify run configeration", fg="blue").grid(row=5, column=0, pady=10)
        self.cc_bool = tk.IntVar()
        self.cb1 = tk.Checkbutton(self, text="Cruise Control", variable=self.cc_bool).grid(row=6, column=0, sticky=tk.W)

        self.seg_vis = tk.IntVar()
        self.seg_vis.set(1)
        self.cb2 = tk.Checkbutton(self, text="Segmentation Visualization", variable=self.seg_vis).grid(row=6, column=1, sticky=tk.W)

        self.detc_bool = tk.IntVar()
        self.detc_bool.set(1)
        self.cb3 = tk.Checkbutton(self, text="Object Detection", variable=self.detc_bool).grid(row=7, column=0, sticky=tk.W)

        self.gps_bool = tk.IntVar()
        self.cb5 = tk.Checkbutton(self, text="GPS", variable=self.gps_bool).grid(row=7, column=1, sticky=tk.W)

        # # --------
        self.beginButton = tk.Button(self, text="Begin", command=self.begin).grid(row=8, column=0, padx=10, pady=10)
        #
        tk.Label(self, text="Once the program has started: ").grid(row=9, column=0, sticky=tk.W)
        tk.Label(self, text="- Press 'a' to disable braking").grid(row=10, column=0, sticky=tk.W)
        tk.Label(self, text="- Press 'h' to visualize steering heat map.").grid(row=11, column=0, sticky=tk.W)

    def begin(self):

        if self.steering_model != None:

            self.driver = Driver(steering_model=self.var.get(),
                                 cruise_control=self.cc_bool.get(),
                                 seg_vis=self.seg_vis.get(),
                                 obj_det=self.detc_bool.get(),
                                 gps=self.gps_bool.get())


            self.driver.drive()
        else:
            messagebox.showerror("Error", "Please select a steering model and click OK")


    def create_label(self, str):

        string = tk.StringVar()
        label = tk.Label(self, textvariable=string, relief=tk.FLAT)
        string.set(str)
        label.pack()

        return label

