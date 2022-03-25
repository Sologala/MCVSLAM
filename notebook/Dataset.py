import numpy as np
import cv2
import glob 
import os
import random


class Dataset:
    def __init__(self,dataset_folder: str):
        self.data_folder = dataset_folder
        self.left_img_path = os.path.join(self.data_folder, "left")
        self.right_img_path = os.path.join(self.data_folder, "right")
        self.wide_img_path = os.path.join(self.data_folder, "wide")
        self.all_timstamp = self.load_all_time_stampp(os.path.join(self.left_img_path, "*.png"))
        
    def load_all_time_stampp(self,filepath):
        time_stamps = glob.glob(filepath)
        ts = [t.split("/")[-1].split(".")[0] for t in time_stamps]
        return ts
    
    def get_random_image(self):
        t = self.all_timstamp[random.randint(0, len(self.all_timstamp))]
        imgleft = cv2.imread(self.left_img_path +  "/{}.png".format(t))
        imgright = cv2.imread(self.right_img_path +  "/{}.png".format(t))
        imgwide = cv2.imread(self.wide_img_path +  "/{}.png".format(t))
        return imgleft, imgright, imgwide